#!/usr/bin/env python3
"""Concurrent action integration test with cubic-interpolated mock controllers.

Uses an isolated ROS domain; does not connect to hardware or MuJoCo.
"""
import math
import os
import re
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from moveit_msgs.msg import Constraints, JointConstraint, PlanningScene, PlanningSceneWorld, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from multi_arm_motion_interfaces.action import ExecuteMotion
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import yaml
from slider_smoke import MockDriver, robot_config


def seconds(t):
    return t.sec + t.nanosec * 1e-9


class CubicDriver(MockDriver):
    def __init__(self):
        self.overlap_samples = 0
        self.publish_states = True
        self.continuous_desired = os.environ.get('MPC_CONTINUOUS_DESIRED_TEST') == '1'
        super().__init__()
        self.desired_q, self.desired_v = list(self.q), list(self.v)
        self.last_tick = self.get_clock().now().nanoseconds*1e-9

    def receive(self, msg):
        # Match JTC's default replacement semantics: it starts interpolating
        # immediately from feedback to a future first sample, rather than
        # retaining the old trajectory until that sample's timestamp.
        self.before_time = self.get_clock().now().nanoseconds*1e-9
        self.before_q, self.before_v = (list(self.desired_q), list(self.desired_v)) if self.continuous_desired else (list(self.q), list(self.v))
        self.command = msg
        self.command_start = seconds(msg.header.stamp) or self.before_time
        self.commands += 1

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        actual_q = list(self.q)
        if self.continuous_desired:
            self.q, self.v = list(self.desired_q), list(self.desired_v)
        if self.command:
            t = self.get_clock().now().nanoseconds * 1e-9 - self.command_start
            points = self.command.points
            if t < 0:
                dt = self.command_start-self.before_time
                if dt > 0:
                    u = max(0., min(1., (now-self.before_time)/dt))
                    self.q = [(2*u**3-3*u**2+1)*x+(u**3-2*u**2+u)*dt*vx +
                              (-2*u**3+3*u**2)*y+(u**3-u**2)*dt*vy
                              for x,vx,y,vy in zip(self.before_q,self.before_v,points[0].positions,points[0].velocities)]
                    self.v = [(6*u*u-6*u)/dt*x+(3*u*u-4*u+1)*vx +
                              (-6*u*u+6*u)/dt*y+(3*u*u-2*u)*vy
                              for x,vx,y,vy in zip(self.before_q,self.before_v,points[0].positions,points[0].velocities)]
            elif t >= seconds(points[-1].time_from_start):
                self.q, self.v = list(points[-1].positions), list(points[-1].velocities)
            elif t >= 0:
                for a, b in zip(points, points[1:]):
                    ta, tb = seconds(a.time_from_start), seconds(b.time_from_start)
                    if ta <= t < tb:
                        u, dt = (t-ta)/(tb-ta), tb-ta
                        self.q = [(2*u**3-3*u**2+1)*x+(u**3-2*u**2+u)*dt*vx +
                                  (-2*u**3+3*u**2)*y+(u**3-u**2)*dt*vy
                                  for x, vx, y, vy in zip(a.positions, a.velocities, b.positions, b.velocities)]
                        self.v = [(6*u*u-6*u)/dt*x+(3*u*u-4*u+1)*vx +
                                  (-6*u*u+6*u)/dt*y+(3*u*u-2*u)*vy
                                  for x, vx, y, vy in zip(a.positions, a.velocities, b.positions, b.velocities)]
                        break
        if self.continuous_desired:
            self.desired_q, self.desired_v = list(self.q), list(self.v)
            dt = max(1e-6, now-self.last_tick)
            alpha = 1.-math.exp(-dt/.04)
            self.q = [a+alpha*(b-a) for a,b in zip(actual_q,self.desired_q)]
            self.v = [(b-a)/dt for a,b in zip(actual_q,self.q)]
        self.last_tick = now
        if abs(self.v[0]) > .001 and abs(self.v[1]) > .001:
            self.overlap_samples += 1
        for i in range(3):
            for j in range(i+1, 3):
                self.minimum_clearance = min(self.minimum_clearance, abs(i*.5+self.q[i]-j*.5-self.q[j])-.2)
        if self.publish_states:
            from sensor_msgs.msg import JointState
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name, msg.position, msg.velocity = ['j0', 'j1', 'j2'], self.q, self.v
            self.pub.publish(msg)


def goal(group, value, timeout=30.):
    result = ExecuteMotion.Goal()
    result.request.group_name = f'group{group}'
    result.request.allowed_planning_time = 1.
    result.request.max_velocity_scaling_factor = 1.
    result.request.max_acceleration_scaling_factor = 1.
    result.request.goal_constraints = [Constraints(joint_constraints=[JointConstraint(
        joint_name=f'j{group}', position=value, tolerance_above=.0001, tolerance_below=.0001, weight=1.)])]
    result.timeout = timeout
    return result


def main():
    if os.environ.get('ROS_DOMAIN_ID') in (None, '', '0', '11', '91'):
        raise RuntimeError('An unused ROS_DOMAIN_ID is required')
    rclpy.init()
    node = CubicDriver()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    directory = Path(tempfile.mkdtemp(prefix='multi_arm_concurrent_'))
    config = robot_config()
    config['/**']['ros__parameters']['mpc']['optimization_budget'] = .015
    config['/**']['ros__parameters']['execution'] = {'command_lead': .04}
    if os.environ.get('MPC_REFERENCE_PROGRESS_TEST') == '1':
        config['/**']['ros__parameters']['execution']['reference_lag_tolerance'] = .01
    if node.continuous_desired:
        config['/**']['ros__parameters']['execution'].update({
            'continuous_desired_commands': True, 'joint_tolerance': .0002, 'stopped_velocity': .0005})
    (directory/'robot.yaml').write_text(yaml.safe_dump(config))
    process = None

    def spin(duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.005)

    def wait(future, timeout=30.):
        end = time.monotonic()+timeout
        while not future.done() and time.monotonic() < end:
            executor.spin_once(timeout_sec=.005)
        if not future.done():
            raise TimeoutError('ROS action/service response timeout')
        return future.result()

    def call(name):
        service = node.create_client(Trigger, name)
        assert service.wait_for_service(timeout_sec=5)
        result = wait(service.call_async(Trigger.Request()))
        node.destroy_client(service)
        assert result.success, result.message

    try:
        with (directory/'coordinator.log').open('w') as log:
            process = subprocess.Popen(['ros2', 'run', 'multi_arm_hybrid_planning', 'coordinator',
                '--ros-args', '--params-file', str(directory/'robot.yaml')], stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
            client = ActionClient(node, ExecuteMotion, '/execute_motion')
            end = time.monotonic()+20
            while not client.server_is_ready() and time.monotonic() < end:
                spin(.05)
                if process.poll() is not None:
                    raise RuntimeError((directory/'coordinator.log').read_text())
            assert client.server_is_ready()
            spin(1)
            a, b = client.send_goal_async(goal(0, -.07)), client.send_goal_async(goal(1, .07))
            ha, hb = wait(a), wait(b)
            assert ha.accepted and hb.accepted
            # Ownership is reserved before global planning begins.
            duplicate = wait(client.send_goal_async(goal(0, -.02)))
            assert not duplicate.accepted, 'overlapping goals must be rejected'
            fa, fb = ha.get_result_async(), hb.get_result_async()
            ra, rb = wait(fa), wait(fb)
            assert ra.result.error_code.val == 1, ra.result.message
            assert rb.result.error_code.val == 1, rb.result.message
            assert node.overlap_samples > 20, f'arms did not overlap: {node.overlap_samples}'
            assert node.minimum_clearance >= .025
            print(f'PASS independent concurrent goals: overlap={node.overlap_samples}, clearance={node.minimum_clearance:.6f}', flush=True)
            # Supplied Cartesian references share collision/constraint checking
            # and braking without being geometrically reoptimized.
            reference = goal(0, -.1)
            reference.reference.joint_names = ['j0']
            reference.reference.points = [
                JointTrajectoryPoint(positions=[node.q[0]], time_from_start=Duration()),
                JointTrajectoryPoint(positions=[-.1], time_from_start=Duration(sec=2))]
            handle = wait(client.send_goal_async(reference))
            assert handle.accepted
            result = wait(handle.get_result_async())
            assert result.result.error_code.val == 1, result.result.message
            reference.reference.points[0].positions = [.5]
            handle = wait(client.send_goal_async(reference))
            assert handle.accepted
            result = wait(handle.get_result_async())
            assert result.result.error_code.val != 1, result
            print('PASS supplied reference and disconnected-start rejection', flush=True)
            reference.reference.points[0].positions = [node.q[0]]
            reference.optimize_reference = True
            handle = wait(client.send_goal_async(reference))
            assert handle.accepted
            result = wait(handle.get_result_async())
            assert result.result.error_code.val == 1, result.result.message
            print('PASS supplied seed with temporal STOMP optimization', flush=True)
            # Equal start/end positions must not complete before traversing an
            # unoptimized supplied reference's intermediate excursion.
            initial=node.q[0]
            loop=goal(0,initial)
            loop.reference.joint_names=['j0']
            loop.reference.points=[JointTrajectoryPoint(positions=[initial]),
                JointTrajectoryPoint(positions=[initial+.03],time_from_start=Duration(sec=1)),
                JointTrajectoryPoint(positions=[initial],time_from_start=Duration(sec=2))]
            handle=wait(client.send_goal_async(loop));assert handle.accepted
            future=handle.get_result_async();peak=initial
            deadline=time.monotonic()+30
            while not future.done() and time.monotonic()<deadline:
                spin(.01);peak=max(peak,node.q[0])
            result=wait(future)
            assert result.result.error_code.val==1,result.result.message
            assert peak>initial+.02, f'loop completed without excursion: peak={peak-initial}'
            committed = re.findall(r'group0 reference committed; start ([0-9.]+), duration ([0-9.]+)',
                                   (directory/'coordinator.log').read_text())
            assert committed, 'loop commitment was not recorded'
            origin, duration = map(float, committed[-1])
            assert node.get_clock().now().nanoseconds*1e-9 >= origin+duration, 'loop completed before its timed end'
            print('PASS timed reference with identical start and goal',flush=True)
            # Scene geometry updates during motion must be checked without
            # unconditionally canceling an otherwise safe independent goal.
            scene_pub = node.create_publisher(PlanningScene, '/monitored_planning_scene', 10)
            moving = wait(client.send_goal_async(goal(0, -.08)))
            assert moving.accepted
            spin(1.5)
            obstacle = CollisionObject(id='scene_transition_probe', operation=CollisionObject.ADD)
            obstacle.header.frame_id = 'world'
            obstacle.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[.05])]
            pose = Pose()
            pose.position.y = 5.
            pose.orientation.w = 1.
            obstacle.primitive_poses = [pose]
            scene_pub.publish(PlanningScene(is_diff=True, world=PlanningSceneWorld(collision_objects=[obstacle])))
            result = wait(moving.get_result_async())
            assert result.result.error_code.val == 1, result.result.message
            obstacle.operation = CollisionObject.REMOVE
            scene_pub.publish(PlanningScene(is_diff=True, world=PlanningSceneWorld(collision_objects=[obstacle])))
            spin(.1)
            print('PASS safe scene update while executing', flush=True)
            # Cancel one group without canceling the peer goal.
            a, b = client.send_goal_async(goal(0, -.15)), client.send_goal_async(goal(1, .15))
            ha, hb = wait(a), wait(b)
            assert ha.accepted and hb.accepted
            cancel_deadline=time.monotonic()+10.
            while abs(node.v[0])<.005 and time.monotonic()<cancel_deadline:
                spin(.01)
            assert abs(node.v[0])>=.005, 'cancel test never entered actual motion'
            wait(ha.cancel_goal_async())
            fa, fb = ha.get_result_async(), hb.get_result_async()
            ra, rb = wait(fa), wait(fb)
            assert ra.status == 5 and rb.result.error_code.val == 1, (ra, rb)
            print('PASS cancel one group while peer completes', flush=True)
            # Impossible endpoint must be reported as failure, no fallback.
            invalid = wait(client.send_goal_async(goal(0, 3.)))
            assert invalid.accepted
            result = wait(invalid.get_result_async())
            assert result.result.error_code.val != 1
            print('PASS unreachable goal reports failure', flush=True)
            call('/stop_motion')
            rejected = wait(client.send_goal_async(goal(0, 0.)))
            assert not rejected.accepted
            spin(.5)
            call('/resume_motion')
            print('PASS stop latch and explicit resume', flush=True)
            timed = wait(client.send_goal_async(goal(0, -.2, timeout=.2)))
            assert timed.accepted
            result = wait(timed.get_result_async())
            assert result.result.error_code.val == -6, result
            print('PASS goal deadline reports TIMED_OUT', flush=True)
            moving = wait(client.send_goal_async(goal(0, -.2)))
            assert moving.accepted
            spin(1.5)
            node.publish_states = False
            result = wait(moving.get_result_async())
            assert result.result.error_code.val == -23, result
            spin(1.2 if node.continuous_desired else .8)
            assert max(abs(x) for x in node.v) < 1e-8
            node.publish_states = True
            spin(.5)
            call('/resume_motion')
            print('PASS stale feedback aborts and retains braking tail', flush=True)
            # Kill the producer during motion: the last short command contains
            # its own checked braking tail and must stop without further traffic.
            moving = wait(client.send_goal_async(goal(0, -.25)))
            assert moving.accepted
            spin(2.)
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
            spin(1.)
            assert max(abs(x) for x in node.v) < 1e-8
            print('PASS producer loss executes final braking tail', flush=True)
    finally:
        print(f'Validation logs: {directory}', flush=True)
        if process and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
