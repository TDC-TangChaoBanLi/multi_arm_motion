#!/usr/bin/env python3
"""Isolated, software-only Hybrid Planning smoke test using three prismatic groups.

No MuJoCo or physical robot. Run in an unused ROS_DOMAIN_ID, after sourcing install.
The mock driver only checks integration; it is not a model of hardware tracking.
"""
import math
from contextlib import nullcontext
import os
from pathlib import Path
import subprocess
import tempfile
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rosidl_runtime_py.convert import message_to_yaml
from moveit_msgs.action import HybridPlanner
from moveit_msgs.msg import MotionSequenceItem, Constraints, JointConstraint
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import yaml


def robot_config():
    links = ['<link name="base"/>']
    for i in range(3):
        links.append(f'''<link name="link{i}"><collision><geometry>
          <sphere radius="0.1"/></geometry></collision></link>
          <joint name="j{i}" type="prismatic"><parent link="base"/><child link="link{i}"/>
          <origin xyz="{i * 0.5} 0 0"/><axis xyz="1 0 0"/>
          <limit lower="-2" upper="2" effort="10" velocity="1"/></joint>''')
    urdf = '<robot name="sliders">' + ''.join(links) + '</robot>'
    groups = ''.join(f'<group name="group{i}"><joint name="j{i}"/></group>' for i in range(3))
    srdf = '<robot name="sliders">' + groups + '<group name="all">' + ''.join(
        f'<joint name="j{i}"/>' for i in range(3)) + '</group></robot>'
    return {'/**': {'ros__parameters': {
        'robot_description': urdf, 'robot_description_semantic': srdf,
        'group_name': 'all', 'use_sim_time': False,
        'mpc': {'enable_execution': True, 'optimization_budget': 0.035},
        'dynamic_stomp': {'prediction_start_delay': 0.0, 'delta_t': 0.1, 'random_seed': 7,
                          'num_iterations': 100, 'num_iterations_after_valid': 0},
    }}}


class MockDriver(Node):
    def __init__(self):
        super().__init__('slider_mock_driver')
        self.q = [0.0] * 3
        self.v = [0.0] * 3
        self.command = None
        self.command_start = 0.0
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.sub = self.create_subscription(JointTrajectory, '/mpc_joint_commands', self.receive, 1)
        self.timer = self.create_timer(0.01, self.tick)
        self.commands = 0
        self.minimum_clearance = math.inf

    def receive(self, msg):
        if msg.joint_names != ['j0', 'j1', 'j2']:
            raise RuntimeError('unexpected mock command joint order')
        self.command, self.command_start = msg, time.monotonic()
        self.commands += 1

    def tick(self):
        if self.command:
            points = self.command.points
            duration = points[-1].time_from_start.sec + points[-1].time_from_start.nanosec * 1e-9
            a = min(1.0, (time.monotonic() - self.command_start) / max(duration, 1e-9))
            start = points[0].positions if len(points) > 1 else self.q
            # Mock transport tracking only; production controller interpolation is a separate gate.
            self.q = [(1-a)*start[j] + a*points[-1].positions[j] for j in range(3)]
            self.v = [0.0 if a >= 1 else (points[-1].positions[j]-start[j])/duration for j in range(3)]
        for i in range(3):
            for j in range(i+1, 3):
                self.minimum_clearance = min(self.minimum_clearance, abs(i*0.5+self.q[i]-j*0.5-self.q[j])-0.2)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name, msg.position, msg.velocity = ['j0', 'j1', 'j2'], self.q, self.v
        self.pub.publish(msg)


def cli_request(node, arguments, timeout):
    request = subprocess.Popen(['ros2', *arguments], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    deadline = time.monotonic() + timeout
    while request.poll() is None and time.monotonic() < deadline:
        node.executor.spin_once(timeout_sec=0.01)
    if request.poll() is None:
        request.terminate()
        request.communicate(timeout=5)
        raise TimeoutError('Hybrid Planning CLI request timed out')
    output = request.communicate()[0]
    if request.returncode:
        raise RuntimeError(output)
    return output


def main():
    if os.environ.get('ROS_DOMAIN_ID') in (None, '', '0', '11', '91'):
        raise RuntimeError('Set an unused ROS_DOMAIN_ID, e.g. 94, for this isolated mock test')
    rclpy.init()
    node = MockDriver()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    process = None
    try:
        with nullcontext(tempfile.mkdtemp(prefix='multi_arm_smoke_')) as directory:
            config = Path(directory) / 'robot.yaml'
            config.write_text(yaml.safe_dump(robot_config()))
            with (Path(directory) / 'hybrid.log').open('w') as log:
                process = subprocess.Popen(['ros2', 'launch', 'multi_arm_hybrid_planning',
                    'hybrid.launch.py', f'robot_config:={config}', 'legacy_runtime:=true'], stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True)
                client = ActionClient(node, HybridPlanner, '/run_hybrid_planning')
                until = time.monotonic() + 20
                while not client.server_is_ready() and time.monotonic() < until:
                    executor.spin_once(timeout_sec=0.05)
                    if process.poll() is not None:
                        raise RuntimeError((Path(directory) / 'hybrid.log').read_text()[-8000:])
                if not client.server_is_ready():
                    raise TimeoutError('Hybrid Planning server not available')
                for _ in range(100):
                    executor.spin_once(timeout_sec=0.01)
                # Verify request/response transport, not just discovery metadata.
                cli_request(node, ['param', 'get', '/hybrid_planning_manager', 'planner_logic_plugin_name'], 15)
                goal = HybridPlanner.Goal()
                goal.planning_group = 'all'
                item = MotionSequenceItem()
                item.req.group_name = 'all'
                item.req.allowed_planning_time = 5.0
                item.req.num_planning_attempts = 1
                item.req.max_velocity_scaling_factor = 1.0
                item.req.max_acceleration_scaling_factor = 1.0
                constraint = Constraints()
                expected = [0.05, -0.05, 0.1]
                for name, value in zip(['j0', 'j1', 'j2'], expected):
                    constraint.joint_constraints.append(JointConstraint(joint_name=name, position=value,
                        tolerance_above=0.001, tolerance_below=0.001, weight=1.0))
                item.req.goal_constraints = [constraint]
                goal.motion_sequence.items = [item]
                output = cli_request(node, ['action', 'send_goal', '/run_hybrid_planning',
                    'moveit_msgs/action/HybridPlanner', message_to_yaml(goal)], 45)
                if 'SUCCEEDED' not in output:
                    raise RuntimeError(f'Hybrid goal failed: {output}')
                error = max(abs(a-b) for a, b in zip(node.q, expected))
                if error > 0.011 or node.minimum_clearance < 0.025 or node.commands == 0:
                    raise RuntimeError(f'invalid execution: error={error}, clearance={node.minimum_clearance}')
                print(f'PASS: 3 groups, {node.commands} commands, max goal error={error:.6f}, '
                      f'min clearance={node.minimum_clearance:.6f}')
    finally:
        if 'directory' in locals():
            print(f'Smoke logs: {directory}/hybrid.log')
        if process and process.poll() is None:
            import signal
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
