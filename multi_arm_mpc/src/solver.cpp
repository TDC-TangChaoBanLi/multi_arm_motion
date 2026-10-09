#include <multi_arm_mpc/solver.hpp>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <limits>
namespace multi_arm_motion
{
void MpcConfig::validate() const
{
  if (horizon < 2 || iterations < 1 || stop_step<1 || stop_step>horizon || !std::isfinite(dt) || dt <= 0 ||
      !std::isfinite(budget_s) || budget_s <= 0 || !std::isfinite(clearance) || clearance <= 0 ||
      !std::isfinite(influence_distance) || influence_distance < clearance ||
      !std::isfinite(finite_difference) || finite_difference <= 0 ||
      !std::isfinite(validation_joint_step) || validation_joint_step <= 0 ||
      !std::isfinite(validation_time_step) || validation_time_step <= 0)
    throw std::invalid_argument("invalid MPC configuration");
  for (double w : {tracking_weight, velocity_weight, acceleration_weight, obstacle_weight, task_weight, terminal_weight})
    if (!std::isfinite(w) || w < 0) throw std::invalid_argument("invalid MPC weight");
}
MpcSolver::MpcSolver(MpcConfig config): config_(config) { config_.validate(); }
MpcResult MpcSolver::solve(const Eigen::VectorXd& q, const Eigen::VectorXd& velocity,
    const Eigen::MatrixXd& reference, const Eigen::VectorXd& lower, const Eigen::VectorXd& upper,
    const Eigen::VectorXd& vmax, const Eigen::VectorXd& amax,
    const DistanceOracle& distance, const StatePredicate& allowed,const DistanceGradientOracle& distance_gradient,
    const DistanceOracle& validation_distance,const TaskCostOracle& task_cost,const StopPredicate& can_stop,
    const std::vector<std::vector<int>>& independent_groups)
{
  MpcResult out; const auto& c = config_; const int n = q.size(), h = c.horizon;
  if (!n || reference.rows() != n || reference.cols() != h || velocity.size() != n ||
      lower.size() != n || upper.size() != n || vmax.size() != n || amax.size() != n ||
      !q.allFinite() || !velocity.allFinite() || !reference.allFinite() ||
      !lower.allFinite() || !upper.allFinite() || !vmax.allFinite() || !amax.allFinite() ||
      (lower.array() >= upper.array()).any() || (vmax.array() <= 0).any() || (amax.array() <= 0).any() ||
      (q.array() < lower.array()).any() || (q.array() > upper.array()).any() ||
      (velocity.array().abs() > vmax.array() + 1e-6).any() || !distance)
  { out.reason = "invalid MPC input or measured state outside limits"; return out; }
  for (const auto& group : independent_groups)
    for (int joint : group)
      if (joint < 0 || joint >= n) {out.reason="invalid independent braking group"; return out;}
  const auto started = std::chrono::steady_clock::now();
  // A safe stationary endpoint must remain reachable despite a nonzero soft
  // repulsion cost. Do not boost tracking for an endpoint made unsafe by a
  // moving obstacle; hard sweep and stopping checks still apply in both cases.
  Eigen::VectorXd tracking=Eigen::VectorXd::Constant(n,c.tracking_weight);
  const Eigen::VectorXd endpoint=reference.col(h-1);
  if(distance(endpoint,h*c.dt)>=c.clearance&&(!allowed||allowed(endpoint,h*c.dt)))
    for(int j=0;j<n;++j)
      if((reference.row(j).array()-endpoint[j]).abs().maxCoeff()<1e-8)
        tracking[j]=std::max(c.tracking_weight,c.terminal_weight);
  auto expired = [&] { return std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count() >= c.budget_s; };
  auto project = [&](Eigen::MatrixXd u) {
    Eigen::VectorXd previous = velocity, position = q;
    for (int k=0;k<h;++k)
    {
      for (int j=0;j<n;++j)
      {
        const double lo = std::max(-vmax[j], previous[j] - amax[j]*c.dt);
        const double hi = std::min(vmax[j], previous[j] + amax[j]*c.dt);
        u(j,k) = std::clamp(u(j,k),lo,hi);
      }
      position += c.dt*u.col(k); previous=u.col(k);
    }
    return u;
  };
  auto integrate = [&](const Eigen::MatrixXd& u) {
    Eigen::MatrixXd x(n,h); Eigen::VectorXd position=q,previous=velocity;
    for(int k=0;k<h;++k) { position += 0.5*c.dt*(previous+u.col(k)); x.col(k)=position; previous=u.col(k); } return x;
  };
  std::string candidate_failure;
  auto evaluate = [&](const Eigen::MatrixXd& u, Eigen::MatrixXd* gradient) {
    const auto x=integrate(u); double total=0; Eigen::MatrixXd gx=Eigen::MatrixXd::Zero(n,h);
    bool task_feasible=true;
    Eigen::MatrixXd gu=2*c.velocity_weight*u;
    for(int k=0;k<h;++k)
    {
      const Eigen::VectorXd error=x.col(k)-reference.col(k);
      total += (tracking.array()*error.array().square()).sum()+c.velocity_weight*u.col(k).squaredNorm();
      gx.col(k)=2*tracking.cwiseProduct(error);
      const Eigen::VectorXd prev=k ? Eigen::VectorXd(u.col(k-1)) : velocity;
      const Eigen::VectorXd delta=u.col(k)-prev;
      total += c.acceleration_weight*delta.squaredNorm();
      gu.col(k)+=2*c.acceleration_weight*delta;
      if(k) gu.col(k-1)-=2*c.acceleration_weight*delta;
      for(int j=0;j<n;++j)
      {
        const double violation=x(j,k)<lower[j] ? x(j,k)-lower[j] : x(j,k)>upper[j] ? x(j,k)-upper[j] : 0;
        total += 10000*violation*violation; gx(j,k)+=20000*violation;
      }
      const double t=(k+1)*c.dt;
      if(task_cost)
      {
        const auto task=task_cost(x.col(k),t,gradient!=nullptr);
        if(!std::isfinite(task.first)||task.first<0)return std::numeric_limits<double>::infinity();
        total+=c.task_weight*task.first;
        if(gradient)
        {
          if(task.second.size()!=n||!task.second.allFinite())return std::numeric_limits<double>::infinity();
          gx.col(k)+=c.task_weight*task.second;
        }
      }
      bool segment_allowed=true;
      if(allowed)
      {
        const Eigen::VectorXd begin=k?Eigen::VectorXd(x.col(k-1)):q;
        const Eigen::VectorXd speed=k?Eigen::VectorXd(u.col(k-1)):velocity;
        const int checks=std::max(1,std::max(int(std::ceil((x.col(k)-begin).cwiseAbs().maxCoeff()/c.validation_joint_step)),
                                            int(std::ceil(c.dt/c.validation_time_step))));
        for(int i=0;i<=checks;++i)
        {
          const double a=double(i)/checks;
          const Eigen::VectorXd sample=begin+c.dt*(a*speed+.5*a*a*(u.col(k)-speed));
          if(!allowed(sample,(k+a)*c.dt)){segment_allowed=false;break;}
        }
      }
      if(!segment_allowed)
      {
        candidate_failure="seed task constraint at t="+std::to_string(t);
        task_feasible=false;
        if(!gradient)return std::numeric_limits<double>::infinity();
      }
      const auto query=gradient&&distance_gradient?distance_gradient(x.col(k),t):
          std::make_pair(distance(x.col(k),t),Eigen::VectorXd());
      const double d=query.first;
      if (std::isnan(d)) return std::numeric_limits<double>::infinity();
      if(d<c.clearance)
      {
        candidate_failure="seed clearance at t="+std::to_string(t)+": distance="+std::to_string(d);
        task_feasible=false;
        if(!gradient)return std::numeric_limits<double>::infinity();
      }
      const double deficit=std::max(0.0,c.influence_distance-d);
      total+=c.obstacle_weight*deficit*deficit;
      if(gradient && deficit>0)
      {
        if(distance_gradient)
        {
          if(query.second.size()!=n||!query.second.allFinite())return std::numeric_limits<double>::infinity();
          gx.col(k)-=2*c.obstacle_weight*deficit*query.second;
        }
        else
        for(int j=0;j<n;++j)
        {
          if(expired()) return std::numeric_limits<double>::infinity();
          Eigen::VectorXd plus=x.col(k),minus=x.col(k); plus[j]+=c.finite_difference;minus[j]-=c.finite_difference;
          const double dd=(distance(plus,t)-distance(minus,t))/(2*c.finite_difference);
          if(std::isfinite(dd)) gx(j,k)-=2*c.obstacle_weight*deficit*dd;
        }
      }
    }
    if(gradient)
    {
      Eigen::VectorXd cumulative=Eigen::VectorXd::Zero(n);
      for(int k=h-1;k>=0;--k) {gu.col(k)+=c.dt*(cumulative+0.5*gx.col(k));cumulative+=gx.col(k);}
      *gradient=gu;
    }
    if(can_stop&&!can_stop(x.col(c.stop_step-1),u.col(c.stop_step-1),c.stop_step*c.dt))
    {candidate_failure="seed braking tail";task_feasible=false;}
    return task_feasible?total:std::numeric_limits<double>::infinity();
  };
  // Recenter the warm start on fresh feedback/reference every cycle. A shifted
  // old velocity alone can keep moving forever when the gradient budget expires.
  Eigen::MatrixXd u(n,h);Eigen::VectorXd seed_position=q,seed_velocity=velocity;
  for(int k=0;k<h;++k)
  {
    const Eigen::VectorXd desired=(reference.col(k)-seed_position)/c.dt;
    for(int j=0;j<n;++j)
      u(j,k)=std::clamp(desired[j],std::max(-vmax[j],seed_velocity[j]-amax[j]*c.dt),
                                 std::min(vmax[j],seed_velocity[j]+amax[j]*c.dt));
    seed_position+=.5*c.dt*(seed_velocity+u.col(k));seed_velocity=u.col(k);
  }
  if(warm_.rows()==n && warm_.cols()==h)
  {
    u.leftCols(h-1)=.75*u.leftCols(h-1)+.25*warm_.rightCols(h-1);
    u.col(h-1)=.75*u.col(h-1)+.25*warm_.col(h-1);
  }
  u=project(u);Eigen::MatrixXd first_gradient;
  double value=evaluate(u,&first_gradient);
  const std::string nominal_seed_failure=candidate_failure;
  // A braking seed gives the optimizer a feasible alternative when the new
  // reference cannot retain a checked stop. It is checked, never assumed safe.
  Eigen::MatrixXd braking(n,h);
  const double stopping_time=std::max(c.dt,(velocity.cwiseAbs().array()/amax.array()).maxCoeff());
  for(int k=0;k<h;++k)braking.col(k)=velocity*std::max(0.,1.-(k+1)*c.dt/stopping_time);
  braking=project(braking);
  const Eigen::MatrixXd nominal_seed=u;
  std::vector<int> paused;
  bool braking_seed=false;
  if(can_stop&&!std::isfinite(value))
  {
    // Try braking just one independent group while peers retain their nominal
    // progress. Every mixed candidate is checked against the complete scene.
    for (const auto& group : independent_groups)
    {
      if (expired()) break;
      auto mixed=nominal_seed;
      for (int joint : group) mixed.row(joint)=braking.row(joint);
      if (std::isfinite(evaluate(mixed,nullptr)))
      {u=mixed;paused=group;value=evaluate(u,&first_gradient);braking_seed=true;break;}
    }
    if (!braking_seed && std::isfinite(evaluate(braking,nullptr)))
    {
      u=braking;value=evaluate(u,&first_gradient);braking_seed=true;
      for(int joint=0;joint<n;++joint)paused.push_back(joint);
    }
  }
  for(int iteration=0;iteration<c.iterations && !expired();++iteration)
  {
    Eigen::MatrixXd gradient;
    if(iteration==0)gradient=first_gradient;else evaluate(u,&gradient);
    if(expired() || gradient.size()==0 || !gradient.allFinite()) break;
    Eigen::VectorXd row_step(n);
    for(int j=0;j<n;++j)row_step[j]=std::min(.1,1./(2*c.dt*c.dt*h*h*tracking[j]+4*c.acceleration_weight+1.));
    double step=1.;
    bool improved=false;
    for(int trial=0;trial<8 && !expired();++trial)
    {
      Eigen::MatrixXd candidate=project(u-step*row_step.asDiagonal()*gradient);
      const double score=evaluate(candidate,nullptr);
      if(score<value) {u=candidate;value=score;improved=true;break;} step*=0.5;
    }
    if(!improved) break;
  }
  out.deadline_reached=expired();
  const auto validation_started=std::chrono::steady_clock::now();
  out.optimization_seconds=std::chrono::duration<double>(validation_started-started).count();
  auto validate=[&](const Eigen::MatrixXd& candidate){
  out.positions=integrate(candidate);out.velocities=candidate;
  out.min_distance=std::numeric_limits<double>::infinity();
  Eigen::VectorXd previous=q;
  // Full horizon post-validation, independent of soft optimization penalties.
  // Deliberately completes even after optimization budget: this is not a hard realtime solver.
  for(int k=0;k<h;++k)
  {
    const Eigen::VectorXd end=out.positions.col(k);
    const int count=std::max(1,std::max(int(std::ceil((end-previous).cwiseAbs().maxCoeff()/c.validation_joint_step)),
                                      int(std::ceil(c.dt/c.validation_time_step))));
    for(int s=0;s<=count;++s)
    {
      const double a=double(s)/count,t=(k+a)*c.dt;
      const Eigen::VectorXd vstart=k ? Eigen::VectorXd(candidate.col(k-1)) : velocity;
      const Eigen::VectorXd x=previous + c.dt*(a*vstart+0.5*a*a*(candidate.col(k)-vstart));
      const double d=validation_distance?validation_distance(x,t):distance(x,t);out.min_distance=std::min(out.min_distance,d);
      if(std::isnan(d)||d<c.clearance)
      {out.reason="predicted clearance violation at t="+std::to_string(t)+" s: distance="+std::to_string(d)+
          " m, required="+std::to_string(c.clearance)+" m";return false;}
      if((x.array()<lower.array()-1e-8).any()||(x.array()>upper.array()+1e-8).any())
      {out.reason="predicted joint bound violation at t="+std::to_string(t)+" s";return false;}
      if(allowed&&!allowed(x,t))
      {out.reason="predicted task constraint violation at t="+std::to_string(t)+" s";return false;}
    }
    previous=end;
  }
  if(can_stop&&!can_stop(out.positions.col(c.stop_step-1),candidate.col(c.stop_step-1),c.stop_step*c.dt))
  {out.reason="candidate cannot retain a validated braking tail";return false;}
  return true;
  };
  if(!validate(u))
  {
    const std::string nominal_failure=out.reason;
    bool recovered=false;
    if(can_stop)for(const auto& group:independent_groups)
    {
      auto mixed=nominal_seed;
      for(int joint:group)mixed.row(joint)=braking.row(joint);
      if(validate(mixed)){u=mixed;paused=group;recovered=true;break;}
    }
    if(!recovered)
    {
      if(!can_stop||!validate(braking)){warm_.resize(0,0);return out;}
      u=braking;paused.clear();for(int joint=0;joint<n;++joint)paused.push_back(joint);
    }
    out.braking_only=true;out.reason="validated braking fallback: "+nominal_failure;
  }
  else if(braking_seed)
  {out.braking_only=true;out.reason="validated braking fallback: "+nominal_seed_failure;}
  out.validation_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-validation_started).count();
  out.paused_joints=std::move(paused);
  out.feasible=true;if(!out.braking_only)out.reason="validated horizon";warm_=u;return out;
}
}
