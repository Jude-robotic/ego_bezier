#include "bezier_opt/cooperative_payload_optimizer.h"

#include <algorithm>
#include <array>
#include <cmath>

#include <limits>

#include <ros/ros.h>

namespace ego_planner
{

namespace
{
constexpr double kEps = 1e-8;

Eigen::Vector3d clampToBox(const Eigen::Vector3d &pt,
                           const Eigen::Vector3d &box_min,
                           const Eigen::Vector3d &box_max)
{
  Eigen::Vector3d out = pt;
  for (int i = 0; i < 3; ++i)
    out(i) = std::max(box_min(i), std::min(box_max(i), out(i)));
  return out;
}

Eigen::Vector3d safeDirection(const Eigen::Vector3d &vec, const Eigen::Vector3d &fallback)
{
  const double n = vec.norm();
  if (n < kEps)
    return fallback;
  return vec / n;
}

double sqr(const double v)
{
  return v * v;
}
}  // namespace

void CooperativePayloadOptimizer::setEnvironment(const GridMap::Ptr &env)
{
  grid_map_ = env;
}

void CooperativePayloadOptimizer::setParams(const CooperativePayloadOptParams &params)
{
  params_ = params;
}

CooperativeWindowOutput CooperativePayloadOptimizer::optimize(const CooperativeWindowInput &input)
{
  CooperativeWindowOutput out;
  out.leader_ctrl = input.leader_initial;
  out.follower1_ctrl = input.follower1_initial;
  out.follower2_ctrl = input.follower2_initial;

  if (input.leader_initial.cols() != input.follower1_initial.cols() ||
      input.leader_initial.cols() != input.follower2_initial.cols() ||
      input.leader_initial.cols() != input.leader_nominal.cols() ||
      input.leader_initial.cols() < 7 || input.segment_duration <= 1e-6)
  {
    ROS_WARN("[CoopPayloadOpt] Invalid window input, skip optimization.");
    return out;
  }

  input_ = input;
  variable_layout_.clear();
  samples_.clear();

  const int cols = input.leader_initial.cols();
  for (int agent_idx = 0; agent_idx < 3; ++agent_idx)
  {
    for (int col = 2; col <= cols - 3; ++col)
    {
      for (int dim = 0; dim < 3; ++dim)
      {
        VariableIndex idx;
        idx.agent_idx = agent_idx;
        idx.dim = dim;
        idx.col = col;
        variable_layout_.push_back(idx);
      }
    }
  }

  const int num_seg = (cols - 1) / 3;
  const int sample_count = std::max(1, params_.samples_per_seg);
  for (int seg = 0; seg < num_seg; ++seg)
  {
    const int base = seg * 3;
    for (int si = 0; si <= sample_count; ++si)
    {
      if (seg > 0 && si == 0)
        continue;

      const double u = static_cast<double>(si) / static_cast<double>(sample_count);
      const double one_minus_u = 1.0 - u;

      SampleRef sample;
      sample.base = base;
      sample.weights[0] = one_minus_u * one_minus_u * one_minus_u;
      sample.weights[1] = 3.0 * one_minus_u * one_minus_u * u;
      sample.weights[2] = 3.0 * one_minus_u * u * u;
      sample.weights[3] = u * u * u;
      sample.nominal_leader = evalSample(input.leader_nominal, sample);
      samples_.push_back(sample);
    }
  }

  if (variable_layout_.empty())
  {
    ROS_WARN("[CoopPayloadOpt] Window too short for movable control points, skip optimization.");
    return out;
  }

  EvalCache init_cache;
  init_cache.leader = input.leader_initial;
  init_cache.follower1 = input.follower1_initial;
  init_cache.follower2 = input.follower2_initial;

  std::vector<double> x;
  pack(init_cache, x);

  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
  lbfgs_params.max_iterations = std::max(10, params_.max_iterations);
  lbfgs_params.g_epsilon = std::max(1e-5, params_.g_epsilon);
  lbfgs_params.mem_size = 12;

  double final_cost = 0.0;
  const int ret = lbfgs::lbfgs_optimize(static_cast<int>(x.size()), x.data(), &final_cost,
                                        CooperativePayloadOptimizer::lbfgsCost,
                                        NULL, NULL, this, &lbfgs_params);

  EvalCache final_cache;
  unpack(x.data(), final_cache);
  CostBreakdown breakdown;
  evaluate(x.data(), nullptr, false, &breakdown);

  out.final_cost = final_cost;
  out.result_code = ret;
  out.payload_valid = breakdown.payload_valid;
  out.leader_ctrl = final_cache.leader;
  out.follower1_ctrl = final_cache.follower1;
  out.follower2_ctrl = final_cache.follower2;

  const bool converged = (ret == lbfgs::LBFGS_CONVERGENCE ||
                          ret == lbfgs::LBFGS_STOP ||
                          ret == lbfgs::LBFGS_ALREADY_MINIMIZED ||
                          ret == lbfgs::LBFGSERR_MAXIMUMITERATION);
  out.success = converged && std::isfinite(final_cost);

  ROS_INFO("[CoopPayloadOpt] result=%d success=%d cost=%.3f payload_valid=%d",
           ret, static_cast<int>(out.success), out.final_cost, static_cast<int>(out.payload_valid));

  return out;
}

double CooperativePayloadOptimizer::lbfgsCost(void *instance, const double *x, double *g, const int /*n*/)
{
  return reinterpret_cast<CooperativePayloadOptimizer *>(instance)->evaluate(x, g, true);
}

double CooperativePayloadOptimizer::evaluate(const double *x, double *g, bool with_grad, CostBreakdown *breakdown) const
{
  EvalCache cache;
  unpack(x, cache);

  Eigen::MatrixXd grad_leader = Eigen::MatrixXd::Zero(3, cache.leader.cols());
  Eigen::MatrixXd grad_f1 = Eigen::MatrixXd::Zero(3, cache.follower1.cols());
  Eigen::MatrixXd grad_f2 = Eigen::MatrixXd::Zero(3, cache.follower2.cols());

  CostBreakdown local_breakdown;

  addSmoothnessCost(cache.leader, input_.segment_duration, local_breakdown.smooth, grad_leader);
  addSmoothnessCost(cache.follower1, input_.segment_duration, local_breakdown.smooth, grad_f1);
  addSmoothnessCost(cache.follower2, input_.segment_duration, local_breakdown.smooth, grad_f2);

  addFeasibilityCost(cache.leader, input_.segment_duration, local_breakdown.feas, grad_leader);
  addFeasibilityCost(cache.follower1, input_.segment_duration, local_breakdown.feas, grad_f1);
  addFeasibilityCost(cache.follower2, input_.segment_duration, local_breakdown.feas, grad_f2);

  addUavObstacleCost(cache, local_breakdown.uav_obs, grad_leader, grad_f1, grad_f2);
  addLeaderReferenceCost(cache, local_breakdown.leader_ref, grad_leader);
  addModeShapeCost(cache, local_breakdown.mode_shape, grad_leader, grad_f1, grad_f2);
  addPayloadFeasibilityCost(cache, local_breakdown.payload_feas, grad_leader, grad_f1, grad_f2,
                            &local_breakdown.payload_valid);
  addSeparationCost(cache, local_breakdown.sep, grad_leader, grad_f1, grad_f2);

  local_breakdown.payload_obs = payloadObstacleCostOnly(cache);

  const double total_cost =
      params_.w_smooth * local_breakdown.smooth +
      params_.w_feas * local_breakdown.feas +
      params_.w_uav_obs * local_breakdown.uav_obs +
      params_.w_leader_ref * local_breakdown.leader_ref +
      params_.w_mode_shape * local_breakdown.mode_shape +
      params_.w_payload_feas * local_breakdown.payload_feas +
      params_.w_payload_obs * local_breakdown.payload_obs +
      params_.w_sep * local_breakdown.sep;

  if (with_grad && g != nullptr)
  {
    // Rebuild weighted gradient from scratch to avoid accidental double-counting.
    grad_leader.setZero();
    grad_f1.setZero();
    grad_f2.setZero();

    double scratch_cost = 0.0;
    Eigen::MatrixXd scratch_leader = Eigen::MatrixXd::Zero(3, cache.leader.cols());
    Eigen::MatrixXd scratch_f1 = Eigen::MatrixXd::Zero(3, cache.follower1.cols());
    Eigen::MatrixXd scratch_f2 = Eigen::MatrixXd::Zero(3, cache.follower2.cols());

    addSmoothnessCost(cache.leader, input_.segment_duration, scratch_cost, scratch_leader);
    addSmoothnessCost(cache.follower1, input_.segment_duration, scratch_cost, scratch_f1);
    addSmoothnessCost(cache.follower2, input_.segment_duration, scratch_cost, scratch_f2);
    grad_leader += params_.w_smooth * scratch_leader;
    grad_f1 += params_.w_smooth * scratch_f1;
    grad_f2 += params_.w_smooth * scratch_f2;

    scratch_cost = 0.0;
    scratch_leader.setZero();
    scratch_f1.setZero();
    scratch_f2.setZero();
    addFeasibilityCost(cache.leader, input_.segment_duration, scratch_cost, scratch_leader);
    addFeasibilityCost(cache.follower1, input_.segment_duration, scratch_cost, scratch_f1);
    addFeasibilityCost(cache.follower2, input_.segment_duration, scratch_cost, scratch_f2);
    grad_leader += params_.w_feas * scratch_leader;
    grad_f1 += params_.w_feas * scratch_f1;
    grad_f2 += params_.w_feas * scratch_f2;

    scratch_cost = 0.0;
    scratch_leader.setZero();
    scratch_f1.setZero();
    scratch_f2.setZero();
    addUavObstacleCost(cache, scratch_cost, scratch_leader, scratch_f1, scratch_f2);
    grad_leader += params_.w_uav_obs * scratch_leader;
    grad_f1 += params_.w_uav_obs * scratch_f1;
    grad_f2 += params_.w_uav_obs * scratch_f2;

    scratch_cost = 0.0;
    scratch_leader.setZero();
    addLeaderReferenceCost(cache, scratch_cost, scratch_leader);
    grad_leader += params_.w_leader_ref * scratch_leader;

    scratch_cost = 0.0;
    scratch_leader.setZero();
    scratch_f1.setZero();
    scratch_f2.setZero();
    addModeShapeCost(cache, scratch_cost, scratch_leader, scratch_f1, scratch_f2);
    grad_leader += params_.w_mode_shape * scratch_leader;
    grad_f1 += params_.w_mode_shape * scratch_f1;
    grad_f2 += params_.w_mode_shape * scratch_f2;

    scratch_cost = 0.0;
    scratch_leader.setZero();
    scratch_f1.setZero();
    scratch_f2.setZero();
    bool payload_valid_dummy = false;
    addPayloadFeasibilityCost(cache, scratch_cost, scratch_leader, scratch_f1, scratch_f2, &payload_valid_dummy);
    grad_leader += params_.w_payload_feas * scratch_leader;
    grad_f1 += params_.w_payload_feas * scratch_f1;
    grad_f2 += params_.w_payload_feas * scratch_f2;

    scratch_cost = 0.0;
    scratch_leader.setZero();
    scratch_f1.setZero();
    scratch_f2.setZero();
    addSeparationCost(cache, scratch_cost, scratch_leader, scratch_f1, scratch_f2);
    grad_leader += params_.w_sep * scratch_leader;
    grad_f1 += params_.w_sep * scratch_f1;
    grad_f2 += params_.w_sep * scratch_f2;

    const double eps = std::max(params_.finite_diff_eps, 1e-4);
    std::vector<double> x_center(variable_layout_.size());
    for (size_t i = 0; i < variable_layout_.size(); ++i)
      x_center[i] = x[i];

    for (size_t i = 0; i < variable_layout_.size(); ++i)
    {
      std::vector<double> x_pert = x_center;
      x_pert[i] += eps;
      EvalCache plus_cache;
      unpack(x_pert.data(), plus_cache);
      const double f_plus = payloadObstacleCostOnly(plus_cache);

      x_pert[i] -= 2.0 * eps;
      EvalCache minus_cache;
      unpack(x_pert.data(), minus_cache);
      const double f_minus = payloadObstacleCostOnly(minus_cache);

      const double fd = (f_plus - f_minus) / (2.0 * eps);
      const VariableIndex &var = variable_layout_[i];
      Eigen::MatrixXd *target = nullptr;
      if (var.agent_idx == 0)
        target = &grad_leader;
      else if (var.agent_idx == 1)
        target = &grad_f1;
      else
        target = &grad_f2;
      (*target)(var.dim, var.col) += params_.w_payload_obs * fd;
    }

    for (size_t i = 0; i < variable_layout_.size(); ++i)
    {
      const VariableIndex &var = variable_layout_[i];
      if (var.agent_idx == 0)
        g[i] = grad_leader(var.dim, var.col);
      else if (var.agent_idx == 1)
        g[i] = grad_f1(var.dim, var.col);
      else
        g[i] = grad_f2(var.dim, var.col);
    }
  }

  if (breakdown != nullptr)
    *breakdown = local_breakdown;
  return total_cost;
}

void CooperativePayloadOptimizer::unpack(const double *x, EvalCache &cache) const
{
  cache.leader = input_.leader_initial;
  cache.follower1 = input_.follower1_initial;
  cache.follower2 = input_.follower2_initial;

  for (size_t i = 0; i < variable_layout_.size(); ++i)
  {
    const VariableIndex &var = variable_layout_[i];
    Eigen::MatrixXd *target = nullptr;
    if (var.agent_idx == 0)
      target = &cache.leader;
    else if (var.agent_idx == 1)
      target = &cache.follower1;
    else
      target = &cache.follower2;
    (*target)(var.dim, var.col) = x[i];
  }
}

void CooperativePayloadOptimizer::pack(const EvalCache &cache, std::vector<double> &x) const
{
  x.resize(variable_layout_.size());
  for (size_t i = 0; i < variable_layout_.size(); ++i)
  {
    const VariableIndex &var = variable_layout_[i];
    const Eigen::MatrixXd *source = nullptr;
    if (var.agent_idx == 0)
      source = &cache.leader;
    else if (var.agent_idx == 1)
      source = &cache.follower1;
    else
      source = &cache.follower2;
    x[i] = (*source)(var.dim, var.col);
  }
}

double CooperativePayloadOptimizer::payloadObstacleCostOnly(const EvalCache &cache) const
{
  double cost = 0.0;
  for (const SampleRef &sample : samples_)
  {
    const Eigen::Vector3d leader_pt = evalSample(cache.leader, sample);
    const Eigen::Vector3d f1_pt = evalSample(cache.follower1, sample);
    const Eigen::Vector3d f2_pt = evalSample(cache.follower2, sample);
    const payload_geometry::PayloadSolution payload =
        payload_geometry::solvePayloadCenterLowerBranch(leader_pt, f1_pt, f2_pt, params_.rope_length);
    const double total_radius = params_.payload_radius + params_.payload_extra_margin;
    if (!payload.valid)
    {
      // Invalid geometry (R>L or near-degenerate triangle) should be heavily penalized.
      const double rope_violation = std::max(0.0, payload.circumradius - params_.rope_length);
      const double area_violation = std::max(0.0, params_.triangle_area_min - payload.area);
      const double invalid_err = 1.0 + 10.0 * rope_violation + 5.0 * area_violation;
      cost += params_.payload_invalid_penalty * invalid_err * invalid_err;
      continue;
    }

    // Generic payload-vs-map clearance (applies even when no fixed obstacle template is active).
    cost += pointObstacleCostWithClearance(payload.selected_center, total_radius, nullptr);

    double clearance = 0.0;
    switch (input_.obstacle.type)
    {
      case CooperativeObstacleType::DOOR_FRAME:
        clearance = payload_geometry::doorFrameSignedClearance(
            input_.obstacle.frame, payload.selected_center, input_.obstacle.gap_center_y,
            input_.obstacle.gap_width, total_radius, input_.obstacle.physical_half_extent);
        break;
      case CooperativeObstacleType::Z_SLIT:
        clearance = payload_geometry::slitSignedClearance(
            input_.obstacle.frame, payload.selected_center, input_.obstacle.z_gap_low,
            input_.obstacle.z_gap_high, total_radius, input_.obstacle.physical_half_extent);
        break;
      case CooperativeObstacleType::RING:
        clearance = payload_geometry::ringSignedClearance(
            input_.obstacle.frame, payload.selected_center, input_.obstacle.major_radius,
            input_.obstacle.minor_radius, total_radius);
        break;
      default:
        clearance = 1.0;
        break;
    }

    if (clearance < 0.0)
      cost += clearance * clearance;
  }

  return cost;
}

void CooperativePayloadOptimizer::addSmoothnessCost(const Eigen::MatrixXd &q, double ts,
                                                    double &cost, Eigen::MatrixXd &grad) const
{
  const int num_segments = (q.cols() - 1) / 3;
  for (int k = 0; k < num_segments; ++k)
  {
    const int idx = k * 3;
    const Eigen::Vector3d jerk_vec = q.col(idx + 3) - 3.0 * q.col(idx + 2) + 3.0 * q.col(idx + 1) - q.col(idx);
    const double weight = 10.0 / std::max(ts * ts * ts, 1e-3);
    cost += weight * jerk_vec.squaredNorm();

    const Eigen::Vector3d g = 2.0 * weight * jerk_vec;
    grad.col(idx) += -g;
    grad.col(idx + 1) += 3.0 * g;
    grad.col(idx + 2) += -3.0 * g;
    grad.col(idx + 3) += g;
  }

  const double cont_weight_vel = 3000.0;
  const double cont_weight_acc = 3000.0;
  for (int k = 0; k < num_segments - 1; ++k)
  {
    const int idx = k * 3;
    const Eigen::Vector3d diff_vel = q.col(idx + 4) - 2.0 * q.col(idx + 3) + q.col(idx + 2);
    cost += cont_weight_vel * diff_vel.squaredNorm();
    const Eigen::Vector3d gv = 2.0 * cont_weight_vel * diff_vel;
    grad.col(idx + 4) += gv;
    grad.col(idx + 3) += -2.0 * gv;
    grad.col(idx + 2) += gv;

    const Eigen::Vector3d diff_acc =
        q.col(idx + 1) - 2.0 * q.col(idx + 2) + 2.0 * q.col(idx + 4) - q.col(idx + 5);
    cost += cont_weight_acc * diff_acc.squaredNorm();
    const Eigen::Vector3d ga = 2.0 * cont_weight_acc * diff_acc;
    grad.col(idx + 1) += ga;
    grad.col(idx + 2) += -2.0 * ga;
    grad.col(idx + 4) += 2.0 * ga;
    grad.col(idx + 5) += -ga;
  }
}

void CooperativePayloadOptimizer::addFeasibilityCost(const Eigen::MatrixXd &q, double ts,
                                                     double &cost, Eigen::MatrixXd &grad) const
{
  const int num_segments = (q.cols() - 1) / 3;
  for (int k = 0; k < num_segments; ++k)
  {
    const int idx = k * 3;
    const std::array<Eigen::Vector3d, 3> vel_pts = {
        (q.col(idx + 1) - q.col(idx)) * 3.0 / ts,
        (q.col(idx + 2) - q.col(idx + 1)) * 3.0 / ts,
        (q.col(idx + 3) - q.col(idx + 2)) * 3.0 / ts};

    const std::array<std::pair<int, int>, 3> vel_idx = {{
        {idx, idx + 1}, {idx + 1, idx + 2}, {idx + 2, idx + 3}}};

    for (int i = 0; i < 3; ++i)
    {
      for (int dim = 0; dim < 3; ++dim)
      {
        const double value = vel_pts[i](dim);
        if (std::abs(value) <= params_.max_vel)
          continue;
        const double diff = std::abs(value) - params_.max_vel;
        const double sign = value > 0.0 ? 1.0 : -1.0;
        cost += diff * diff;
        const double g = 2.0 * diff * sign * 3.0 / ts;
        grad(dim, vel_idx[i].second) += g;
        grad(dim, vel_idx[i].first) -= g;
      }
    }

    const std::array<Eigen::Vector3d, 2> acc_pts = {
        (q.col(idx + 2) - 2.0 * q.col(idx + 1) + q.col(idx)) * 6.0 / (ts * ts),
        (q.col(idx + 3) - 2.0 * q.col(idx + 2) + q.col(idx + 1)) * 6.0 / (ts * ts)};

    const std::array<std::array<int, 3>, 2> acc_idx = {{
        {{idx, idx + 1, idx + 2}},
        {{idx + 1, idx + 2, idx + 3}}}};

    for (int i = 0; i < 2; ++i)
    {
      for (int dim = 0; dim < 3; ++dim)
      {
        const double value = acc_pts[i](dim);
        if (std::abs(value) <= params_.max_acc)
          continue;
        const double diff = std::abs(value) - params_.max_acc;
        const double sign = value > 0.0 ? 1.0 : -1.0;
        cost += diff * diff;
        const double factor = 2.0 * diff * sign * 6.0 / (ts * ts);
        grad(dim, acc_idx[i][0]) += factor;
        grad(dim, acc_idx[i][1]) -= 2.0 * factor;
        grad(dim, acc_idx[i][2]) += factor;
      }
    }
  }
}

void CooperativePayloadOptimizer::addLeaderReferenceCost(const EvalCache &cache, double &cost,
                                                         Eigen::MatrixXd &grad_leader) const
{
  const Eigen::MatrixXd diff = cache.leader - input_.leader_nominal;
  cost += diff.squaredNorm();
  grad_leader += 2.0 * diff;
}

void CooperativePayloadOptimizer::addModeShapeCost(const EvalCache &cache,
                                                   double &cost,
                                                   Eigen::MatrixXd &grad_leader,
                                                   Eigen::MatrixXd &grad_f1,
                                                   Eigen::MatrixXd &grad_f2) const
{
  const Eigen::Vector3d offset1 =
      -input_.obstacle.back_offset * input_.obstacle.forward_axis +
      0.5 * input_.obstacle.primary_span * input_.obstacle.primary_axis +
      input_.obstacle.aux_span * input_.obstacle.auxiliary_axis;
  const Eigen::Vector3d offset2 =
      -input_.obstacle.back_offset * input_.obstacle.forward_axis -
      0.5 * input_.obstacle.primary_span * input_.obstacle.primary_axis -
      input_.obstacle.aux_span * input_.obstacle.auxiliary_axis;

  for (const SampleRef &sample : samples_)
  {
    const Eigen::Vector3d leader_pt = evalSample(cache.leader, sample);
    const Eigen::Vector3d follower1_pt = evalSample(cache.follower1, sample);
    const Eigen::Vector3d follower2_pt = evalSample(cache.follower2, sample);

    const Eigen::Vector3d leader_target = sample.nominal_leader + input_.obstacle.leader_bias_world;
    const Eigen::Vector3d leader_diff = leader_pt - leader_target;
    cost += leader_diff.squaredNorm();
    scatterSampleGradient(sample, 2.0 * leader_diff, grad_leader);

    const Eigen::Vector3d f1_diff = follower1_pt - (leader_pt + offset1);
    cost += f1_diff.squaredNorm();
    scatterSampleGradient(sample, 2.0 * f1_diff, grad_f1);
    scatterSampleGradient(sample, -2.0 * f1_diff, grad_leader);

    const Eigen::Vector3d f2_diff = follower2_pt - (leader_pt + offset2);
    cost += f2_diff.squaredNorm();
    scatterSampleGradient(sample, 2.0 * f2_diff, grad_f2);
    scatterSampleGradient(sample, -2.0 * f2_diff, grad_leader);
  }
}

void CooperativePayloadOptimizer::addPayloadFeasibilityCost(const EvalCache &cache,
                                                            double &cost,
                                                            Eigen::MatrixXd &grad_leader,
                                                            Eigen::MatrixXd &grad_f1,
                                                            Eigen::MatrixXd &grad_f2,
                                                            bool *payload_valid) const
{
  bool all_valid = true;
  for (const SampleRef &sample : samples_)
  {
    const Eigen::Vector3d leader_pt = evalSample(cache.leader, sample);
    const Eigen::Vector3d follower1_pt = evalSample(cache.follower1, sample);
    const Eigen::Vector3d follower2_pt = evalSample(cache.follower2, sample);

    const payload_geometry::TriangleAreaGrad area =
        payload_geometry::computeTriangleAreaAndGrad(leader_pt, follower1_pt, follower2_pt);
    const payload_geometry::CircumradiusGrad circum =
        payload_geometry::computeCircumradiusAndGrad(leader_pt, follower1_pt, follower2_pt);

    if (!area.valid || !circum.valid)
    {
      const double rope_violation = std::max(0.0, circum.circumradius - params_.rope_length);
      const double area_violation = std::max(0.0, params_.triangle_area_min - area.area);
      const double invalid_err = 1.0 + 10.0 * rope_violation + 5.0 * area_violation;
      cost += params_.payload_invalid_penalty * invalid_err * invalid_err;
      all_valid = false;
      continue;
    }

    if (area.area < params_.triangle_area_min)
    {
      const double err = params_.triangle_area_min - area.area;
      cost += err * err;
      scatterSampleGradient(sample, -2.0 * err * area.grad_p0, grad_leader);
      scatterSampleGradient(sample, -2.0 * err * area.grad_p1, grad_f1);
      scatterSampleGradient(sample, -2.0 * err * area.grad_p2, grad_f2);
      all_valid = false;
    }

    if (circum.circumradius > params_.rope_length)
    {
      const double err = circum.circumradius - params_.rope_length;
      cost += err * err;
      scatterSampleGradient(sample, 2.0 * err * circum.grad_p0, grad_leader);
      scatterSampleGradient(sample, 2.0 * err * circum.grad_p1, grad_f1);
      scatterSampleGradient(sample, 2.0 * err * circum.grad_p2, grad_f2);
      all_valid = false;
    }
  }

  if (payload_valid != nullptr)
    *payload_valid = all_valid;
}

void CooperativePayloadOptimizer::addSeparationCost(const EvalCache &cache,
                                                    double &cost,
                                                    Eigen::MatrixXd &grad_leader,
                                                    Eigen::MatrixXd &grad_f1,
                                                    Eigen::MatrixXd &grad_f2) const
{
  for (const SampleRef &sample : samples_)
  {
    const Eigen::Vector3d leader_pt = evalSample(cache.leader, sample);
    const Eigen::Vector3d follower1_pt = evalSample(cache.follower1, sample);
    const Eigen::Vector3d follower2_pt = evalSample(cache.follower2, sample);

    const std::array<Eigen::Vector3d, 3> pts = {leader_pt, follower1_pt, follower2_pt};
    for (int i = 0; i < 3; ++i)
    {
      for (int j = i + 1; j < 3; ++j)
      {
        const Eigen::Vector3d diff = pts[i] - pts[j];
        const double dist = diff.norm();
        if (dist >= params_.inter_uav_sep_min || dist < kEps)
          continue;

        const double err = params_.inter_uav_sep_min - dist;
        const Eigen::Vector3d grad_i = -2.0 * err * diff / dist;
        const Eigen::Vector3d grad_j = -grad_i;
        cost += err * err;

        if (i == 0)
          scatterSampleGradient(sample, grad_i, grad_leader);
        else if (i == 1)
          scatterSampleGradient(sample, grad_i, grad_f1);
        else
          scatterSampleGradient(sample, grad_i, grad_f2);

        if (j == 0)
          scatterSampleGradient(sample, grad_j, grad_leader);
        else if (j == 1)
          scatterSampleGradient(sample, grad_j, grad_f1);
        else
          scatterSampleGradient(sample, grad_j, grad_f2);
      }
    }
  }
}

void CooperativePayloadOptimizer::addUavObstacleCost(const EvalCache &cache,
                                                     double &cost,
                                                     Eigen::MatrixXd &grad_leader,
                                                     Eigen::MatrixXd &grad_f1,
                                                     Eigen::MatrixXd &grad_f2) const
{
  for (int col = 0; col < cache.leader.cols(); ++col)
  {
    Eigen::Vector3d pt_grad = Eigen::Vector3d::Zero();
    cost += pointObstacleCost(cache.leader.col(col), &pt_grad);
    grad_leader.col(col) += pt_grad;

    pt_grad.setZero();
    cost += pointObstacleCost(cache.follower1.col(col), &pt_grad);
    grad_f1.col(col) += pt_grad;

    pt_grad.setZero();
    cost += pointObstacleCost(cache.follower2.col(col), &pt_grad);
    grad_f2.col(col) += pt_grad;
  }
}

Eigen::Vector3d CooperativePayloadOptimizer::evalSample(const Eigen::MatrixXd &ctrl,
                                                        const SampleRef &sample) const
{
  Eigen::Vector3d out = Eigen::Vector3d::Zero();
  for (int i = 0; i < 4; ++i)
    out += sample.weights[i] * ctrl.col(sample.base + i);
  return out;
}

void CooperativePayloadOptimizer::scatterSampleGradient(const SampleRef &sample,
                                                        const Eigen::Vector3d &sample_grad,
                                                        Eigen::MatrixXd &ctrl_grad) const
{
  for (int i = 0; i < 4; ++i)
    ctrl_grad.col(sample.base + i) += sample.weights[i] * sample_grad;
}

double CooperativePayloadOptimizer::pointObstacleCost(const Eigen::Vector3d &pt,
                                                      Eigen::Vector3d *grad_out) const
{
  return pointObstacleCostWithClearance(pt, params_.uav_clearance, grad_out);
}

double CooperativePayloadOptimizer::pointObstacleCostWithClearance(const Eigen::Vector3d &pt,
                                                                   double clearance,
                                                                   Eigen::Vector3d *grad_out) const
{
  if (grad_out != nullptr)
    grad_out->setZero();

  if (!grid_map_)
    return 0.0;

  clearance = std::max(0.0, clearance);

  const Eigen::Vector3d map_min = grid_map_->getMapMinBoundary();
  const Eigen::Vector3d map_max = grid_map_->getMapMaxBoundary();

  if (!grid_map_->isInMap(pt))
  {
    const Eigen::Vector3d clamped = clampToBox(pt, map_min, map_max);
    const Eigen::Vector3d diff = pt - clamped;
    const double dist = diff.norm();
    if (dist < kEps)
      return 0.0;
    const double err = clearance + dist;
    if (grad_out != nullptr)
      *grad_out = 2.0 * err * safeDirection(diff, Eigen::Vector3d::UnitX());
    return err * err;
  }

  Eigen::Vector3i center_idx;
  grid_map_->posToIndex(pt, center_idx);
  const double res = grid_map_->getResolution();
  const int search_steps = std::max(1, static_cast<int>(std::ceil(params_.obstacle_search_radius / std::max(res, 1e-3))));

  double best_dist = std::numeric_limits<double>::infinity();
  Eigen::Vector3d best_occ = pt;

  for (int dx = -search_steps; dx <= search_steps; ++dx)
  {
    for (int dy = -search_steps; dy <= search_steps; ++dy)
    {
      for (int dz = -search_steps; dz <= search_steps; ++dz)
      {
        Eigen::Vector3i idx = center_idx + Eigen::Vector3i(dx, dy, dz);
        if (!grid_map_->isInMap(idx))
          continue;

        Eigen::Vector3d occ_pos;
        grid_map_->indexToPos(idx, occ_pos);
        if (grid_map_->getInflateOccupancy(occ_pos) <= 0)
          continue;

        const double dist = (pt - occ_pos).norm();
        if (dist < best_dist)
        {
          best_dist = dist;
          best_occ = occ_pos;
        }
      }
    }
  }

  if (!std::isfinite(best_dist) || best_dist >= clearance)
    return 0.0;

  const double err = clearance - best_dist;
  if (grad_out != nullptr)
  {
    Eigen::Vector3d diff = pt - best_occ;
    if (diff.norm() < kEps)
    {
      diff = safeDirection(pt - 0.5 * (map_min + map_max), Eigen::Vector3d::UnitX());
    }
    else
    {
      diff.normalize();
    }
    *grad_out = -2.0 * err * diff;
  }
  return err * err;
}

}  // namespace ego_planner
