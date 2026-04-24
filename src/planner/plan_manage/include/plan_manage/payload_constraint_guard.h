#ifndef _PAYLOAD_CONSTRAINT_GUARD_H_
#define _PAYLOAD_CONSTRAINT_GUARD_H_

#include <bezier_opt/payload_geometry.h>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace ego_planner
{
namespace plan_manage
{

struct PayloadConstraintGuardParams
{
  double rope_length{1.0};
  double rope_margin{0.0};
  double triangle_area_min{0.05};
  double area_margin{0.0};
  double inter_uav_sep_min{0.8};
  double pair_margin{0.0};
};

struct PayloadConstraintGuardState
{
  bool valid{false};
  double area{0.0};
  double circumradius{std::numeric_limits<double>::infinity()};
  double min_pair_distance{0.0};
};

struct PayloadConstraintGuardResult
{
  double blend{1.0};
  bool base_valid{false};
  bool desired_valid{false};
  PayloadConstraintGuardState state;
};

struct PayloadConstraintTrajectoryState
{
  bool valid{false};
  bool evaluated{false};
  int sample_count{0};
  int invalid_samples{0};
  double min_area{std::numeric_limits<double>::infinity()};
  double max_circumradius{0.0};
  double min_pair_distance{std::numeric_limits<double>::infinity()};
};

struct PayloadConstraintTrajectoryGuardResult
{
  double blend{1.0};
  bool base_valid{false};
  bool desired_valid{false};
  PayloadConstraintTrajectoryState trajectory;
};

inline Eigen::Vector3d interpolatePoint(const Eigen::Vector3d &from,
                                        const Eigen::Vector3d &to,
                                        const double blend)
{
  return from + blend * (to - from);
}

inline double bezierDuration(const std::vector<double> &segment_durations)
{
  double total = 0.0;
  for (const double dt : segment_durations)
    total += dt;
  return total;
}

inline bool evalBezierCtrl(const Eigen::MatrixXd &ctrl,
                           const std::vector<double> &segment_durations,
                           const int order,
                           const double t_query,
                           Eigen::Vector3d &result)
{
  const int n_pts = ctrl.cols();
  const int n_seg = static_cast<int>(segment_durations.size());
  if (n_pts <= 0 || n_seg <= 0 || order < 1)
    return false;

  double t_local = std::max(0.0, t_query);
  int seg_idx = n_seg - 1;
  for (int i = 0; i < n_seg; ++i)
  {
    if (t_local <= segment_durations[static_cast<size_t>(i)] || i == n_seg - 1)
    {
      seg_idx = i;
      break;
    }
    t_local -= segment_durations[static_cast<size_t>(i)];
  }

  const double seg_dt = segment_durations[static_cast<size_t>(seg_idx)];
  const double u = (seg_dt < 1e-9) ? 1.0 : std::max(0.0, std::min(1.0, t_local / seg_dt));
  const int base = seg_idx * order;

  std::vector<Eigen::Vector3d> pts(static_cast<size_t>(order + 1), Eigen::Vector3d::Zero());
  for (int k = 0; k <= order; ++k)
  {
    const int idx = std::min(base + k, n_pts - 1);
    pts[static_cast<size_t>(k)] = ctrl.col(idx);
  }

  int sz = static_cast<int>(pts.size());
  while (sz > 1)
  {
    for (int i = 0; i < sz - 1; ++i)
      pts[static_cast<size_t>(i)] =
          (1.0 - u) * pts[static_cast<size_t>(i)] + u * pts[static_cast<size_t>(i + 1)];
    --sz;
  }

  result = pts.front();
  return true;
}

inline PayloadConstraintGuardState evaluatePayloadConstraintGuard(
    const Eigen::Vector3d &leader,
    const Eigen::Vector3d &follower1,
    const Eigen::Vector3d &follower2,
    const PayloadConstraintGuardParams &params)
{
  constexpr double kTol = 1e-6;

  PayloadConstraintGuardState out;
  const auto area_grad =
      payload_geometry::computeTriangleAreaAndGrad(leader, follower1, follower2);
  const auto circum_grad =
      payload_geometry::computeCircumradiusAndGrad(leader, follower1, follower2);

  out.area = area_grad.area;
  if (circum_grad.valid)
    out.circumradius = circum_grad.circumradius;

  out.min_pair_distance =
      std::min({(leader - follower1).norm(),
                (leader - follower2).norm(),
                (follower1 - follower2).norm()});

  const double effective_rope = std::max(0.0, params.rope_length - params.rope_margin);
  const double required_area = std::max(0.0, params.triangle_area_min + params.area_margin);
  const double required_pair_dist =
      std::max(0.0, params.inter_uav_sep_min + params.pair_margin);

  if (!area_grad.valid || !circum_grad.valid)
    return out;

  if (effective_rope <= kTol)
    return out;

  const bool area_ok = out.area + kTol >= required_area;
  const bool rope_ok = out.circumradius <= effective_rope + kTol;
  const bool pair_ok = out.min_pair_distance + kTol >= required_pair_dist;

  out.valid = area_ok && rope_ok && pair_ok;
  return out;
}

inline PayloadConstraintGuardResult projectPayloadConstraintBlend(
    const Eigen::Vector3d &leader,
    const Eigen::Vector3d &base_follower1,
    const Eigen::Vector3d &base_follower2,
    const Eigen::Vector3d &desired_follower1,
    const Eigen::Vector3d &desired_follower2,
    const PayloadConstraintGuardParams &params)
{
  constexpr int kCoarseSamples = 24;
  constexpr int kBinaryRefineIters = 12;

  PayloadConstraintGuardResult out;
  out.state = evaluatePayloadConstraintGuard(
      leader, base_follower1, base_follower2, params);
  out.base_valid = out.state.valid;

  const PayloadConstraintGuardState desired_state =
      evaluatePayloadConstraintGuard(
          leader, desired_follower1, desired_follower2, params);
  out.desired_valid = desired_state.valid;
  if (desired_state.valid)
  {
    out.blend = 1.0;
    out.state = desired_state;
    return out;
  }

  double best_blend = 0.0;
  PayloadConstraintGuardState best_state = out.state;
  bool found_feasible = out.base_valid;
  double search_hi = 0.0;

  for (int sample = kCoarseSamples; sample >= 0; --sample)
  {
    const double blend = static_cast<double>(sample) / static_cast<double>(kCoarseSamples);
    const PayloadConstraintGuardState state =
        evaluatePayloadConstraintGuard(
            leader,
            interpolatePoint(base_follower1, desired_follower1, blend),
            interpolatePoint(base_follower2, desired_follower2, blend),
            params);
    if (!state.valid)
      continue;

    best_blend = blend;
    best_state = state;
    found_feasible = true;
    search_hi = std::min(1.0, blend + 1.0 / static_cast<double>(kCoarseSamples));
    break;
  }

  if (!found_feasible)
  {
    out.blend = 0.0;
    out.state = best_state;
    return out;
  }

  double lo = best_blend;
  double hi = search_hi;
  for (int iter = 0; iter < kBinaryRefineIters; ++iter)
  {
    const double mid = 0.5 * (lo + hi);
    const PayloadConstraintGuardState state =
        evaluatePayloadConstraintGuard(
            leader,
            interpolatePoint(base_follower1, desired_follower1, mid),
            interpolatePoint(base_follower2, desired_follower2, mid),
            params);
    if (state.valid)
    {
      lo = mid;
      best_state = state;
    }
    else
    {
      hi = mid;
    }
  }

  out.blend = lo;
  out.state = best_state;
  return out;
}

inline PayloadConstraintTrajectoryState evaluatePayloadConstraintTrajectoryGuard(
    const Eigen::MatrixXd &leader_ctrl,
    const Eigen::MatrixXd &base_ctrl1,
    const Eigen::MatrixXd &base_ctrl2,
    const Eigen::MatrixXd &desired_ctrl1,
    const Eigen::MatrixXd &desired_ctrl2,
    const std::vector<double> &segment_durations,
    const int order,
    const double blend,
    const PayloadConstraintGuardParams &params,
    const int samples_per_segment)
{
  PayloadConstraintTrajectoryState out;
  const int n_seg = static_cast<int>(segment_durations.size());
  if (n_seg <= 0 || order < 1 ||
      leader_ctrl.cols() <= 0 || base_ctrl1.cols() <= 0 || base_ctrl2.cols() <= 0 ||
      desired_ctrl1.cols() <= 0 || desired_ctrl2.cols() <= 0)
    return out;

  const int samples_per_seg = std::max(1, samples_per_segment);
  const double clamped_blend = std::max(0.0, std::min(1.0, blend));
  out.evaluated = true;

  double t_prefix = 0.0;
  for (int seg_idx = 0; seg_idx < n_seg; ++seg_idx)
  {
    const double seg_dt = segment_durations[static_cast<size_t>(seg_idx)];
    for (int k = (seg_idx == 0 ? 0 : 1); k <= samples_per_seg; ++k)
    {
      const double u = static_cast<double>(k) / static_cast<double>(samples_per_seg);
      const double t = t_prefix + u * seg_dt;

      Eigen::Vector3d leader_pt;
      Eigen::Vector3d base_pt1;
      Eigen::Vector3d base_pt2;
      Eigen::Vector3d desired_pt1;
      Eigen::Vector3d desired_pt2;
      if (!evalBezierCtrl(leader_ctrl, segment_durations, order, t, leader_pt) ||
          !evalBezierCtrl(base_ctrl1, segment_durations, order, t, base_pt1) ||
          !evalBezierCtrl(base_ctrl2, segment_durations, order, t, base_pt2) ||
          !evalBezierCtrl(desired_ctrl1, segment_durations, order, t, desired_pt1) ||
          !evalBezierCtrl(desired_ctrl2, segment_durations, order, t, desired_pt2))
      {
        return out;
      }

      const PayloadConstraintGuardState state = evaluatePayloadConstraintGuard(
          leader_pt,
          interpolatePoint(base_pt1, desired_pt1, clamped_blend),
          interpolatePoint(base_pt2, desired_pt2, clamped_blend),
          params);

      ++out.sample_count;
      if (!state.valid)
        ++out.invalid_samples;

      out.min_area = std::min(out.min_area, state.area);
      out.max_circumradius = std::max(out.max_circumradius, state.circumradius);
      out.min_pair_distance = std::min(out.min_pair_distance, state.min_pair_distance);
    }
    t_prefix += seg_dt;
  }

  if (out.sample_count <= 0)
    return out;

  out.valid = (out.invalid_samples == 0);
  return out;
}

inline PayloadConstraintTrajectoryGuardResult projectPayloadConstraintTrajectoryBlend(
    const Eigen::MatrixXd &leader_ctrl,
    const Eigen::MatrixXd &base_ctrl1,
    const Eigen::MatrixXd &base_ctrl2,
    const Eigen::MatrixXd &desired_ctrl1,
    const Eigen::MatrixXd &desired_ctrl2,
    const std::vector<double> &segment_durations,
    const int order,
    const PayloadConstraintGuardParams &params,
    const int samples_per_segment,
    const int coarse_samples = 24,
    const int binary_refine_iters = 12)
{
  PayloadConstraintTrajectoryGuardResult out;
  out.trajectory = evaluatePayloadConstraintTrajectoryGuard(
      leader_ctrl, base_ctrl1, base_ctrl2, desired_ctrl1, desired_ctrl2,
      segment_durations, order, 0.0, params, samples_per_segment);
  out.base_valid = out.trajectory.valid;

  const PayloadConstraintTrajectoryState desired_state =
      evaluatePayloadConstraintTrajectoryGuard(
          leader_ctrl, base_ctrl1, base_ctrl2, desired_ctrl1, desired_ctrl2,
          segment_durations, order, 1.0, params, samples_per_segment);
  out.desired_valid = desired_state.valid;
  if (desired_state.valid)
  {
    out.blend = 1.0;
    out.trajectory = desired_state;
    return out;
  }

  double best_blend = 0.0;
  PayloadConstraintTrajectoryState best_state = out.trajectory;
  bool found_feasible = false;
  double search_hi = 0.0;

  for (int sample = std::max(1, coarse_samples); sample >= 0; --sample)
  {
    const double blend =
        static_cast<double>(sample) / static_cast<double>(std::max(1, coarse_samples));
    const PayloadConstraintTrajectoryState state =
        evaluatePayloadConstraintTrajectoryGuard(
            leader_ctrl, base_ctrl1, base_ctrl2, desired_ctrl1, desired_ctrl2,
            segment_durations, order, blend, params, samples_per_segment);
    if (!state.valid)
      continue;

    best_blend = blend;
    best_state = state;
    found_feasible = true;
    search_hi =
        std::min(1.0, blend + 1.0 / static_cast<double>(std::max(1, coarse_samples)));
    break;
  }

  if (!found_feasible)
  {
    out.blend = 0.0;
    out.trajectory = best_state;
    return out;
  }

  double lo = best_blend;
  double hi = search_hi;
  for (int iter = 0; iter < std::max(0, binary_refine_iters); ++iter)
  {
    const double mid = 0.5 * (lo + hi);
    const PayloadConstraintTrajectoryState state =
        evaluatePayloadConstraintTrajectoryGuard(
            leader_ctrl, base_ctrl1, base_ctrl2, desired_ctrl1, desired_ctrl2,
            segment_durations, order, mid, params, samples_per_segment);
    if (state.valid)
    {
      lo = mid;
      best_state = state;
    }
    else
    {
      hi = mid;
    }
  }

  out.blend = lo;
  out.trajectory = best_state;
  return out;
}

}  // namespace plan_manage
}  // namespace ego_planner

#endif
