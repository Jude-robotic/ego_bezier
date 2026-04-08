#include "bezier_opt/payload_geometry.h"

#include <algorithm>
#include <cmath>

namespace ego_planner
{
namespace payload_geometry
{

namespace
{
constexpr double kEps = 1e-8;

Eigen::Vector3d safeNormalized(const Eigen::Vector3d &vec)
{
  const double n = vec.norm();
  if (n < kEps)
    return Eigen::Vector3d::Zero();
  return vec / n;
}
}  // namespace

TriangleAreaGrad computeTriangleAreaAndGrad(const Eigen::Vector3d &p0,
                                            const Eigen::Vector3d &p1,
                                            const Eigen::Vector3d &p2)
{
  TriangleAreaGrad out;

  const Eigen::Vector3d cross_vec = (p1 - p0).cross(p2 - p0);
  const double cross_norm = cross_vec.norm();
  out.area = 0.5 * cross_norm;

  if (cross_norm < kEps)
    return out;

  const Eigen::Vector3d normal_hat = cross_vec / cross_norm;
  out.grad_p0 = 0.5 * (p1 - p2).cross(normal_hat);
  out.grad_p1 = 0.5 * (p2 - p0).cross(normal_hat);
  out.grad_p2 = 0.5 * (p0 - p1).cross(normal_hat);
  out.valid = true;
  return out;
}

CircumradiusGrad computeCircumradiusAndGrad(const Eigen::Vector3d &p0,
                                            const Eigen::Vector3d &p1,
                                            const Eigen::Vector3d &p2)
{
  CircumradiusGrad out;
  const TriangleAreaGrad area_grad = computeTriangleAreaAndGrad(p0, p1, p2);
  if (!area_grad.valid || area_grad.area < kEps)
    return out;

  const Eigen::Vector3d d01 = p0 - p1;
  const Eigen::Vector3d d12 = p1 - p2;
  const Eigen::Vector3d d20 = p2 - p0;
  const double l01 = d01.norm();
  const double l12 = d12.norm();
  const double l20 = d20.norm();
  if (l01 < kEps || l12 < kEps || l20 < kEps)
    return out;

  out.circumradius = (l01 * l12 * l20) / std::max(4.0 * area_grad.area, kEps);
  const double inv_area = 1.0 / std::max(area_grad.area, kEps);

  out.grad_p0 = out.circumradius * (d01 / (l01 * l01) - d20 / (l20 * l20) - area_grad.grad_p0 * inv_area);
  out.grad_p1 = out.circumradius * (-d01 / (l01 * l01) + d12 / (l12 * l12) - area_grad.grad_p1 * inv_area);
  out.grad_p2 = out.circumradius * (-d12 / (l12 * l12) + d20 / (l20 * l20) - area_grad.grad_p2 * inv_area);
  out.valid = true;
  return out;
}

PayloadSolution solvePayloadCenterLowerBranch(const Eigen::Vector3d &p0,
                                              const Eigen::Vector3d &p1,
                                              const Eigen::Vector3d &p2,
                                              double rope_length)
{
  PayloadSolution out;
  const TriangleAreaGrad area_grad = computeTriangleAreaAndGrad(p0, p1, p2);
  const CircumradiusGrad circum_grad = computeCircumradiusAndGrad(p0, p1, p2);
  out.area = area_grad.area;
  out.circumradius = circum_grad.circumradius;

  if (!area_grad.valid || !circum_grad.valid)
    return out;

  const Eigen::Vector3d normal = (p1 - p0).cross(p2 - p0);
  const Eigen::Vector3d normal_hat = safeNormalized(normal);
  if (normal_hat.squaredNorm() < kEps)
    return out;

  const Eigen::Vector3d a = p1 - p0;
  const Eigen::Vector3d b = p2 - p0;
  const double a2 = a.squaredNorm();
  const double b2 = b.squaredNorm();
  const Eigen::Vector3d n2 = normal * normal.squaredNorm();
  if (n2.squaredNorm() < kEps)
    return out;

  const Eigen::Vector3d circumcenter =
      p0 + (b2 * normal.cross(a) + a2 * b.cross(normal)) / (2.0 * normal.squaredNorm());

  const double inside = rope_length * rope_length - circum_grad.circumradius * circum_grad.circumradius;
  if (inside < 0.0)
    return out;

  const double height = std::sqrt(std::max(0.0, inside));
  out.circumcenter = circumcenter;
  out.plane_normal = normal_hat;
  out.center_upper = circumcenter + height * normal_hat;
  out.center_lower = circumcenter - height * normal_hat;
  out.selected_center = (out.center_lower.z() <= out.center_upper.z()) ? out.center_lower : out.center_upper;
  out.valid = true;
  return out;
}

double doorFrameSignedClearance(const LocalFrame &frame,
                                const Eigen::Vector3d &point_world,
                                double gap_center_y,
                                double gap_width,
                                double payload_radius,
                                double half_extent_x)
{
  const Eigen::Vector3d local = frame.toLocal(point_world);
  const double slab_margin = std::abs(local.x()) - (half_extent_x + payload_radius);
  if (slab_margin > 0.0)
    return slab_margin;

  return 0.5 * gap_width - payload_radius - std::abs(local.y() - gap_center_y);
}

double slitSignedClearance(const LocalFrame &frame,
                           const Eigen::Vector3d &point_world,
                           double z_gap_low,
                           double z_gap_high,
                           double payload_radius,
                           double half_extent_x)
{
  const Eigen::Vector3d local = frame.toLocal(point_world);
  const double slab_margin = std::abs(local.x()) - (half_extent_x + payload_radius);
  if (slab_margin > 0.0)
    return slab_margin;

  const double lower = local.z() - z_gap_low;
  const double upper = z_gap_high - local.z();
  return std::min(lower, upper) - payload_radius;
}

double torusSignedDistance(const LocalFrame &frame,
                           const Eigen::Vector3d &point_world,
                           double major_radius,
                           double minor_radius)
{
  const Eigen::Vector3d local = frame.toLocal(point_world);
  const double yz_radius = std::sqrt(local.y() * local.y() + local.z() * local.z());
  const double q0 = yz_radius - major_radius;
  const double q1 = local.x();
  return std::sqrt(q0 * q0 + q1 * q1) - minor_radius;
}

double ringSignedClearance(const LocalFrame &frame,
                           const Eigen::Vector3d &point_world,
                           double major_radius,
                           double minor_radius,
                           double payload_radius)
{
  return torusSignedDistance(frame, point_world, major_radius, minor_radius) - payload_radius;
}

}  // namespace payload_geometry
}  // namespace ego_planner
