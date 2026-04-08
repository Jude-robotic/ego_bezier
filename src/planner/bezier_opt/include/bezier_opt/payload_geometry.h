#ifndef _PAYLOAD_GEOMETRY_H_
#define _PAYLOAD_GEOMETRY_H_

#include <Eigen/Eigen>

namespace ego_planner
{
namespace payload_geometry
{

struct TriangleAreaGrad
{
  bool valid{false};
  double area{0.0};
  Eigen::Vector3d grad_p0{Eigen::Vector3d::Zero()};
  Eigen::Vector3d grad_p1{Eigen::Vector3d::Zero()};
  Eigen::Vector3d grad_p2{Eigen::Vector3d::Zero()};
};

struct CircumradiusGrad
{
  bool valid{false};
  double circumradius{0.0};
  Eigen::Vector3d grad_p0{Eigen::Vector3d::Zero()};
  Eigen::Vector3d grad_p1{Eigen::Vector3d::Zero()};
  Eigen::Vector3d grad_p2{Eigen::Vector3d::Zero()};
};

struct PayloadSolution
{
  bool valid{false};
  double area{0.0};
  double circumradius{0.0};
  Eigen::Vector3d circumcenter{Eigen::Vector3d::Zero()};
  Eigen::Vector3d plane_normal{Eigen::Vector3d::Zero()};
  Eigen::Vector3d center_lower{Eigen::Vector3d::Zero()};
  Eigen::Vector3d center_upper{Eigen::Vector3d::Zero()};
  Eigen::Vector3d selected_center{Eigen::Vector3d::Zero()};
};

struct LocalFrame
{
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d world_from_local{Eigen::Matrix3d::Identity()};

  inline Eigen::Vector3d toLocal(const Eigen::Vector3d &world_pt) const
  {
    return world_from_local.transpose() * (world_pt - center);
  }

  inline Eigen::Vector3d toWorld(const Eigen::Vector3d &local_pt) const
  {
    return center + world_from_local * local_pt;
  }
};

TriangleAreaGrad computeTriangleAreaAndGrad(const Eigen::Vector3d &p0,
                                            const Eigen::Vector3d &p1,
                                            const Eigen::Vector3d &p2);

CircumradiusGrad computeCircumradiusAndGrad(const Eigen::Vector3d &p0,
                                            const Eigen::Vector3d &p1,
                                            const Eigen::Vector3d &p2);

PayloadSolution solvePayloadCenterLowerBranch(const Eigen::Vector3d &p0,
                                              const Eigen::Vector3d &p1,
                                              const Eigen::Vector3d &p2,
                                              double rope_length);

double doorFrameSignedClearance(const LocalFrame &frame,
                                const Eigen::Vector3d &point_world,
                                double gap_center_y,
                                double gap_width,
                                double payload_radius,
                                double half_extent_x);

double slitSignedClearance(const LocalFrame &frame,
                           const Eigen::Vector3d &point_world,
                           double z_gap_low,
                           double z_gap_high,
                           double payload_radius,
                           double half_extent_x);

double torusSignedDistance(const LocalFrame &frame,
                           const Eigen::Vector3d &point_world,
                           double major_radius,
                           double minor_radius);

double ringSignedClearance(const LocalFrame &frame,
                           const Eigen::Vector3d &point_world,
                           double major_radius,
                           double minor_radius,
                           double payload_radius);

}  // namespace payload_geometry
}  // namespace ego_planner

#endif
