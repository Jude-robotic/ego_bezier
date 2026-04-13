#ifndef _COOPERATIVE_PAYLOAD_OPTIMIZER_H_
#define _COOPERATIVE_PAYLOAD_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <plan_env/grid_map.h>

#include <bezier_opt/lbfgs.hpp>
#include <bezier_opt/payload_geometry.h>

#include <vector>

namespace ego_planner
{

enum class CooperativeObstacleType
{
  NONE = 0,
  DOOR_FRAME,
  RING,
  Z_SLIT
};

struct CooperativePayloadOptParams
{
  double w_smooth{10.0};
  double w_feas{1.0};
  double w_uav_obs{100.0};
  double w_leader_ref{50.0};
  double w_mode_shape{30.0};
  double w_payload_feas{120.0};
  double w_payload_obs{160.0};
  double w_sep{40.0};

  double uav_clearance{0.4};
  double max_vel{2.0};
  double max_acc{3.0};

  double rope_length{1.0};
  double payload_radius{0.2};
  double payload_extra_margin{0.05};
  double triangle_area_min{0.05};
  double rope_relax{0.0};
  double triangle_area_relax{0.0};
  double inter_uav_sep_min{0.8};
  double payload_invalid_penalty{2000.0};

  int samples_per_seg{3};
  int max_iterations{60};
  double g_epsilon{1e-3};
  double finite_diff_eps{1e-3};
  double obstacle_search_radius{1.5};
};

struct CooperativeObstacleSpec
{
  CooperativeObstacleType type{CooperativeObstacleType::NONE};
  payload_geometry::LocalFrame frame;
  Eigen::Vector3d forward_axis{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d primary_axis{Eigen::Vector3d::UnitZ()};
  Eigen::Vector3d auxiliary_axis{Eigen::Vector3d::UnitY()};

  double physical_half_extent{0.2};
  double gap_center_y{0.0};
  double gap_width{1.3};
  double z_gap_low{0.6};
  double z_gap_high{2.0};
  double major_radius{1.8};
  double minor_radius{0.4};

  double back_offset{1.1};
  double primary_span{1.0};
  double aux_span{0.2};
  Eigen::Vector3d leader_bias_world{Eigen::Vector3d::Zero()};
};

struct CooperativeWindowInput
{
  Eigen::MatrixXd leader_nominal;
  Eigen::MatrixXd leader_initial;
  Eigen::MatrixXd follower1_initial;
  Eigen::MatrixXd follower2_initial;
  double segment_duration{0.1};
  CooperativeObstacleSpec obstacle;
};

struct CooperativeWindowOutput
{
  bool success{false};
  int result_code{0};
  double final_cost{0.0};
  bool payload_valid{false};
  Eigen::MatrixXd leader_ctrl;
  Eigen::MatrixXd follower1_ctrl;
  Eigen::MatrixXd follower2_ctrl;
};

class CooperativePayloadOptimizer
{
public:
  CooperativePayloadOptimizer() = default;
  ~CooperativePayloadOptimizer() = default;

  void setEnvironment(const GridMap::Ptr &env);
  void setParams(const CooperativePayloadOptParams &params);
  CooperativeWindowOutput optimize(const CooperativeWindowInput &input);

private:
  struct TrajRefs
  {
    Eigen::MatrixXd leader_nominal;
    Eigen::MatrixXd leader_initial;
    Eigen::MatrixXd follower1_initial;
    Eigen::MatrixXd follower2_initial;
  };

  struct VariableIndex
  {
    int agent_idx{0};
    int dim{0};
    int col{0};
  };

  struct EvalCache
  {
    Eigen::MatrixXd leader;
    Eigen::MatrixXd follower1;
    Eigen::MatrixXd follower2;
  };

  struct CostBreakdown
  {
    double smooth{0.0};
    double feas{0.0};
    double uav_obs{0.0};
    double leader_ref{0.0};
    double mode_shape{0.0};
    double payload_feas{0.0};
    double payload_obs{0.0};
    double sep{0.0};
    bool payload_valid{false};
  };

  struct SampleRef
  {
    int base{0};
    double weights[4]{0.0, 0.0, 0.0, 0.0};
    Eigen::Vector3d nominal_leader{Eigen::Vector3d::Zero()};
  };

  static double lbfgsCost(void *instance, const double *x, double *g, const int n);
  double evaluate(const double *x, double *g, bool with_grad, CostBreakdown *breakdown = nullptr) const;
  void unpack(const double *x, EvalCache &cache) const;
  void pack(const EvalCache &cache, std::vector<double> &x) const;

  double payloadObstacleCostOnly(const EvalCache &cache) const;
  void addSmoothnessCost(const Eigen::MatrixXd &q, double ts, double &cost, Eigen::MatrixXd &grad) const;
  void addFeasibilityCost(const Eigen::MatrixXd &q, double ts, double &cost, Eigen::MatrixXd &grad) const;
  void addLeaderReferenceCost(const EvalCache &cache, double &cost, Eigen::MatrixXd &grad_leader) const;
  void addModeShapeCost(const EvalCache &cache,
                        double &cost,
                        Eigen::MatrixXd &grad_leader,
                        Eigen::MatrixXd &grad_f1,
                        Eigen::MatrixXd &grad_f2) const;
  void addPayloadFeasibilityCost(const EvalCache &cache,
                                 double &cost,
                                 Eigen::MatrixXd &grad_leader,
                                 Eigen::MatrixXd &grad_f1,
                                 Eigen::MatrixXd &grad_f2,
                                 bool *payload_valid) const;
  void addSeparationCost(const EvalCache &cache,
                         double &cost,
                         Eigen::MatrixXd &grad_leader,
                         Eigen::MatrixXd &grad_f1,
                         Eigen::MatrixXd &grad_f2) const;
  void addUavObstacleCost(const EvalCache &cache,
                          double &cost,
                          Eigen::MatrixXd &grad_leader,
                          Eigen::MatrixXd &grad_f1,
                          Eigen::MatrixXd &grad_f2) const;

  Eigen::Vector3d evalSample(const Eigen::MatrixXd &ctrl, const SampleRef &sample) const;
  void scatterSampleGradient(const SampleRef &sample,
                             const Eigen::Vector3d &sample_grad,
                             Eigen::MatrixXd &ctrl_grad) const;
  double pointObstacleCostWithClearance(const Eigen::Vector3d &pt,
                                        double clearance,
                                        Eigen::Vector3d *grad_out) const;
  double pointObstacleCost(const Eigen::Vector3d &pt, Eigen::Vector3d *grad_out) const;

private:
  GridMap::Ptr grid_map_;
  CooperativePayloadOptParams params_;
  CooperativeWindowInput input_;
  std::vector<VariableIndex> variable_layout_;
  std::vector<SampleRef> samples_;
};

}  // namespace ego_planner

#endif
