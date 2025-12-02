#include "bspline_opt/bspline_optimizer.h"
#include "bspline_opt/gradient_descent_optimizer.h"

namespace ego_planner
{

  /**
   * @brief 从ROS参数服务器读取优化器参数
   * @param nh ROS节点句柄
   */
  void BsplineOptimizer::setParam(ros::NodeHandle &nh)
  {
    // 读取平滑性代价权重
    nh.param("optimization/lambda_smooth", lambda1_, -1.0);
    // 读取碰撞代价权重
    nh.param("optimization/lambda_collision", lambda2_, -1.0);
    // 读取可行性代价权重（速度/加速度约束）
    nh.param("optimization/lambda_feasibility", lambda3_, -1.0);
    // 读取拟合代价权重（轨迹与参考点的偏差）
    nh.param("optimization/lambda_fitness", lambda4_, -1.0);

    // 读取安全距离阈值
    nh.param("optimization/dist0", dist0_, -1.0);
    // 读取最大速度限制
    nh.param("optimization/max_vel", max_vel_, -1.0);
    // 读取最大加速度限制
    nh.param("optimization/max_acc", max_acc_, -1.0);

    // 读取曲线阶数（贝塞尔曲线固定为3阶）
    nh.param("optimization/order", order_, 3);
  }

  /**
   * @brief 设置环境地图（用于碰撞检测）
   * @param env 栅格地图指针
   */
  void BsplineOptimizer::setEnvironment(const GridMap::Ptr &env)
  {
    this->grid_map_ = env;
  }

  /**
   * @brief 设置控制点
   * @param points 控制点矩阵，每列是一个3D控制点
   */
  void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.points = points;
  }

  /**
   * @brief 设置贝塞尔曲线段的时间间隔
   * @param ts 每段贝塞尔曲线的持续时间
   */
  void BsplineOptimizer::setBsplineInterval(const double &ts) { segment_duration_ = ts; }

  /**
   * @brief 初始化控制点并进行A*路径搜索（用于碰撞避免）
   * @param init_points 初始控制点矩阵
   * @param flag_first_init 是否首次初始化
   * @return A*搜索得到的路径集合（当前简化实现返回空）
   * 
   * 说明：对于分段贝塞尔曲线，控制点数量应为 3*N + 1，其中N为段数
   * 每段使用4个控制点：P_{3k}, P_{3k+1}, P_{3k+2}, P_{3k+3}
   */
  std::vector<std::vector<Eigen::Vector3d>> BsplineOptimizer::initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init /*= true*/)
  {
    if (flag_first_init)
    {
      // 设置安全间隙
      cps_.clearance = dist0_;
      // 调整控制点容器大小
      cps_.resize(init_points.cols());
      // 复制初始控制点
      cps_.points = init_points;
    }
    
    // 当前简化实现：返回空路径
    // 完整实现需要对贝塞尔曲线段进行碰撞检测和A*搜索
    return vector<vector<Eigen::Vector3d>>();
  }

  /**
   * @brief L-BFGS优化器的提前退出回调函数
   * @param func_data 优化器对象指针
   * @param x 当前优化变量
   * @param g 当前梯度
   * @param fx 当前代价值
   * @param xnorm 变量范数
   * @param gnorm 梯度范数
   * @param step 步长
   * @param n 变量数量
   * @param k 迭代次数
   * @param ls 线搜索次数
   * @return 非零值表示需要提前退出优化
   */
  int BsplineOptimizer::earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
    // 如果遇到错误或需要反弹（碰撞处理），则提前退出
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  /**
   * @brief Rebound优化的代价函数（包含碰撞避免）
   * @param func_data 优化器对象指针
   * @param x 当前优化变量（控制点坐标）
   * @param grad 输出梯度
   * @param n 变量数量
   * @return 总代价值
   */
  double BsplineOptimizer::costFunctionRebound(void *func_data, const double *x, double *grad, const int n)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
    double cost;
    // 计算组合代价（平滑性 + 碰撞 + 可行性）
    opt->combineCostRebound(x, grad, cost, n);
    // 增加迭代计数
    opt->iter_num_ += 1;
    return cost;
  }

  /**
   * @brief Refine优化的代价函数（轨迹精化）
   * @param func_data 优化器对象指针
   * @param x 当前优化变量（控制点坐标）
   * @param grad 输出梯度
   * @param n 变量数量
   * @return 总代价值
   */
  double BsplineOptimizer::costFunctionRefine(void *func_data, const double *x, double *grad, const int n)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
    double cost;
    // 计算组合代价（平滑性 + 拟合 + 可行性）
    opt->combineCostRefine(x, grad, cost, n);
    // 增加迭代计数
    opt->iter_num_ += 1;
    return cost;
  }

  /**
   * @brief 计算距离代价（碰撞避免）
   * @param q 控制点矩阵
   * @param cost 输出代价值
   * @param gradient 输出梯度矩阵
   * @param iter_num 当前迭代次数
   * @param smoothness_cost 平滑性代价（用于调整权重）
   * 
   * 说明：使用预计算的障碍物方向和基准点，惩罚控制点进入障碍物区域
   * 代价函数采用三次多项式形式，在安全距离阈值处平滑过渡
   */
  void BsplineOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost)
  {
    cost = 0.0;
    // 遍历所有控制点
    int end_idx = q.cols(); 
    // 分界距离（安全间隙）
    double demarcation = cps_.clearance;
    // 三次多项式系数，保证在demarcation处连续
    double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);

    for (int i = 0; i < end_idx; ++i)
    {
      // 安全检查：确保方向向量已初始化
      if (i >= cps_.direction.size()) continue;
      
      // 遍历该控制点的所有约束方向
      for (size_t j = 0; j < cps_.direction[i].size(); ++j)
      {
        // 计算控制点到基准点的有符号距离（沿约束方向投影）
        double dist = (q.col(i) - cps_.base_point[i][j]).dot(cps_.direction[i][j]);
        // 距离误差 = 安全间隙 - 实际距离
        double dist_err = cps_.clearance - dist;
        // 梯度方向
        Eigen::Vector3d dist_grad = cps_.direction[i][j];

        if (dist_err < 0)
        {
          // 控制点在安全区域外，无需惩罚
        }
        else if (dist_err < demarcation)
        {
          // 控制点接近障碍物，使用三次惩罚
          cost += pow(dist_err, 3);
          gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;
        }
        else
        {
          // 控制点深入障碍物区域，使用更强的二次惩罚
          cost += a * dist_err * dist_err + b * dist_err + c;
          gradient.col(i) += -(2.0 * a * dist_err + b) * dist_grad;
        }
      }
    }
  }

  /**
   * @brief 计算拟合代价（轨迹与参考点的偏差）
   * @param q 控制点矩阵
   * @param cost 输出代价值
   * @param gradient 输出梯度矩阵
   * 
   * 说明：最小化控制点与参考点之间的欧氏距离平方和
   * 用于轨迹精化阶段，保持轨迹接近原始规划
   */
  void BsplineOptimizer::calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {
    cost = 0.0;
    
    int end_idx = q.cols();
    // 检查参考点数量是否匹配
    if (ref_pts_.size() != end_idx) return;

    for (int i = 0; i < end_idx; ++i)
    {
      // 计算控制点与参考点的偏差
      Eigen::Vector3d diff = q.col(i) - ref_pts_[i];
      // 二次代价
      cost += diff.squaredNorm();
      // 梯度 = 2 * 偏差向量
      gradient.col(i) += 2 * diff;
    }
  }

  /**
   * @brief 计算平滑性代价（最小化jerk并保证段间连续性）
   * @param q 控制点矩阵
   * @param cost 输出代价值
   * @param gradient 输出梯度矩阵
   * @param falg_use_jerk 是否使用jerk代价（未使用）
   * 
   * 说明：分段贝塞尔曲线的平滑性包含两部分：
   * 1. 段内jerk最小化：使轨迹平滑
   * 2. 段间连续性约束：保证速度和加速度在连接点连续
   * 
   * 控制点结构：[P0, P1, P2, P3, P4, P5, P6, ...]
   *            |---段0---|---段1---|
   * 段k使用控制点：P_{3k}, P_{3k+1}, P_{3k+2}, P_{3k+3}
   */
  void BsplineOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                            Eigen::MatrixXd &gradient, bool falg_use_jerk)
  {
    cost = 0.0;
    // 计算贝塞尔曲线段数：(控制点数-1)/3
    int num_segments = (q.cols() - 1) / 3;
    // 每段的时间间隔
    double ts = segment_duration_;
    // 预计算时间幂次的倒数
    double ts_inv3 = 1.0 / pow(ts, 3);
    double ts_inv2 = 1.0 / pow(ts, 2);
    double ts_inv = 1.0 / ts;

    // ==================== 第一部分：段内Jerk代价 ====================
    // 三次贝塞尔曲线的jerk（三阶导数）是常数
    // Jerk = 6 * (P3 - 3*P2 + 3*P1 - P0) / T^3
    // 代价 = 积分(Jerk^2 * dt) = Jerk^2 * T
    for (int k = 0; k < num_segments; ++k)
    {
        // 第k段的起始控制点索引
        int idx = k * 3;
        // 获取该段的4个控制点
        Eigen::Vector3d p0 = q.col(idx);
        Eigen::Vector3d p1 = q.col(idx + 1);
        Eigen::Vector3d p2 = q.col(idx + 2);
        Eigen::Vector3d p3 = q.col(idx + 3);

        // Jerk向量（未除以时间）
        Eigen::Vector3d jerk_vec = (p3 - 3 * p2 + 3 * p1 - p0);
        // 权重 = (6/T^3)^2 * T = 36/T^5
        double weight = 36.0 / pow(ts, 5); 
        
        // 代价 = weight * |jerk_vec|^2
        cost += weight * jerk_vec.squaredNorm();

        // 梯度 = 2 * weight * jerk_vec
        Eigen::Vector3d grad_j = 2.0 * weight * jerk_vec;
        
        // 对各控制点的梯度贡献（链式法则）
        // d(jerk_vec)/dP0 = -1, d(jerk_vec)/dP1 = 3, d(jerk_vec)/dP2 = -3, d(jerk_vec)/dP3 = 1
        gradient.col(idx)     += -grad_j;
        gradient.col(idx + 1) +=  3.0 * grad_j;
        gradient.col(idx + 2) += -3.0 * grad_j;
        gradient.col(idx + 3) +=  grad_j;
    }

    // ==================== 第二部分：段间连续性约束 ====================
    // 使用强惩罚项保证相邻段在连接点的速度和加速度连续
    
    // 速度连续性权重
    double cont_weight_vel = 1000.0;
    // 加速度连续性权重
    double cont_weight_acc = 1000.0;

    for (int k = 0; k < num_segments - 1; ++k)
    {
        int idx = k * 3;
        // 连接点为 P_{idx+3}
        // 段k末端控制点：P2=q(idx+2), P3=q(idx+3)
        // 段k+1起始控制点：P3=q(idx+3), P4=q(idx+4)
        
        // ---------- 速度连续性 ----------
        // 段k末端速度：V_end = 3*(P3-P2)/T
        // 段k+1起始速度：V_start = 3*(P4-P3)/T
        // 连续性条件：V_end = V_start
        // 即：P3-P2 = P4-P3 => P4 - 2*P3 + P2 = 0
        Eigen::Vector3d diff_vel = (q.col(idx + 4) - 2 * q.col(idx + 3) + q.col(idx + 2));
        // 惩罚偏差的平方
        cost += cont_weight_vel * diff_vel.squaredNorm();
        
        // 计算梯度
        Eigen::Vector3d grad_v = 2.0 * cont_weight_vel * diff_vel;
        gradient.col(idx + 4) += grad_v;        // d(diff_vel)/dP4 = 1
        gradient.col(idx + 3) += -2.0 * grad_v; // d(diff_vel)/dP3 = -2
        gradient.col(idx + 2) += grad_v;        // d(diff_vel)/dP2 = 1

        // ---------- 加速度连续性 ----------
        // 段k末端加速度：A_end = 6*(P3-2*P2+P1)/T^2
        // 段k+1起始加速度：A_start = 6*(P5-2*P4+P3)/T^2
        // 连续性条件：A_end = A_start
        // 即：P3-2*P2+P1 = P5-2*P4+P3 => P1 - 2*P2 + 2*P4 - P5 = 0
        Eigen::Vector3d diff_acc = (q.col(idx + 1) - 2 * q.col(idx + 2) + 2 * q.col(idx + 4) - q.col(idx + 5));
        // 惩罚偏差的平方
        cost += cont_weight_acc * diff_acc.squaredNorm();

        // 计算梯度
        Eigen::Vector3d grad_a = 2.0 * cont_weight_acc * diff_acc;
        gradient.col(idx + 1) += grad_a;        // d(diff_acc)/dP1 = 1
        gradient.col(idx + 2) += -2.0 * grad_a; // d(diff_acc)/dP2 = -2
        gradient.col(idx + 4) += 2.0 * grad_a;  // d(diff_acc)/dP4 = 2
        gradient.col(idx + 5) += -grad_a;       // d(diff_acc)/dP5 = -1
    }
  }

  /**
   * @brief 计算可行性代价（速度和加速度约束）
   * @param q 控制点矩阵
   * @param cost 输出代价值
   * @param gradient 输出梯度矩阵
   * 
   * 说明：惩罚超过物理限制的速度和加速度
   * 
   * 三次贝塞尔曲线的导数性质：
   * - 速度是二次贝塞尔曲线，有3个速度控制点 V0, V1, V2
   *   Vi = 3*(P_{i+1} - P_i)/T
   * - 加速度是线性贝塞尔曲线，有2个加速度控制点 A0, A1
   *   Ai = 6*(P_{i+2} - 2*P_{i+1} + P_i)/T^2
   * 
   * 由于贝塞尔曲线的凸包性质，实际速度/加速度不会超过控制点的范围
   */
  void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                             Eigen::MatrixXd &gradient)
  {
    cost = 0.0;
    double ts = segment_duration_;
    // 贝塞尔曲线段数
    int num_segments = (q.cols() - 1) / 3;

    // 遍历每个贝塞尔曲线段
    for (int k = 0; k < num_segments; ++k)
    {
        // 第k段的起始控制点索引
        int idx = k * 3;
        // 获取该段的4个控制点
        Eigen::Vector3d p0 = q.col(idx);
        Eigen::Vector3d p1 = q.col(idx + 1);
        Eigen::Vector3d p2 = q.col(idx + 2);
        Eigen::Vector3d p3 = q.col(idx + 3);

        // ==================== 速度约束 ====================
        // 速度控制点：V0 = 3*(P1-P0)/T, V1 = 3*(P2-P1)/T, V2 = 3*(P3-P2)/T
        vector<Eigen::Vector3d> vs = {(p1-p0)*3/ts, (p2-p1)*3/ts, (p3-p2)*3/ts};
        // 速度控制点对应的位置控制点索引对
        vector<pair<int, int>> v_indices = {{idx, idx+1}, {idx+1, idx+2}, {idx+2, idx+3}};

        // 检查每个速度控制点的每个分量
        for(int i=0; i<3; ++i) {
            Eigen::Vector3d v = vs[i];
            for(int dim=0; dim<3; ++dim) {
                // 如果速度超过限制
                if(abs(v(dim)) > max_vel_) {
                    // 超出量
                    double diff = abs(v(dim)) - max_vel_;
                    // 二次惩罚
                    cost += pow(diff, 2);
                    // 梯度：2 * diff * sign(v)
                    double grad = 2 * diff * (v(dim) > 0 ? 1 : -1);
                    // 链式法则：dC/dP = dC/dV * dV/dP
                    // V = 3*(P_b - P_a)/T => dV/dP_b = 3/T, dV/dP_a = -3/T
                    gradient(dim, v_indices[i].second) += grad * 3.0 / ts;
                    gradient(dim, v_indices[i].first)  += grad * -3.0 / ts;
                }
            }
        }

        // ==================== 加速度约束 ====================
        // 加速度控制点：A0 = 6*(P2-2*P1+P0)/T^2, A1 = 6*(P3-2*P2+P1)/T^2
        vector<Eigen::Vector3d> as = {(p2 - 2*p1 + p0)*6/(ts*ts), (p3 - 2*p2 + p1)*6/(ts*ts)};
        // 加速度控制点对应的位置控制点索引（系数为1, -2, 1）
        vector<vector<int>> a_indices = {{idx, idx+1, idx+2}, {idx+1, idx+2, idx+3}};

        // 检查每个加速度控制点的每个分量
        for(int i=0; i<2; ++i) {
            Eigen::Vector3d a = as[i];
            for(int dim=0; dim<3; ++dim) {
                // 如果加速度超过限制
                if(abs(a(dim)) > max_acc_) {
                    // 超出量
                    double diff = abs(a(dim)) - max_acc_;
                    // 二次惩罚
                    cost += pow(diff, 2);
                    // 梯度：2 * diff * sign(a)
                    double grad = 2 * diff * (a(dim) > 0 ? 1 : -1);
                    // 链式法则因子
                    double factor = 6.0 / (ts*ts);
                    
                    // A = 6*(P_c - 2*P_b + P_a)/T^2
                    // dA/dP_a = 6/T^2, dA/dP_b = -12/T^2, dA/dP_c = 6/T^2
                    gradient(dim, a_indices[i][0]) += grad * factor * 1.0;
                    gradient(dim, a_indices[i][1]) += grad * factor * -2.0;
                    gradient(dim, a_indices[i][2]) += grad * factor * 1.0;
                }
            }
        }
    }
  }

  /**
   * @brief 检查碰撞并更新反弹方向
   * @return 是否存在碰撞
   * 
   * 说明：当前为简化实现，完整实现需要：
   * 1. 对每个贝塞尔曲线段进行碰撞检测
   * 2. 更新 cps_.base_point 和 cps_.direction 用于距离代价计算
   */
  bool BsplineOptimizer::check_collision_and_rebound(void)
  {
      return false; 
  }

  /**
   * @brief 执行Rebound轨迹优化（碰撞避免优化）
   * @param optimal_points 输出优化后的控制点
   * @param ts 贝塞尔曲线段的时间间隔
   * @return 优化是否成功
   */
  bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts)
  {
    // 设置时间间隔
    setBsplineInterval(ts);
    // 执行优化
    bool flag_success = rebound_optimize();
    // 返回优化后的控制点
    optimal_points = cps_.points;
    return flag_success;
  }

  /**
   * @brief 执行Refine轨迹优化（轨迹精化）
   * @param init_points 初始控制点
   * @param ts 贝塞尔曲线段的时间间隔
   * @param optimal_points 输出优化后的控制点
   * @return 优化是否成功
   */
  bool BsplineOptimizer::BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points)
  {
    // 设置初始控制点
    setControlPoints(init_points);
    // 设置时间间隔
    setBsplineInterval(ts);
    // 执行优化
    bool flag_success = refine_optimize();
    // 返回优化后的控制点
    optimal_points = cps_.points;
    return flag_success;
  }

  /**
   * @brief 执行Rebound优化的核心函数
   * @return 优化是否成功
   * 
   * 说明：使用L-BFGS算法优化控制点位置
   * 目标：最小化 平滑性代价 + 碰撞代价 + 可行性代价
   */
  bool BsplineOptimizer::rebound_optimize()
  {
    // 重置迭代计数器
    iter_num_ = 0;
    // 优化变量数量 = 控制点数 * 3（每个控制点3个坐标）
    variable_num_ = cps_.points.size();
    double final_cost;
    
    // 将控制点矩阵展开为一维数组（L-BFGS要求）
    std::vector<double> q(variable_num_);
    Eigen::Map<Eigen::MatrixXd>(q.data(), 3, cps_.points.cols()) = cps_.points;

    // 配置L-BFGS优化器参数
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.max_iterations = 500;   // 最大迭代次数
    lbfgs_params.g_epsilon = 1e-3;       // 放宽收敛阈值
    lbfgs_params.mem_size = 16;          // 增加历史记忆大小

    // 调用L-BFGS优化器
    int result = lbfgs::lbfgs_optimize(variable_num_, q.data(), &final_cost, 
        BsplineOptimizer::costFunctionRebound, NULL, BsplineOptimizer::earlyExit, 
        this, &lbfgs_params);
    
    // 检查优化结果
    if (result != lbfgs::LBFGS_CONVERGENCE && result != lbfgs::LBFGS_STOP && 
        result != lbfgs::LBFGS_ALREADY_MINIMIZED)
    {
        std::cout << "[BsplineOptimizer] Rebound optimization: " << lbfgs::lbfgs_strerror(result) 
                  << " (" << result << "), final_cost=" << final_cost << std::endl;
    }

    // 将优化结果复制回控制点矩阵
    cps_.points = Eigen::Map<Eigen::MatrixXd>(q.data(), 3, cps_.points.cols());
    
    // 返回优化是否成功（即使达到最大迭代次数，如果代价足够低也认为成功）
    bool success = (result == lbfgs::LBFGS_CONVERGENCE || result == lbfgs::LBFGS_STOP || 
                    result == lbfgs::LBFGS_ALREADY_MINIMIZED);
    
    // 如果达到最大迭代次数但最终代价较低，也认为成功
    if (result == lbfgs::LBFGSERR_MAXIMUMITERATION && final_cost < 10.0) {
        std::cout << "[BsplineOptimizer] Max iterations reached but cost is low, accepting result" << std::endl;
        success = true;
    }
    
    return success;
  }
  bool BsplineOptimizer::refine_optimize()
  {
    // 重置迭代计数器
    iter_num_ = 0;
    // 优化变量数量
    variable_num_ = cps_.points.size();
    double final_cost;

    // 将控制点矩阵展开为一维数组
    std::vector<double> q(variable_num_);
    Eigen::Map<Eigen::MatrixXd>(q.data(), 3, cps_.points.cols()) = cps_.points;

    // 配置L-BFGS优化器参数
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.max_iterations = 500;
    lbfgs_params.g_epsilon = 1e-3;
    lbfgs_params.mem_size = 16;

    // 调用L-BFGS优化器（Refine不需要提前退出回调）
    int result = lbfgs::lbfgs_optimize(variable_num_, q.data(), &final_cost, 
        BsplineOptimizer::costFunctionRefine, NULL, NULL, this, &lbfgs_params);
    
    // 检查优化结果
    if (result != lbfgs::LBFGS_CONVERGENCE && result != lbfgs::LBFGS_STOP && 
        result != lbfgs::LBFGS_ALREADY_MINIMIZED)
    {
        std::cout << "[BsplineOptimizer] Refine optimization: " << lbfgs::lbfgs_strerror(result) 
                  << " (" << result << "), final_cost=" << final_cost << std::endl;
    }

    // 将优化结果复制回控制点矩阵
    cps_.points = Eigen::Map<Eigen::MatrixXd>(q.data(), 3, cps_.points.cols());

    // 返回优化是否成功
    bool success = (result == lbfgs::LBFGS_CONVERGENCE || result == lbfgs::LBFGS_STOP || 
                    result == lbfgs::LBFGS_ALREADY_MINIMIZED);
    
    // 如果达到最大迭代次数但最终代价较低，也认为成功
    if (result == lbfgs::LBFGSERR_MAXIMUMITERATION && final_cost < 10.0) {
        success = true;
    }
    
    return success;
  }
  void BsplineOptimizer::combineCostRebound(const double *x, double *grad, double &f_combine, const int n)
  {
    // 将一维数组映射为矩阵形式（不复制数据）
    Eigen::Map<const Eigen::MatrixXd> q(x, 3, cps_.points.cols());
    Eigen::Map<Eigen::MatrixXd> grad_mat(grad, 3, cps_.points.cols());
    // 初始化梯度为零
    grad_mat.setZero();

    // 各项代价和梯度
    double f_smoothness, f_distance, f_feasibility;
    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.points.cols());

    // 计算各项代价
    calcSmoothnessCost(q, f_smoothness, g_smoothness);
    calcDistanceCostRebound(q, f_distance, g_distance, iter_num_, f_smoothness);
    calcFeasibilityCost(q, f_feasibility, g_feasibility);

    // 加权求和得到总代价
    f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility;
    // 加权求和得到总梯度
    grad_mat = lambda1_ * g_smoothness + new_lambda2_ * g_distance + lambda3_ * g_feasibility;
    
    // 诊断输出（每20次迭代输出一次）
    if (iter_num_ % 20 == 0) {
        std::cout << "[Opt iter " << iter_num_ << "] smooth=" << f_smoothness 
                  << " dist=" << f_distance << " feas=" << f_feasibility
                  << " total=" << f_combine 
                  << " grad_norm=" << grad_mat.norm() << std::endl;
    }
  }
  /**
   * @brief 组合Refine优化的各项代价
   * @param x 当前优化变量（控制点坐标数组）
   * @param grad 输出梯度数组
   * @param f_combine 输出总代价值
   * @param n 变量数量
   * 
   * 说明：总代价 = λ1*平滑性代价 + λ4*拟合代价 + λ3*可行性代价
   */
  void BsplineOptimizer::combineCostRefine(const double *x, double *grad, double &f_combine, const int n)
  {
    // 将一维数组映射为矩阵形式
    Eigen::Map<const Eigen::MatrixXd> q(x, 3, cps_.points.cols());
    Eigen::Map<Eigen::MatrixXd> grad_mat(grad, 3, cps_.points.cols());
    // 初始化梯度为零
    grad_mat.setZero();

    // 各项代价和梯度
    double f_smoothness, f_fitness, f_feasibility;
    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_fitness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.points.cols());

    // 计算各项代价
    calcSmoothnessCost(q, f_smoothness, g_smoothness);
    calcFitnessCost(q, f_fitness, g_fitness);
    calcFeasibilityCost(q, f_feasibility, g_feasibility);

    // 加权求和得到总代价
    f_combine = lambda1_ * f_smoothness + lambda4_ * f_fitness + lambda3_ * f_feasibility;
    // 加权求和得到总梯度
    grad_mat = lambda1_ * g_smoothness + lambda4_ * g_fitness + lambda3_ * g_feasibility;
  }

} // namespace ego_planner
