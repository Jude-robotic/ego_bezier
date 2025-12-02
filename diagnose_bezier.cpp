#include <iostream>
#include <Eigen/Dense>
#include <vector>

using namespace std;
using namespace Eigen;

// 模拟当前的参数化函数
void parameterizeToBspline_OLD(const double &ts, const vector<Vector3d> &point_set,
                               const vector<Vector3d> &start_end_derivative,
                               MatrixXd &ctrl_pts)
{
    if (point_set.size() < 2) return;
    int K = point_set.size();
    int num_segments = K - 1;
    int num_ctrl_pts = num_segments * 3 + 1;
    ctrl_pts.resize(3, num_ctrl_pts);

    for (int i = 0; i < K; ++i) {
        ctrl_pts.col(i * 3) = point_set[i];
    }

    for (int i = 0; i < K; ++i) {
        Vector3d tangent;
        if (i == 0) {
            tangent = (point_set[1] - point_set[0]); 
            if (start_end_derivative.size() >= 2) tangent = start_end_derivative[1];
        } else if (i == K - 1) {
            tangent = (point_set[K-1] - point_set[K-2]);
            if (start_end_derivative.size() >= 4) tangent = start_end_derivative[3];
        } else {
            tangent = 0.5 * (point_set[i+1] - point_set[i-1]);
        }

        if (i < num_segments) {
            ctrl_pts.col(i * 3 + 1) = point_set[i] + tangent * ts / 3.0;
        }
        if (i > 0) {
            ctrl_pts.col(i * 3 - 1) = point_set[i] - tangent * ts / 3.0;
        }
    }
}

// 正确的参数化函数
void parameterizeToBspline_FIXED(const double &ts, const vector<Vector3d> &point_set,
                                 const vector<Vector3d> &start_end_derivative,
                                 MatrixXd &ctrl_pts)
{
    if (point_set.size() < 2) return;
    int K = point_set.size();
    int num_segments = K - 1;
    int num_ctrl_pts = num_segments * 3 + 1;
    ctrl_pts.resize(3, num_ctrl_pts);

    // Step 1: 设置所有连接点 (P0, P3, P6, ...)
    for (int i = 0; i < K; ++i) {
        ctrl_pts.col(i * 3) = point_set[i];
    }

    // Step 2: 计算每个连接点的切向量
    vector<Vector3d> tangents(K);
    for (int i = 0; i < K; ++i) {
        if (i == 0) {
            // 起点切向量 = 起始速度
            if (start_end_derivative.size() >= 1 && start_end_derivative[0].norm() > 1e-6) {
                tangents[i] = start_end_derivative[0];
            } else {
                tangents[i] = (point_set[1] - point_set[0]) / ts;
            }
        } else if (i == K - 1) {
            // 终点切向量 = 终止速度
            if (start_end_derivative.size() >= 2 && start_end_derivative[1].norm() > 1e-6) {
                tangents[i] = start_end_derivative[1];
            } else {
                tangents[i] = (point_set[K-1] - point_set[K-2]) / ts;
            }
        } else {
            // 中间点切向量 = 两侧差分的平均
            tangents[i] = 0.5 * ((point_set[i+1] - point_set[i]) / ts + (point_set[i] - point_set[i-1]) / ts);
        }
    }

    // Step 3: 设置每段的控制点P1和P2
    // 对于贝塞尔曲线段k: P_{3k}, P_{3k+1}, P_{3k+2}, P_{3k+3}
    // 起点速度 V(0) = 3*(P1 - P0) / T => P1 = P0 + V(0)*T/3
    // 终点速度 V(T) = 3*(P3 - P2) / T => P2 = P3 - V(T)*T/3
    for (int k = 0; k < num_segments; ++k) {
        int idx = k * 3;
        // P1 = P0 + tangent_start * ts / 3
        ctrl_pts.col(idx + 1) = ctrl_pts.col(idx) + tangents[k] * ts / 3.0;
        // P2 = P3 - tangent_end * ts / 3
        ctrl_pts.col(idx + 2) = ctrl_pts.col(idx + 3) - tangents[k + 1] * ts / 3.0;
    }
}

void checkContinuity(const MatrixXd& ctrl_pts, double ts, const string& name) {
    int num_segments = (ctrl_pts.cols() - 1) / 3;
    cout << "\n=== " << name << " 连续性检查 ===" << endl;
    cout << "控制点数量: " << ctrl_pts.cols() << ", 段数: " << num_segments << endl;
    
    for (int k = 0; k < num_segments - 1; ++k) {
        int idx = k * 3;
        // 段k: P(idx), P(idx+1), P(idx+2), P(idx+3)
        // 段k+1: P(idx+3), P(idx+4), P(idx+5), P(idx+6)
        
        // 速度连续性
        Vector3d v_end_k = 3 * (ctrl_pts.col(idx + 3) - ctrl_pts.col(idx + 2)) / ts;
        Vector3d v_start_k1 = 3 * (ctrl_pts.col(idx + 4) - ctrl_pts.col(idx + 3)) / ts;
        double vel_diff = (v_end_k - v_start_k1).norm();
        
        // 加速度连续性
        Vector3d a_end_k = 6 * (ctrl_pts.col(idx + 3) - 2*ctrl_pts.col(idx + 2) + ctrl_pts.col(idx + 1)) / (ts * ts);
        Vector3d a_start_k1 = 6 * (ctrl_pts.col(idx + 5) - 2*ctrl_pts.col(idx + 4) + ctrl_pts.col(idx + 3)) / (ts * ts);
        double acc_diff = (a_end_k - a_start_k1).norm();
        
        cout << "段" << k << "->段" << k+1 << ": 速度差=" << vel_diff << ", 加速度差=" << acc_diff << endl;
    }
}

int main() {
    // 模拟一条简单路径
    vector<Vector3d> point_set;
    point_set.push_back(Vector3d(-18, 0, 0));
    point_set.push_back(Vector3d(-16, 0.5, 0.3));
    point_set.push_back(Vector3d(-14, 1.0, 0.6));
    point_set.push_back(Vector3d(-12, 1.5, 1.0));
    point_set.push_back(Vector3d(-10.9, 2, 1.61));
    
    vector<Vector3d> start_end_derivative;
    start_end_derivative.push_back(Vector3d(0, 0, 0));  // start_vel
    start_end_derivative.push_back(Vector3d(0, 0, 0));  // end_vel
    start_end_derivative.push_back(Vector3d(0, 0, 0));  // start_acc
    start_end_derivative.push_back(Vector3d(0, 0, 0));  // end_acc
    
    double ts = 0.5;
    
    MatrixXd ctrl_pts_old, ctrl_pts_fixed;
    parameterizeToBspline_OLD(ts, point_set, start_end_derivative, ctrl_pts_old);
    parameterizeToBspline_FIXED(ts, point_set, start_end_derivative, ctrl_pts_fixed);
    
    cout << "旧版控制点:\n" << ctrl_pts_old.transpose() << endl;
    cout << "\n新版控制点:\n" << ctrl_pts_fixed.transpose() << endl;
    
    checkContinuity(ctrl_pts_old, ts, "旧版");
    checkContinuity(ctrl_pts_fixed, ts, "新版");
    
    return 0;
}
