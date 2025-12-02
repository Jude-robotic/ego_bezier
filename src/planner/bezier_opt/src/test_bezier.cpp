/**
 * @file test_bezier.cpp
 * @brief Test program for Piecewise Bezier curve implementation
 */

#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include <bezier_opt/piecewise_bezier.h>

using namespace std;
using namespace ego_planner;

// 辅助打印函数
void printVec(const Eigen::Vector3d& v, string name) {
    cout << name << ": [" << v(0) << ", " << v(1) << ", " << v(2) << "]" << endl;
}

int main(int argc, char** argv) {
    cout << "========== 开始测试分段贝塞尔曲线基础数学库 ==========" << endl;

    // 1. 构建测试控制点 (3阶贝塞尔，4个点)
    // 假设是一条直线上的点，方便观察： (0,0,0) -> (1,1,1) -> (2,2,2) -> (3,3,3)
    // 为了测试凸包，我们把中间拉高一点
    Eigen::MatrixXd ctrl_pts(3, 4);
    ctrl_pts.col(0) << 0.0, 0.0, 0.0; // P0 起点
    ctrl_pts.col(1) << 1.0, 5.0, 0.0; // P1 拉高Y轴
    ctrl_pts.col(2) << 2.0, 5.0, 0.0; // P2 拉高Y轴
    ctrl_pts.col(3) << 3.0, 0.0, 0.0; // P3 终点

    // 2. 初始化曲线对象
    // order = 3, interval = 1.0 (假设这一段时长为1秒)
    PiecewiseBezier bezier_traj(ctrl_pts, 3, 1.0);

    // ---------------------------------------------------------
    // 测试点 1: 端点插值性 (Endpoint Interpolation)
    // ---------------------------------------------------------
    cout << "\n[Test 1] 端点插值性验证:" << endl;
    Eigen::Vector3d start_pos = bezier_traj.evaluate(0.0); // t=0
    // 结束时间为 segment_duration，这里取 t=1.0
    Eigen::Vector3d end_pos = bezier_traj.evaluate(1.0);   // t=1.0

    printVec(ctrl_pts.col(0), "期望起点 P0");
    printVec(start_pos,       "实际起点 P(0)");
    printVec(ctrl_pts.col(3), "期望终点 P3");
    printVec(end_pos,         "实际终点 P(1)");

    double err_start = (start_pos - ctrl_pts.col(0)).norm();
    double err_end = (end_pos - ctrl_pts.col(3)).norm();

    if (err_start < 1e-4 && err_end < 1e-4) {
        cout << "\033[32m>>> PASS: 端点重合，符合贝塞尔曲线特性。\033[0m" << endl;
    } else {
        cout << "\033[31m>>> FAIL: 端点不重合！时间归一化逻辑有误。\033[0m" << endl;
    }

    // ---------------------------------------------------------
    // 测试点 2: 凸包性质 (Convex Hull Property)
    // ---------------------------------------------------------
    cout << "\n[Test 2] 凸包性质验证:" << endl;
    bool in_convex_hull = true;
    double max_y = 5.0; // P1和P2的Y是5.0
    double min_y = 0.0; // P0和P3的Y是0.0

    for (double t = 0.0; t <= 1.0; t += 0.05) {
        Eigen::Vector3d p = bezier_traj.evaluate(t);
        if (p(1) > max_y + 1e-4 || p(1) < min_y - 1e-4) {
            in_convex_hull = false;
            cout << "t=" << t << " Point out of bounds: " << p.transpose() << endl;
        }
    }

    if (in_convex_hull) {
        cout << "\033[32m>>> PASS: 所有采样点均在凸包范围内。\033[0m" << endl;
    } else {
        cout << "\033[31m>>> FAIL: 曲线超出了控制点定义的范围，基函数公式可能错误。\033[0m" << endl;
    }

    // ---------------------------------------------------------
    // 测试点 3: 速度导数方向 (Derivative Direction)
    // ---------------------------------------------------------
    cout << "\n[Test 3] 起始速度方向验证:" << endl;
    // 贝塞尔曲线起点的切线方向应为 P1 - P0
    // 获取导数对象
    PiecewiseBezier vel_traj = bezier_traj.getDerivative();
    Eigen::Vector3d v_0 = vel_traj.evaluate(0.0);
    
    Eigen::Vector3d p0_to_p1 = ctrl_pts.col(1) - ctrl_pts.col(0);
    
    // 归一化后比较方向
    double dot_prod = v_0.normalized().dot(p0_to_p1.normalized());

    printVec(v_0, "实际速度 V(0)");
    printVec(p0_to_p1, "几何向量 P1-P0");
    cout << "方向点积 (期望接近 1.0): " << dot_prod << endl;

    if (dot_prod > 0.99) {
        cout << "\033[32m>>> PASS: 速度方向正确。\033[0m" << endl;
    } else {
        cout << "\033[31m>>> FAIL: 速度方向与控制点连线不一致。\033[0m" << endl;
    }

    // ---------------------------------------------------------
    // 测试点 4: 多段贝塞尔曲线测试 (Multi-segment Test)
    // ---------------------------------------------------------
    cout << "\n[Test 4] 多段贝塞尔曲线测试:" << endl;
    // 构建两段贝塞尔曲线 (7个控制点 = 3*2+1)
    // Segment 1: P0->P3, Segment 2: P3->P6
    Eigen::MatrixXd multi_ctrl_pts(3, 7);
    multi_ctrl_pts.col(0) << 0.0, 0.0, 0.0;
    multi_ctrl_pts.col(1) << 1.0, 1.0, 0.0;
    multi_ctrl_pts.col(2) << 2.0, 1.0, 0.0;
    multi_ctrl_pts.col(3) << 3.0, 0.0, 0.0; // Junction
    multi_ctrl_pts.col(4) << 4.0, -1.0, 0.0;
    multi_ctrl_pts.col(5) << 5.0, -1.0, 0.0;
    multi_ctrl_pts.col(6) << 6.0, 0.0, 0.0;

    PiecewiseBezier multi_bezier(multi_ctrl_pts, 3, 1.0);
    
    // Check junction point at t=1.0
    Eigen::Vector3d junction_pos = multi_bezier.evaluate(1.0);
    printVec(multi_ctrl_pts.col(3), "期望连接点 P3");
    printVec(junction_pos,          "实际连接点 P(1.0)");
    
    if ((junction_pos - multi_ctrl_pts.col(3)).norm() < 1e-4) {
        cout << "\033[32m>>> PASS: 多段连接点位置正确。\033[0m" << endl;
    } else {
        cout << "\033[31m>>> FAIL: 多段连接点位置错误。\033[0m" << endl;
    }

    // ---------------------------------------------------------
    // 测试点 5: evaluateT 函数测试
    // ---------------------------------------------------------
    cout << "\n[Test 5] evaluateT 函数测试:" << endl;
    double t_min, t_max;
    bezier_traj.getTimeSpan(t_min, t_max);
    
    Eigen::Vector3d pos_t = bezier_traj.evaluateT(t_min);
    printVec(pos_t, "evaluateT(t_min)");
    
    if ((pos_t - ctrl_pts.col(0)).norm() < 1e-4) {
        cout << "\033[32m>>> PASS: evaluateT 在起点正确。\033[0m" << endl;
    } else {
        cout << "\033[31m>>> FAIL: evaluateT 在起点错误。\033[0m" << endl;
    }

    cout << "\n========== 测试完成 ==========" << endl;

    return 0;
}
