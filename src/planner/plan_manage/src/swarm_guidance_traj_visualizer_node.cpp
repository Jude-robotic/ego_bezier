/**
 * @file swarm_guidance_traj_visualizer_node.cpp
 * @brief 从机引导轨迹可视化节点
 *
 * 功能：
 *  - 订阅两个从机的 guidance_bezier 话题
 *  - 对 Bezier 轨迹进行密集采样，以 LINE_STRIP 形式显示完整轨迹
 *  - 以 SPHERE_LIST 显示 Bezier 控制点（帮助判断轨迹平滑性）
 *  - 以实时 SPHERE 显示"当前期望位置"（t = now - start_time 处的评估值）
 *  - 所有 marker 发布到 /swarm/guidance_viz/markers (MarkerArray)
 *
 * 颜色约定：
 *  agent_1 引导轨迹 —— 青色 (Cyan)      控制点 —— 浅青  当前期望 —— 品红 (Magenta)
 *  agent_2 引导轨迹 —— 黄色 (Yellow)    控制点 —— 浅黄  当前期望 —— 绿色 (Lime)
 *
 * 用法（RViz 中订阅）：
 *   Add → MarkerArray → Topic: /swarm/guidance_viz/markers
 */

#include <ros/ros.h>
#include <ego_planner/Bezier.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
//  Piecewise Bezier Evaluator（与 follower_tracking_diagnosis.py 逻辑完全一致）
// ─────────────────────────────────────────────────────────────────────────────
static bool evalBezier(const ego_planner::Bezier &msg, double t_query,
                       geometry_msgs::Point &result)
{
    const int n_pts = static_cast<int>(msg.pos_pts.size());
    const int n_seg = static_cast<int>(msg.segment_durations.size());
    const int order = msg.order;

    if (n_seg == 0 || n_pts == 0 || order < 1)
        return false;

    // 保证 t >= 0
    double t_local = std::max(0.0, t_query);

    // 找到 t 所在的段
    int seg_idx = n_seg - 1;
    for (int i = 0; i < n_seg; ++i)
    {
        if (t_local <= msg.segment_durations[i] || i == n_seg - 1)
        {
            seg_idx = i;
            break;
        }
        t_local -= msg.segment_durations[i];
    }

    const double dur_seg = msg.segment_durations[seg_idx];
    const double s = (dur_seg < 1e-9) ? 1.0
                                       : std::min(1.0, std::max(0.0, t_local / dur_seg));

    // 取该段的 order+1 个控制点
    const int base = seg_idx * order;
    struct P3 { double x, y, z; };
    std::vector<P3> pts;
    pts.reserve(order + 1);
    for (int k = 0; k <= order; ++k)
    {
        int idx = std::min(base + k, n_pts - 1);
        pts.push_back({msg.pos_pts[idx].x, msg.pos_pts[idx].y, msg.pos_pts[idx].z});
    }

    // de Casteljau 算法
    int sz = static_cast<int>(pts.size());
    while (sz > 1)
    {
        for (int i = 0; i < sz - 1; ++i)
        {
            pts[i].x = (1.0 - s) * pts[i].x + s * pts[i + 1].x;
            pts[i].y = (1.0 - s) * pts[i].y + s * pts[i + 1].y;
            pts[i].z = (1.0 - s) * pts[i].z + s * pts[i + 1].z;
        }
        --sz;
    }

    result.x = pts[0].x;
    result.y = pts[0].y;
    result.z = pts[0].z;
    return true;
}

static double getBezierDuration(const ego_planner::Bezier &msg)
{
    double dur = 0.0;
    for (double d : msg.segment_durations)
        dur += d;
    return dur;
}

// ─────────────────────────────────────────────────────────────────────────────
//  每个 agent 的可视化颜色配置
// ─────────────────────────────────────────────────────────────────────────────
struct AgentColor
{
    // 轨迹曲线
    float traj_r, traj_g, traj_b;
    // 控制点
    float ctrl_r, ctrl_g, ctrl_b;
    // 当前期望位置球
    float des_r, des_g, des_b;
};

// ─────────────────────────────────────────────────────────────────────────────
//  主节点类
// ─────────────────────────────────────────────────────────────────────────────
class SwarmGuidanceTrajVisualizer
{
public:
    SwarmGuidanceTrajVisualizer()
        : nh_(), pnh_("~")
    {
        // 参数
        pnh_.param("publish_rate",     publish_rate_,     30.0);
        pnh_.param("sample_dt",        sample_dt_,        0.05);   // 轨迹采样间隔 (s)
        pnh_.param("traj_line_width",  traj_line_width_,  0.07);
        pnh_.param("ctrl_sphere_size", ctrl_sphere_size_, 0.13);
        pnh_.param("desired_pos_size", desired_pos_size_, 0.30);
        pnh_.param("marker_topic",     marker_topic_,
                   std::string("/swarm/guidance_viz/markers"));

        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            marker_topic_, 5);

        // agent 1: 青色轨迹 / 品红期望位置
        colors_[0] = {0.0f, 1.0f, 1.0f,    // traj: cyan
                      0.4f, 0.9f, 0.9f,    // ctrl: light cyan
                      1.0f, 0.0f, 1.0f};   // desired: magenta

        // agent 2: 黄色轨迹 / 绿色期望位置
        colors_[1] = {1.0f, 1.0f, 0.0f,    // traj: yellow
                      0.9f, 0.9f, 0.3f,    // ctrl: light yellow
                      0.2f, 1.0f, 0.2f};   // desired: lime

        // 订阅两个从机的引导轨迹
        sub_[0] = nh_.subscribe<ego_planner::Bezier>(
            "/swarm/agent_1/guidance_bezier", 5,
            [this](const ego_planner::BezierConstPtr &msg) { onBezier(msg, 0); });

        sub_[1] = nh_.subscribe<ego_planner::Bezier>(
            "/swarm/agent_2/guidance_bezier", 5,
            [this](const ego_planner::BezierConstPtr &msg) { onBezier(msg, 1); });

        timer_ = nh_.createTimer(
            ros::Duration(1.0 / std::max(1.0, publish_rate_)),
            &SwarmGuidanceTrajVisualizer::onTimer, this);

        ROS_INFO("[GuidanceTrajViz] Started.");
        ROS_INFO("[GuidanceTrajViz]   agent_1 -> /swarm/agent_1/guidance_bezier");
        ROS_INFO("[GuidanceTrajViz]   agent_2 -> /swarm/agent_2/guidance_bezier");
        ROS_INFO("[GuidanceTrajViz]   output  -> %s", marker_topic_.c_str());
        ROS_INFO("[GuidanceTrajViz] Colors: agent1=CYAN/MAGENTA, agent2=YELLOW/LIME");
    }

private:
    // ── 收到新 guidance bezier 时重建静态轨迹 markers ────────────────────────
    void onBezier(const ego_planner::BezierConstPtr &msg, int idx)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        latest_[idx]   = *msg;
        has_[idx]      = true;
        dirty_[idx]    = true;  // 标记需要重建轨迹 markers
        recv_count_[idx]++;
        ROS_INFO_THROTTLE(2.0, "[GuidanceTrajViz] agent_%d: new bezier #%d "
                          "(segs=%zu, pts=%zu, dur=%.2fs)",
                          idx + 1, recv_count_[idx],
                          msg->segment_durations.size(),
                          msg->pos_pts.size(),
                          getBezierDuration(*msg));
    }

    // ── 定时器：发布所有 markers ──────────────────────────────────────────────
    void onTimer(const ros::TimerEvent &)
    {
        if (marker_pub_.getNumSubscribers() == 0)
            return;

        visualization_msgs::MarkerArray array_msg;

        // 先发 DELETEALL 清除过期 marker
        {
            visualization_msgs::Marker clr;
            clr.header.frame_id = "world";
            clr.header.stamp    = ros::Time::now();
            clr.action          = visualization_msgs::Marker::DELETEALL;
            array_msg.markers.push_back(clr);
        }

        std::lock_guard<std::mutex> lk(mutex_);

        for (int idx = 0; idx < 2; ++idx)
        {
            if (!has_[idx])
                continue;

            const int agent_id = idx + 1;
            const AgentColor &col = colors_[idx];
            const ego_planner::Bezier &bz = latest_[idx];

            // ── 若 bezier 有更新则重建轨迹 markers ──────────────────────────
            if (dirty_[idx])
            {
                traj_markers_[idx] = buildTrajMarkers(bz, agent_id, col);
                dirty_[idx] = false;
            }

            // 发布轨迹曲线 + 控制点 markers
            for (auto &mk : traj_markers_[idx])
            {
                mk.header.stamp = ros::Time::now();
                array_msg.markers.push_back(mk);
            }

            // ── 实时期望位置（t = now - start_time）────────────────────────
            double t_in_traj = (ros::Time::now() - bz.start_time).toSec();
            double total_dur  = getBezierDuration(bz);

            geometry_msgs::Point des_pos;
            if (evalBezier(bz, t_in_traj, des_pos))
            {
                visualization_msgs::Marker des_mk;
                des_mk.header.frame_id = "world";
                des_mk.header.stamp    = ros::Time::now();
                des_mk.ns              = "guidance_desired_" + std::to_string(agent_id);
                des_mk.id              = 0;
                des_mk.type            = visualization_msgs::Marker::SPHERE;
                des_mk.action          = visualization_msgs::Marker::ADD;
                des_mk.pose.position   = des_pos;
                des_mk.pose.orientation.w = 1.0;
                des_mk.scale.x = desired_pos_size_;
                des_mk.scale.y = desired_pos_size_;
                des_mk.scale.z = desired_pos_size_;
                des_mk.color.r = col.des_r;
                des_mk.color.g = col.des_g;
                des_mk.color.b = col.des_b;
                // 轨迹结束后（t > total_dur）透明度降低提示
                des_mk.color.a = (t_in_traj <= total_dur + 0.5) ? 1.0f : 0.35f;
                des_mk.lifetime = ros::Duration(0.15);  // 若停止发布自动消失

                array_msg.markers.push_back(des_mk);
            }

            // ── 轨迹进度文字标注 ─────────────────────────────────────────────
            {
                geometry_msgs::Point txt_pos;
                evalBezier(bz, std::min(t_in_traj, total_dur), txt_pos);
                txt_pos.z += 0.55;

                char buf[64];
                snprintf(buf, sizeof(buf), "UAV%d  t=%.1f/%.1fs",
                         agent_id, std::max(0.0, t_in_traj), total_dur);

                visualization_msgs::Marker txt_mk;
                txt_mk.header.frame_id   = "world";
                txt_mk.header.stamp      = ros::Time::now();
                txt_mk.ns                = "guidance_label_" + std::to_string(agent_id);
                txt_mk.id                = 0;
                txt_mk.type              = visualization_msgs::Marker::TEXT_VIEW_FACING;
                txt_mk.action            = visualization_msgs::Marker::ADD;
                txt_mk.pose.position     = txt_pos;
                txt_mk.pose.orientation.w = 1.0;
                txt_mk.scale.z           = 0.22;
                txt_mk.color.r           = col.des_r;
                txt_mk.color.g           = col.des_g;
                txt_mk.color.b           = col.des_b;
                txt_mk.color.a           = 1.0f;
                txt_mk.text              = buf;
                txt_mk.lifetime          = ros::Duration(0.15);

                array_msg.markers.push_back(txt_mk);
            }
        }

        marker_pub_.publish(array_msg);
    }

    // ── 根据 bezier 消息构建静态轨迹 markers（曲线线段 + 控制点）───────────
    std::vector<visualization_msgs::Marker> buildTrajMarkers(
        const ego_planner::Bezier &msg, int agent_id, const AgentColor &col)
    {
        std::vector<visualization_msgs::Marker> markers;
        if (msg.pos_pts.empty() || msg.segment_durations.empty())
            return markers;

        const double total_dur = getBezierDuration(msg);

        // ── 1. 轨迹曲线（密集采样 LINE_STRIP）────────────────────────────────
        {
            visualization_msgs::Marker mk;
            mk.header.frame_id  = "world";
            mk.header.stamp     = ros::Time::now();
            mk.ns               = "guidance_traj_" + std::to_string(agent_id);
            mk.id               = 0;
            mk.type             = visualization_msgs::Marker::LINE_STRIP;
            mk.action           = visualization_msgs::Marker::ADD;
            mk.pose.orientation.w = 1.0;
            mk.scale.x          = traj_line_width_;
            mk.color.r          = col.traj_r;
            mk.color.g          = col.traj_g;
            mk.color.b          = col.traj_b;
            mk.color.a          = 0.92f;
            mk.lifetime         = ros::Duration(0);  // 持续显示直到被替换

            for (double t = 0.0; t <= total_dur + sample_dt_ * 0.5; t += sample_dt_)
            {
                geometry_msgs::Point p;
                if (evalBezier(msg, t, p))
                    mk.points.push_back(p);
            }
            markers.push_back(mk);
        }

        // ── 2. 控制点（SPHERE_LIST）────────────────────────────────────────
        {
            visualization_msgs::Marker mk;
            mk.header.frame_id  = "world";
            mk.header.stamp     = ros::Time::now();
            mk.ns               = "guidance_ctrl_" + std::to_string(agent_id);
            mk.id               = 0;
            mk.type             = visualization_msgs::Marker::SPHERE_LIST;
            mk.action           = visualization_msgs::Marker::ADD;
            mk.pose.orientation.w = 1.0;
            mk.scale.x          = ctrl_sphere_size_;
            mk.scale.y          = ctrl_sphere_size_;
            mk.scale.z          = ctrl_sphere_size_;
            mk.color.r          = col.ctrl_r;
            mk.color.g          = col.ctrl_g;
            mk.color.b          = col.ctrl_b;
            mk.color.a          = 0.65f;
            mk.lifetime         = ros::Duration(0);

            for (const auto &pt : msg.pos_pts)
                mk.points.push_back(pt);

            markers.push_back(mk);
        }

        // ── 3. 段边界标记（在每段起点显示小球，帮助判断段间连续性）─────────
        {
            visualization_msgs::Marker mk;
            mk.header.frame_id  = "world";
            mk.header.stamp     = ros::Time::now();
            mk.ns               = "guidance_seg_" + std::to_string(agent_id);
            mk.id               = 0;
            mk.type             = visualization_msgs::Marker::SPHERE_LIST;
            mk.action           = visualization_msgs::Marker::ADD;
            mk.pose.orientation.w = 1.0;
            mk.scale.x          = ctrl_sphere_size_ * 1.6;
            mk.scale.y          = ctrl_sphere_size_ * 1.6;
            mk.scale.z          = ctrl_sphere_size_ * 1.6;
            mk.color.r          = 1.0f;
            mk.color.g          = 1.0f;
            mk.color.b          = 1.0f;
            mk.color.a          = 0.80f;
            mk.lifetime         = ros::Duration(0);

            double t_acc = 0.0;
            for (int s = 0; s < static_cast<int>(msg.segment_durations.size()); ++s)
            {
                geometry_msgs::Point p;
                if (evalBezier(msg, t_acc, p))
                    mk.points.push_back(p);
                t_acc += msg.segment_durations[s];
            }
            // 终点
            {
                geometry_msgs::Point p;
                if (evalBezier(msg, total_dur, p))
                    mk.points.push_back(p);
            }

            markers.push_back(mk);
        }

        return markers;
    }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Publisher  marker_pub_;
    ros::Subscriber sub_[2];
    ros::Timer      timer_;
    std::mutex      mutex_;

    ego_planner::Bezier             latest_[2];
    bool                            has_[2]         = {false, false};
    bool                            dirty_[2]       = {false, false};
    int                             recv_count_[2]  = {0, 0};
    std::vector<visualization_msgs::Marker> traj_markers_[2];

    AgentColor colors_[2];

    std::string marker_topic_;
    double publish_rate_;
    double sample_dt_;
    double traj_line_width_;
    double ctrl_sphere_size_;
    double desired_pos_size_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_guidance_traj_visualizer");
    SwarmGuidanceTrajVisualizer node;
    ros::spin();
    return 0;
}
