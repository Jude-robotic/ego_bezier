#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <random>

using namespace std;

// pcl::search::KdTree<pcl::PointXYZ> kdtreeLocalMap;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

random_device rd;
//default_random_engine eng(0);
default_random_engine eng(rd()); 
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_w;
uniform_real_distribution<double> rand_h;
uniform_real_distribution<double> rand_inf;

ros::Publisher _local_map_pub;
ros::Publisher _all_map_pub;
ros::Publisher click_map_pub_;
ros::Subscriber _odom_sub;

vector<double> _state;

int _obs_num;
double _x_size, _y_size, _z_size;
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
double _z_limit, _sensing_range, _resolution, _sense_rate, _init_x, _init_y;
double _min_dist;

bool _map_ok = false;
bool _has_odom = false;

int circle_num_;
double radius_l_, radius_h_, z_l_, z_h_;
double theta_;
uniform_real_distribution<double> rand_radius_;
uniform_real_distribution<double> rand_radius2_;
uniform_real_distribution<double> rand_theta_;
uniform_real_distribution<double> rand_z_;

sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

sensor_msgs::PointCloud2 localMap_pcd;
pcl::PointCloud<pcl::PointXYZ> clicked_cloud_;

void RandomMapGenerate() {
  pcl::PointXYZ pt_random;

  rand_x = uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);

  rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
  rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
  rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
  rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

  // generate polar obs
  for (int i = 0; i < _obs_num; i++) {
    double x, y, w, h;
    x = rand_x(eng);
    y = rand_y(eng);
    w = rand_w(eng);

    if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0) {
      i--;
      continue;
    }

    if (sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0) {
      i--;
      continue;
    }

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    int widNum = ceil(w / _resolution);

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
        h = rand_h(eng);
        int heiNum = ceil(h / _resolution);
        for (int t = -20; t < heiNum; t++) {
          pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
          pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
          pt_random.z = (t + 0.5) * _resolution + 1e-2;
          cloudMap.points.push_back(pt_random);
        }
      }
  }

  // generate circle obs
  for (int i = 0; i < circle_num_; ++i) {
    double x, y, z;
    x = rand_x(eng);
    y = rand_y(eng);
    z = rand_z_(eng);

    if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0) {
      i--;
      continue;
    }

    if (sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0) {
      i--;
      continue;
    }

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    Eigen::Vector3d translate(x, y, z);

    double theta = rand_theta_(eng);
    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
        1;

    double radius1 = rand_radius_(eng);
    double radius2 = rand_radius2_(eng);

    // draw a circle centered at (x,y,z)
    Eigen::Vector3d cpt;
    for (double angle = 0.0; angle < 6.282; angle += _resolution / 2) {
      cpt(0) = 0.0;
      cpt(1) = radius1 * cos(angle);
      cpt(2) = radius2 * sin(angle);

      // inflate
      Eigen::Vector3d cpt_if;
      for (int ifx = -0; ifx <= 0; ++ifx)
        for (int ify = -0; ify <= 0; ++ify)
          for (int ifz = -0; ifz <= 0; ++ifz) {
            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                           ifz * _resolution);
            cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
            pt_random.x = cpt_if(0);
            pt_random.y = cpt_if(1);
            pt_random.z = cpt_if(2);
            cloudMap.push_back(pt_random);
          }
    }
  }

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  ROS_WARN("Finished generate random map ");

  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;
}

void RandomMapGenerateCylinder() {
  pcl::PointXYZ pt_random;

  vector<Eigen::Vector2d> obs_position;

  rand_x = uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);
  rand_inf = uniform_real_distribution<double>(0.5, 1.5);

  rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
  rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
  rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
  rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

  // generate polar obs
  for (int i = 0; i < _obs_num && ros::ok(); i++) {
    double x, y, w, h, inf;
    x = rand_x(eng);
    y = rand_y(eng);
    w = rand_w(eng);
    inf = rand_inf(eng);

    if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0) {
      i--;
      continue;
    }

    if (sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0) {
      i--;
      continue;
    }
    
    bool flag_continue = false;
    for ( auto p : obs_position )
      if ( (Eigen::Vector2d(x,y) - p).norm() < _min_dist /*metres*/ )
      {
        i--;
        flag_continue = true;
        break;
      }
    if ( flag_continue ) continue;

    obs_position.push_back( Eigen::Vector2d(x,y) );
    

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    int widNum = ceil((w*inf) / _resolution);
    double radius = (w*inf) / 2;

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
        h = rand_h(eng);
        int heiNum = ceil(h / _resolution);
        for (int t = -30; t < heiNum; t++) {
          double temp_x = x + (r + 0.5) * _resolution + 1e-2;
          double temp_y = y + (s + 0.5) * _resolution + 1e-2;
          double temp_z = (t + 0.5) * _resolution + 1e-2;
          if ( (Eigen::Vector2d(temp_x,temp_y) - Eigen::Vector2d(x,y)).norm() <= radius )
          {
            pt_random.x = temp_x;
            pt_random.y = temp_y;
            pt_random.z = temp_z;
            cloudMap.points.push_back(pt_random);
          }
        }
      }
  }

  // generate circle obs
  for (int i = 0; i < circle_num_; ++i) {
    double x, y, z;
    x = rand_x(eng);
    y = rand_y(eng);
    z = rand_z_(eng);

    if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0) {
      i--;
      continue;
    }

    if (sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0) {
      i--;
      continue;
    }

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    Eigen::Vector3d translate(x, y, z);

    double theta = rand_theta_(eng);
    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
        1;

    double radius1 = rand_radius_(eng);
    double radius2 = rand_radius2_(eng);

    // draw a circle centered at (x,y,z)
    Eigen::Vector3d cpt;
    for (double angle = 0.0; angle < 6.282; angle += _resolution / 2) {
      cpt(0) = 0.0;
      cpt(1) = radius1 * cos(angle);
      cpt(2) = radius2 * sin(angle);

      // inflate
      Eigen::Vector3d cpt_if;
      for (int ifx = -0; ifx <= 0; ++ifx)
        for (int ify = -0; ify <= 0; ++ify)
          for (int ifz = -0; ifz <= 0; ++ifz) {
            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                           ifz * _resolution);
            cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
            pt_random.x = cpt_if(0);
            pt_random.y = cpt_if(1);
            pt_random.z = cpt_if(2);
            cloudMap.push_back(pt_random);
          }
    }
  }

  // generate floor 
  // pcl::PointXYZ pt;
  // pt.z = 0.1;
  // for ( pt.x = _x_l; pt.x <= _x_h; pt.x += _resolution )
  //   for ( pt.y = _y_l; pt.y <= _y_h; pt.y += _resolution )
  //   {
  //     cloudMap.push_back(pt);
  //   }

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  ROS_WARN("Finished generate random map ");

  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;
}

void rcvOdometryCallbck(const nav_msgs::Odometry odom) {
  if (odom.child_frame_id == "X" || odom.child_frame_id == "O") return;
  _has_odom = true;

  _state = {odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z,
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z,
            0.0,
            0.0,
            0.0};
}

int i = 0;
void pubSensedPoints() {
  // if (i < 10) {
  pcl::toROSMsg(cloudMap, globalMap_pcd);
  globalMap_pcd.header.frame_id = "world";
  _all_map_pub.publish(globalMap_pcd);
  // }

  return;

  /* ---------- only publish points around current position ---------- */
  if (!_map_ok || !_has_odom) return;

  pcl::PointCloud<pcl::PointXYZ> localMap;

  pcl::PointXYZ searchPoint(_state[0], _state[1], _state[2]);
  pointIdxRadiusSearch.clear();
  pointRadiusSquaredDistance.clear();

  pcl::PointXYZ pt;

  if (isnan(searchPoint.x) || isnan(searchPoint.y) || isnan(searchPoint.z))
    return;

  if (kdtreeLocalMap.radiusSearch(searchPoint, _sensing_range,
                                  pointIdxRadiusSearch,
                                  pointRadiusSquaredDistance) > 0) {
    for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
      pt = cloudMap.points[pointIdxRadiusSearch[i]];
      localMap.points.push_back(pt);
    }
  } else {
    ROS_ERROR("[Map server] No obstacles .");
    return;
  }

  localMap.width = localMap.points.size();
  localMap.height = 1;
  localMap.is_dense = true;

  pcl::toROSMsg(localMap, localMap_pcd);
  localMap_pcd.header.frame_id = "world";
  _local_map_pub.publish(localMap_pcd);
}

void clickCallback(const geometry_msgs::PoseStamped& msg) {
  double x = msg.pose.position.x;
  double y = msg.pose.position.y;
  double w = rand_w(eng);
  double h;
  pcl::PointXYZ pt_random;

  x = floor(x / _resolution) * _resolution + _resolution / 2.0;
  y = floor(y / _resolution) * _resolution + _resolution / 2.0;

  int widNum = ceil(w / _resolution);

  for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
    for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
      h = rand_h(eng);
      int heiNum = ceil(h / _resolution);
      for (int t = -1; t < heiNum; t++) {
        pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
        pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
        pt_random.z = (t + 0.5) * _resolution + 1e-2;
        clicked_cloud_.points.push_back(pt_random);
        cloudMap.points.push_back(pt_random);
      }
    }
  clicked_cloud_.width = clicked_cloud_.points.size();
  clicked_cloud_.height = 1;
  clicked_cloud_.is_dense = true;

  pcl::toROSMsg(clicked_cloud_, localMap_pcd);
  localMap_pcd.header.frame_id = "world";
  click_map_pub_.publish(localMap_pcd);

  cloudMap.width = cloudMap.points.size();

  return;
}
// Generate an empty corridor map (boundary walls only, no obstacles)
// 用于 swarm_nonehall.launch：只有走廊围墙，内部完全无障碍
void GenerateEmptyCorridorMap() {
  pcl::PointXYZ pt;

  // Corridor parameters: 40x10x3 meters, centered at origin
  double corridor_length = 40.0;  // X direction
  double corridor_width  = 10.0;  // Y direction
  double corridor_height = _z_size;
  double wall_thickness  = 0.3;

  // Left wall (y = -corridor_width/2)
  for (double x = -corridor_length/2; x <= corridor_length/2; x += _resolution) {
    for (double y = -corridor_width/2 - wall_thickness; y <= -corridor_width/2; y += _resolution) {
      for (double z = 0; z <= corridor_height; z += _resolution) {
        pt.x = x; pt.y = y; pt.z = z;
        cloudMap.push_back(pt);
      }
    }
  }

  // Right wall (y = corridor_width/2)
  for (double x = -corridor_length/2; x <= corridor_length/2; x += _resolution) {
    for (double y = corridor_width/2; y <= corridor_width/2 + wall_thickness; y += _resolution) {
      for (double z = 0; z <= corridor_height; z += _resolution) {
        pt.x = x; pt.y = y; pt.z = z;
        cloudMap.push_back(pt);
      }
    }
  }

  // Front wall (x = -corridor_length/2)
  for (double x = -corridor_length/2 - wall_thickness; x <= -corridor_length/2; x += _resolution) {
    for (double y = -corridor_width/2; y <= corridor_width/2; y += _resolution) {
      for (double z = 0; z <= corridor_height; z += _resolution) {
        pt.x = x; pt.y = y; pt.z = z;
        cloudMap.push_back(pt);
      }
    }
  }

  // Back wall (x = corridor_length/2)
  for (double x = corridor_length/2; x <= corridor_length/2 + wall_thickness; x += _resolution) {
    for (double y = -corridor_width/2; y <= corridor_width/2; y += _resolution) {
      for (double z = 0; z <= corridor_height; z += _resolution) {
        pt.x = x; pt.y = y; pt.z = z;
        cloudMap.push_back(pt);
      }
    }
  }

  cloudMap.width  = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  ROS_WARN("Finished generating EMPTY corridor map (40x10x%.1fm), no obstacles inside", corridor_height);

  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());
  _map_ok = true;
}

// Generate a fixed cross-wall map，用于first_testenv.launch
void GenerateFixedCrossWallMap() {
  pcl::PointXYZ pt;
  
  // Corridor parameters: 40x10x3 meters, centered at origin
  double corridor_length = 40.0;  // X direction
  double corridor_width = 10.0;   // Y direction
  double corridor_height = _z_size;
  double wall_thickness = 0.3;
  
  // Generate corridor boundary walls
  // Left wall (y = -corridor_width/2)
  for (double x = -corridor_length/2; x <= corridor_length/2; x += _resolution) {
    for (double y = -corridor_width/2 - wall_thickness; y <= -corridor_width/2; y += _resolution) {
      for (double z = 0; z <= corridor_height; z += _resolution) {
        pt.x = x;
        pt.y = y;
        pt.z = z;
        cloudMap.push_back(pt);
      }
    }
  }
  
  // Right wall (y = corridor_width/2)
  for (double x = -corridor_length/2; x <= corridor_length/2; x += _resolution) {
    for (double y = corridor_width/2; y <= corridor_width/2 + wall_thickness; y += _resolution) {
      for (double z = 0; z <= corridor_height; z += _resolution) {
        pt.x = x;
        pt.y = y;
        pt.z = z;
        cloudMap.push_back(pt);
      }
    }
  }
  
  // Front wall (x = -corridor_length/2)
  for (double x = -corridor_length/2 - wall_thickness; x <= -corridor_length/2; x += _resolution) {
    for (double y = -corridor_width/2; y <= corridor_width/2; y += _resolution) {
      for (double z = 0; z <= corridor_height; z += _resolution) {
        pt.x = x;
        pt.y = y;
        pt.z = z;
        cloudMap.push_back(pt);
      }
    }
  }
  
  // Back wall (x = corridor_length/2)
  for (double x = corridor_length/2; x <= corridor_length/2 + wall_thickness; x += _resolution) {
    for (double y = -corridor_width/2; y <= corridor_width/2; y += _resolution) {
      for (double z = 0; z <= corridor_height; z += _resolution) {
        pt.x = x;
        pt.y = y;
        pt.z = z;
        cloudMap.push_back(pt);
      }
    }
  }
  
  // ========== Door-frame obstacles for swarm_longhall ==========
  //
  // Design goal:
  //   gap_width = 1.3 m  →  single drone (body ~0.4m) can pass,
  //                          V-formation span (~2.24m) cannot pass.
  //
  // Leader waypoints: (-18,0) → (-10,3) → (0,-4) → (10,3) → (18,0)
  // Gap center Y is set to the interpolated leader Y at each gate X.
  //
  //   Gate 1  x=-12  gap_center_y= 1.5   (segment 1: y goes 0→3)
  //   Gate 2  x= -2  gap_center_y=-2.5   (segment 2: y goes 3→-4)
  //   Gate 3  x=  6  gap_center_y=-0.5   (segment 3: y goes -4→3)
  //   Gate 4  x= 14  gap_center_y= 1.5   (segment 4: y goes 3→0)

  const double gap_half      = 0.65;   // half of 1.3m gap
  const double frame_thick   = 0.40;   // wall thickness in X
  const double frame_height  = corridor_height;  // floor-to-ceiling

  // Lambda-like macro: emit one door-frame wall (two panels, one gap)
  // Because C++03/11 lambdas capturing locals work fine here:
  auto addDoorFrame = [&](double fx, double fy) {
    for (double dx = -frame_thick / 2.0; dx <= frame_thick / 2.0; dx += _resolution) {
      double x = fx + dx;
      // Left panel: y in [-corridor_width/2 , fy - gap_half]
      for (double y = -corridor_width / 2.0; y <= fy - gap_half; y += _resolution)
        for (double z = 0.0; z <= frame_height; z += _resolution) {
          pt.x = x; pt.y = y; pt.z = z;
          cloudMap.push_back(pt);
        }
      // Right panel: y in [fy + gap_half , +corridor_width/2]
      for (double y = fy + gap_half; y <= corridor_width / 2.0; y += _resolution)
        for (double z = 0.0; z <= frame_height; z += _resolution) {
          pt.x = x; pt.y = y; pt.z = z;
          cloudMap.push_back(pt);
        }
    }
  };

  // Gate 1 — x=-12, gap centered at y=+1.5
  addDoorFrame(-12.0,  1.5);
  // Gate 3 — x= +6, gap centered at y=-0.5
  addDoorFrame(  6.0, -0.5);

  // ========== Floating Ring obstacle (toroidal, passable through center) ==========
  // 圆环中心在走廊中央，主机可从中心穿过，编队需要收拢或变换
  {
    const double ring_x = -7.0;
    const double ring_y = 1.0;
    const double ring_z = 1.5;
    const double ring_major_r = 1.8;  // 大圆半径（圆环中心到管道中心的距离）
    const double ring_minor_r = 0.4;  // 管道半径（管壁厚度）
    // 圆环轴线沿 X 方向（即圆环平面是 YZ 平面），无人机沿 X 飞行可穿过中心
    for (double theta = 0.0; theta < 2.0 * M_PI; theta += 0.08) {
      // 圆环骨架点（在 YZ 平面上的大圆）
      double cy = ring_y + ring_major_r * cos(theta);
      double cz = ring_z + ring_major_r * sin(theta);
      // 围绕骨架点生成管道截面
      for (double phi = 0.0; phi < 2.0 * M_PI; phi += 0.15) {
        pt.x = ring_x + ring_minor_r * 0.5 * cos(phi);  // X 方向管壁薄一些
        pt.y = cy + ring_minor_r * cos(phi) * cos(theta);
        pt.z = cz + ring_minor_r * sin(phi);
        if (pt.z > 0.05)  // 不低于地面
          cloudMap.push_back(pt);
      }
    }
    ROS_INFO("Added floating ring at (%.1f, %.1f, %.1f), major_r=%.1f",
             ring_x, ring_y, ring_z, ring_major_r);
  }

  // ========== Z-axis narrow slit (ceiling lowered + floor raised, 1.0m vertical gap) ==========
  // 水平方向全开放（XY 平面可自由通过），但 Z 轴只留 0.8~1.8m 的缝隙
  // 迫使编队在垂直方向压扁——与门框的 Z 轴避障策略形成对比
  {
    const double slit_x = -2.0;
    const double slit_thick = 0.4;    // X 方向厚度
    const double slit_gap_low  = 0.6; // 缝隙下沿 Z
    const double slit_gap_high = 2.0; // 缝隙上沿 Z
    for (double x = slit_x - slit_thick / 2.0; x <= slit_x + slit_thick / 2.0; x += _resolution) {
      for (double y = -corridor_width / 2.0; y <= corridor_width / 2.0; y += _resolution) {
        // 底部墙体：z = 0 到 slit_gap_low
        for (double z = 0.0; z <= slit_gap_low; z += _resolution) {
          pt.x = x; pt.y = y; pt.z = z;
          cloudMap.push_back(pt);
        }
        // 顶部墙体：z = slit_gap_high 到 corridor_height
        for (double z = slit_gap_high; z <= corridor_height; z += _resolution) {
          pt.x = x; pt.y = y; pt.z = z;
          cloudMap.push_back(pt);
        }
      }
    }
    ROS_INFO("Added Z-axis narrow slit at x=%.1f, passable z=[%.1f, %.1f]",
             slit_x, slit_gap_low, slit_gap_high);
  }

  // ========== Second floating ring (larger, tilted 15 deg around Y) ==========
  {
    const double ring2_x = 14.0;
    const double ring2_y = 1.5;
    const double ring2_z = 1.5;
    const double ring2_major_r = 1.4;
    const double ring2_minor_r = 0.35;
    const double tilt = 15.0 * M_PI / 180.0;  // 绕 Y 轴倾斜 15 度
    for (double theta = 0.0; theta < 2.0 * M_PI; theta += 0.08) {
      double cy = ring2_y + ring2_major_r * cos(theta);
      double cz = ring2_z + ring2_major_r * sin(theta);
      for (double phi = 0.0; phi < 2.0 * M_PI; phi += 0.15) {
        double local_x = ring2_minor_r * cos(phi);
        double local_y = ring2_minor_r * cos(phi) * cos(theta);
        double local_z = ring2_minor_r * sin(phi);
        // 施加 Y 轴倾斜
        pt.x = ring2_x + local_x * cos(tilt) - (cz - ring2_z + local_z) * sin(tilt);
        pt.y = cy + local_y;
        pt.z = ring2_z + local_x * sin(tilt) + (cz - ring2_z + local_z) * cos(tilt);
        if (pt.z > 0.05)
          cloudMap.push_back(pt);
      }
    }
    ROS_INFO("Added tilted ring at (%.1f, %.1f, %.1f), major_r=%.1f, tilt=15deg",
             ring2_x, ring2_y, ring2_z, ring2_major_r);
  }

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  ROS_WARN("Finished generating corridor map (40x10x3m) with 4 door-frame gates + 2 rings + 1 Z-slit");

  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());
  _map_ok = true;
}


// Generate inclined corridor map (30 degrees downward slope)
// 生成30度向下倾斜的走廊地图，用于测试局部坐标系
void GenerateInclinedCorridorMap() {
  pcl::PointXYZ pt;
  
  // Corridor parameters
  double corridor_length = 40.0;  // X direction (horizontal projection)
  double corridor_width = 6.0;    // Y direction
  double corridor_local_height = 4.0;  // Height of corridor cross-section
  double wall_thickness = 0.3;
  
  // Slope parameters: 30 degrees downward
  // tan(30°) ≈ 0.577, so for every 1m horizontal, descend 0.577m
  double slope_angle = 30.0 * M_PI / 180.0;  // 30 degrees in radians
  double slope_tan = tan(slope_angle);
  
  // Starting height (top of corridor) and ending height (bottom)
  double start_z = 25.0;  // Start at z=25
  double end_z = start_z - corridor_length * slope_tan;  // ~25 - 23.1 = ~1.9
  
  ROS_INFO("Generating inclined corridor: slope=30deg, start_z=%.1f, end_z=%.1f", start_z, end_z);
  
  // Generate inclined corridor walls
  // The corridor floor follows z = start_z - slope_tan * (x + corridor_length/2)
  
  double step = 0.2;  // Coarser resolution for performance
  
  // ========== Floor (bottom surface of inclined corridor) ==========
  for (double x = -corridor_length/2; x <= corridor_length/2; x += step) {
    double floor_z = start_z - slope_tan * (x + corridor_length/2);
    for (double y = -corridor_width/2; y <= corridor_width/2; y += step) {
      for (double dz = -wall_thickness; dz <= 0; dz += _resolution) {
        pt.x = x;
        pt.y = y;
        pt.z = floor_z + dz;
        cloudMap.push_back(pt);
      }
    }
  }
  
  // ========== Ceiling (top surface of inclined corridor) ==========
  for (double x = -corridor_length/2; x <= corridor_length/2; x += step) {
    double floor_z = start_z - slope_tan * (x + corridor_length/2);
    double ceil_z = floor_z + corridor_local_height;
    for (double y = -corridor_width/2; y <= corridor_width/2; y += step) {
      for (double dz = 0; dz <= wall_thickness; dz += _resolution) {
        pt.x = x;
        pt.y = y;
        pt.z = ceil_z + dz;
        cloudMap.push_back(pt);
      }
    }
  }
  
  // ========== Left wall (y = -corridor_width/2) ==========
  for (double x = -corridor_length/2; x <= corridor_length/2; x += step) {
    double floor_z = start_z - slope_tan * (x + corridor_length/2);
    for (double dy = -wall_thickness; dy <= 0; dy += _resolution) {
      for (double z = floor_z; z <= floor_z + corridor_local_height; z += step) {
        pt.x = x;
        pt.y = -corridor_width/2 + dy;
        pt.z = z;
        cloudMap.push_back(pt);
      }
    }
  }
  
  // ========== Right wall (y = corridor_width/2) ==========
  for (double x = -corridor_length/2; x <= corridor_length/2; x += step) {
    double floor_z = start_z - slope_tan * (x + corridor_length/2);
    for (double dy = 0; dy <= wall_thickness; dy += _resolution) {
      for (double z = floor_z; z <= floor_z + corridor_local_height; z += step) {
        pt.x = x;
        pt.y = corridor_width/2 + dy;
        pt.z = z;
        cloudMap.push_back(pt);
      }
    }
  }
  
  // ========== Front wall (entrance at x = -corridor_length/2, top of slope) ==========
  double front_floor_z = start_z;
  for (double dx = -wall_thickness; dx <= 0; dx += _resolution) {
    for (double y = -corridor_width/2; y <= corridor_width/2; y += step) {
      for (double z = front_floor_z; z <= front_floor_z + corridor_local_height; z += step) {
        pt.x = -corridor_length/2 + dx;
        pt.y = y;
        pt.z = z;
        cloudMap.push_back(pt);
      }
    }
  }
  
  // ========== Back wall (exit at x = corridor_length/2, bottom of slope) ==========
  double back_floor_z = end_z;
  for (double dx = 0; dx <= wall_thickness; dx += _resolution) {
    for (double y = -corridor_width/2; y <= corridor_width/2; y += step) {
      for (double z = back_floor_z; z <= back_floor_z + corridor_local_height; z += step) {
        pt.x = corridor_length/2 + dx;
        pt.y = y;
        pt.z = z;
        cloudMap.push_back(pt);
      }
    }
  }
  
  // ========== Obstacles along the inclined path ==========
  
  // Obstacle 1: Cylinder near start (x=-14)
  double obs1_x = -14.0;
  double obs1_y = 1.5;
  double obs1_floor_z = start_z - slope_tan * (obs1_x + corridor_length/2);
  double obs1_radius = 0.8;
  double obs1_height = 2.5;
  for (double x = obs1_x - obs1_radius; x <= obs1_x + obs1_radius; x += _resolution) {
    for (double y = obs1_y - obs1_radius; y <= obs1_y + obs1_radius; y += _resolution) {
      if (sqrt(pow(x - obs1_x, 2) + pow(y - obs1_y, 2)) <= obs1_radius) {
        for (double z = obs1_floor_z; z <= obs1_floor_z + obs1_height; z += _resolution) {
          pt.x = x;
          pt.y = y;
          pt.z = z;
          cloudMap.push_back(pt);
        }
      }
    }
  }
  
  // Obstacle 2: Box at x=-6
  double obs2_x = -6.0;
  double obs2_y = -1.5;
  double obs2_floor_z = start_z - slope_tan * (obs2_x + corridor_length/2);
  double obs2_size = 1.2;
  double obs2_height = 2.0;
  for (double x = obs2_x - obs2_size/2; x <= obs2_x + obs2_size/2; x += _resolution) {
    for (double y = obs2_y - obs2_size/2; y <= obs2_y + obs2_size/2; y += _resolution) {
      for (double z = obs2_floor_z; z <= obs2_floor_z + obs2_height; z += _resolution) {
        pt.x = x;
        pt.y = y;
        pt.z = z;
        cloudMap.push_back(pt);
      }
    }
  }
  
  // Obstacle 3: Suspended ring/torus at x=2 (middle section)
  double obs3_x = 2.0;
  double obs3_y = 0.0;
  double obs3_floor_z = start_z - slope_tan * (obs3_x + corridor_length/2);
  double obs3_z_center = obs3_floor_z + 2.0;
  double obs3_major_radius = 1.5;  // Ring major radius
  double obs3_minor_radius = 0.3;  // Ring tube radius
  for (double theta = 0; theta < 2*M_PI; theta += 0.15) {
    double ring_x = obs3_x + obs3_major_radius * cos(theta);
    double ring_y = obs3_y + obs3_major_radius * sin(theta);
    for (double phi = 0; phi < 2*M_PI; phi += 0.3) {
      pt.x = ring_x + obs3_minor_radius * cos(phi) * cos(theta);
      pt.y = ring_y + obs3_minor_radius * cos(phi) * sin(theta);
      pt.z = obs3_z_center + obs3_minor_radius * sin(phi);
      cloudMap.push_back(pt);
    }
  }
  
  // Obstacle 4: Cylinder at x=10
  double obs4_x = 14.0;
  double obs4_y = 1.0;
  double obs4_floor_z = start_z - slope_tan * (obs4_x + corridor_length/2);
  double obs4_radius = 0.6;
  double obs4_height = 2.8;
  for (double x = obs4_x - obs4_radius; x <= obs4_x + obs4_radius; x += _resolution) {
    for (double y = obs4_y - obs4_radius; y <= obs4_y + obs4_radius; y += _resolution) {
      if (sqrt(pow(x - obs4_x, 2) + pow(y - obs4_y, 2)) <= obs4_radius) {
        for (double z = obs4_floor_z; z <= obs4_floor_z + obs4_height; z += _resolution) {
          pt.x = x;
          pt.y = y;
          pt.z = z;
          cloudMap.push_back(pt);
        }
      }
    }
  }
  
  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;
  
  ROS_WARN("Finished generating INCLINED corridor map (40m long, 30deg slope, z: %.1f to %.1f)", start_z, end_z);
  
  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());
  _map_ok = true;
}
// Generate fixed cross-wall map with additional obstacles (cylinders and cubes)
void GenerateFixedObstacleMap() {
  pcl::PointXYZ pt;
  
  // Map parameters: 20x20 meters, centered at origin
  double map_size = 20.0;
  double wall_thickness = 0.3;
  double wall_height = _z_size;  // Use full map height to prevent flying over walls
  double door_width = 2.0;  // Width of each door opening
  
  // ========== Generate all walls (same as GenerateFixedCrossWallMap) ==========
  
  // Generate horizontal wall (along X-axis at y=0) with 2 doors
  for (double x = -map_size/2; x <= map_size/2; x += _resolution) {
    for (double y = -wall_thickness/2; y <= wall_thickness/2; y += _resolution) {
      for (double z = 0.0; z <= wall_height; z += _resolution) {
        // Two doors: one at x=-6, another at x=+6
        if (fabs(x+6) > door_width/2 && fabs(x-6) > door_width/2) {
          pt.x = x;
          pt.y = y;
          pt.z = z;
          cloudMap.points.push_back(pt);
        }
      }
    }
  }
  
  // Generate vertical wall (along Y-axis at x=0) with 2 doors
  for (double y = -map_size/2; y <= map_size/2; y += _resolution) {
    for (double x = -wall_thickness/2; x <= wall_thickness/2; x += _resolution) {
      for (double z = 0.0; z <= wall_height; z += _resolution) {
        // Two doors: one at y=-5, another at y=+5
        if (fabs(y+5) > door_width/2 && fabs(y-5) > door_width/2) {
          pt.x = x;
          pt.y = y;
          pt.z = z;
          cloudMap.points.push_back(pt);
        }
      }
    }
  }
  
  // Bottom wall (y = -map_size/2)
  for (double x = -map_size/2; x <= map_size/2; x += _resolution) {
    for (double y = -map_size/2; y <= -map_size/2 + wall_thickness; y += _resolution) {
      for (double z = 0.0; z <= wall_height; z += _resolution) {
        pt.x = x;
        pt.y = y;
        pt.z = z;
        cloudMap.points.push_back(pt);
      }
    }
  }
  
  // Top wall (y = map_size/2)
  for (double x = -map_size/2; x <= map_size/2; x += _resolution) {
    for (double y = map_size/2 - wall_thickness; y <= map_size/2; y += _resolution) {
      for (double z = 0.0; z <= wall_height; z += _resolution) {
        pt.x = x;
        pt.y = y;
        pt.z = z;
        cloudMap.points.push_back(pt);
      }
    }
  }
  
  // Left wall (x = -map_size/2)
  for (double y = -map_size/2; y <= map_size/2; y += _resolution) {
    for (double x = -map_size/2; x <= -map_size/2 + wall_thickness; x += _resolution) {
      for (double z = 0.0; z <= wall_height; z += _resolution) {
        pt.x = x;
        pt.y = y;
        pt.z = z;
        cloudMap.points.push_back(pt);
      }
    }
  }
  
  // Right wall (x = map_size/2)
  for (double y = -map_size/2; y <= map_size/2; y += _resolution) {
    for (double x = map_size/2 - wall_thickness; x <= map_size/2; x += _resolution) {
      for (double z = 0.0; z <= wall_height; z += _resolution) {
        pt.x = x;
        pt.y = y;
        pt.z = z;
        cloudMap.points.push_back(pt);
      }
    }
  }
  
  // ========== Generate additional obstacles ==========
  
  // Obstacle 1: Cylinder in bottom-right quadrant
  double cyl1_x = 5.0;
  double cyl1_y = -7.0;
  double cyl1_radius = 0.6;
  for (double x = cyl1_x - cyl1_radius; x <= cyl1_x + cyl1_radius; x += _resolution) {
    for (double y = cyl1_y - cyl1_radius; y <= cyl1_y + cyl1_radius; y += _resolution) {
      double dist = sqrt(pow(x - cyl1_x, 2) + pow(y - cyl1_y, 2));
      if (dist <= cyl1_radius) {
        for (double z = 0.0; z <= wall_height; z += _resolution) {
          pt.x = x;
          pt.y = y;
          pt.z = z;
          cloudMap.points.push_back(pt);
        }
      }
    }
  }
  
  // Obstacle 2: Cube in bottom-left quadrant
  double cube1_x = -4.0;
  double cube1_y = -6.5;
  double cube1_size = 1.0;
  for (double x = cube1_x - cube1_size/2; x <= cube1_x + cube1_size/2; x += _resolution) {
    for (double y = cube1_y - cube1_size/2; y <= cube1_y + cube1_size/2; y += _resolution) {
      for (double z = 0.0; z <= wall_height; z += _resolution) {
        pt.x = x;
        pt.y = y;
        pt.z = z;
        cloudMap.points.push_back(pt);
      }
    }
  }
  
  // Obstacle 3: Cylinder in top-left quadrant
  double cyl2_x = -6.5;
  double cyl2_y = 4.5;
  double cyl2_radius = 0.7;
  for (double x = cyl2_x - cyl2_radius; x <= cyl2_x + cyl2_radius; x += _resolution) {
    for (double y = cyl2_y - cyl2_radius; y <= cyl2_y + cyl2_radius; y += _resolution) {
      double dist = sqrt(pow(x - cyl2_x, 2) + pow(y - cyl2_y, 2));
      if (dist <= cyl2_radius) {
        for (double z = 0.0; z <= wall_height; z += _resolution) {
          pt.x = x;
          pt.y = y;
          pt.z = z;
          cloudMap.points.push_back(pt);
        }
      }
    }
  }
  
  // Obstacle 4: Cube in top-right quadrant
  double cube2_x = 3.5;
  double cube2_y = 6.5;
  double cube2_size = 1.2;
  for (double x = cube2_x - cube2_size/2; x <= cube2_x + cube2_size/2; x += _resolution) {
    for (double y = cube2_y - cube2_size/2; y <= cube2_y + cube2_size/2; y += _resolution) {
      for (double z = 0.0; z <= wall_height; z += _resolution) {
        pt.x = x;
        pt.y = y;
        pt.z = z;
        cloudMap.points.push_back(pt);
      }
    }
  }
  
  // Obstacle 5: big cylinder near center (in bottom-right of center cross)
  double cyl3_x = 5.5;
  double cyl3_y = -2.0;
  double cyl3_radius = 0.8;
  for (double x = cyl3_x - cyl3_radius; x <= cyl3_x + cyl3_radius; x += _resolution) {
    for (double y = cyl3_y - cyl3_radius; y <= cyl3_y + cyl3_radius; y += _resolution) {
      double dist = sqrt(pow(x - cyl3_x, 2) + pow(y - cyl3_y, 2));
      if (dist <= cyl3_radius) {
        for (double z = 0.0; z <= wall_height; z += _resolution) {
          pt.x = x;
          pt.y = y;
          pt.z = z;
          cloudMap.points.push_back(pt);
        }
      }
    }
  }
  
  // Obstacle 6: Cube in top-right quadrant
  double cube3_x = 6.5;
  double cube3_y = 3.5;
  double cube3_size = 1.2;
  for (double x = cube3_x - cube3_size/2; x <= cube3_x + cube3_size/2; x += _resolution) {
    for (double y = cube3_y - cube3_size/2; y <= cube3_y + cube3_size/2; y += _resolution) {
      for (double z = 0.0; z <= wall_height; z += _resolution) {
        pt.x = x;
        pt.y = y;
        pt.z = z;
        cloudMap.points.push_back(pt);
      }
    }
  }

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;
  
  ROS_WARN("Finished generating fixed cross-wall map with obstacles (2 cylinders + 2 cubes in quadrants, 1 small cylinder near center)");
  
  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());
  _map_ok = true;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "random_map_sensing");
  ros::NodeHandle n("~");

  _local_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/local_cloud", 1);
  _all_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);

  _odom_sub = n.subscribe("odometry", 50, rcvOdometryCallbck);

  click_map_pub_ =
      n.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/local_map", 1);
  // ros::Subscriber click_sub = n.subscribe("/goal", 10, clickCallback);

  n.param("init_state_x", _init_x, 0.0);
  n.param("init_state_y", _init_y, 0.0);

  n.param("map/x_size", _x_size, 50.0);
  n.param("map/y_size", _y_size, 50.0);
  n.param("map/z_size", _z_size, 5.0);
  n.param("map/obs_num", _obs_num, 30);
  n.param("map/resolution", _resolution, 0.1);
  n.param("map/circle_num", circle_num_, 30);

  n.param("ObstacleShape/lower_rad", _w_l, 0.3);
  n.param("ObstacleShape/upper_rad", _w_h, 0.8);
  n.param("ObstacleShape/lower_hei", _h_l, 3.0);
  n.param("ObstacleShape/upper_hei", _h_h, 7.0);

  n.param("ObstacleShape/radius_l", radius_l_, 7.0);
  n.param("ObstacleShape/radius_h", radius_h_, 7.0);
  n.param("ObstacleShape/z_l", z_l_, 7.0);
  n.param("ObstacleShape/z_h", z_h_, 7.0);
  n.param("ObstacleShape/theta", theta_, 7.0);

  n.param("sensing/radius", _sensing_range, 10.0);
  n.param("sensing/radius", _sense_rate, 10.0);

  n.param("min_distance", _min_dist, 1.0);

  std::string map_type;
  n.param<std::string>("map/type", map_type, "random");

  _x_l = -_x_size / 2.0;
  _x_h = +_x_size / 2.0;

  _y_l = -_y_size / 2.0;
  _y_h = +_y_size / 2.0;

  _obs_num = min(_obs_num, (int)_x_size * 10);
  _z_limit = _z_size;

  ros::Duration(0.5).sleep();

  // Select map generation based on parameter
  if (map_type == "fixed_cross") {
    GenerateFixedObstacleMap();
  } else if (map_type == "corridor") {
    GenerateFixedCrossWallMap();  // Now generates corridor map
  } else if (map_type == "corridor_empty") {
    GenerateEmptyCorridorMap();   // Empty corridor, no obstacles
  } else if (map_type == "inclined_corridor") {
    GenerateInclinedCorridorMap();  // 30-degree inclined corridor
  } else {
    // RandomMapGenerate();
    RandomMapGenerateCylinder();
  }

  ros::Rate loop_rate(_sense_rate);

  while (ros::ok()) {
    pubSensedPoints();
    ros::spinOnce();
    loop_rate.sleep();
  }
}