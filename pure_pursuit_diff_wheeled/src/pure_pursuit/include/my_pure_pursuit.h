#ifndef MY_PURE_PURSUIT_H
#define MY_PURE_PURSUIT_H

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

class MyPurePursuit
{
public:
    MyPurePursuit();

private:
    // 回调函数
    void info_get_cb(const ros::TimerEvent&);
    void car_ctl_cb(const ros::TimerEvent&);
    void odom_cb(const nav_msgs::Odometry::ConstPtr& pose);
    void goal_cb(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
    void global_path_cb(const nav_msgs::Path::ConstPtr& pathMsg);
    // 功能函数
    void find_closest_point();
    void find_forward_point();
    void pure_pursuit_control();
    void car_vel_control();
    void car_lfc_set();
    void start_car();
    void visualization();
    void normalize_angle(float& angle);

private:
    // ROS句柄
    ros::NodeHandle nh;
    // 内部变量
    float goal_x, goal_y;
    float car_x, car_y, car_yaw;
    float car_steer, car_vel, max_vel;
    bool path_update_flag;
    bool start_car_flag;
    nav_msgs::Path target_path;
    // 订阅话题
    ros::Subscriber goal_sub;
    ros::Subscriber global_path_sub;
    ros::Subscriber car_pose_sub;
    // 发布话题
    ros::Publisher cmd_vel_pub;
    ros::Publisher marker_pub;

    ros::Timer timer1, timer2;
    tf::TransformListener tf_listener;

    // 定义车体参数
    float param_k;
    float param_lfc;
    float max_lfc;
    float param_l;
};

#endif