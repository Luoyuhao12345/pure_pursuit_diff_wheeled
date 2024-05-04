#include "my_pure_pursuit.h"

MyPurePursuit::MyPurePursuit()
{
    ROS_INFO("car_start");

    nh.param("/pure_pursuit/param_k", param_k, float(0.0));
    nh.param("/pure_pursuit/param_lfc", param_lfc, float(0.0));
    nh.param("/pure_pursuit/max_lfc", max_lfc, float(0.0));
    nh.param("/pure_pursuit/param_l", param_l, float(0.0));
    nh.param("/pure_pursuit/max_vel", max_vel, float(0.0));


    // nh.param("/pure_pursuit/param_k", param_k, float(1));
    // nh.param("/pure_pursuit/param_lfc", param_lfc, float(0.2));
    // nh.param("/pure_pursuit/max_lfc", max_lfc, float(0.7));
    // nh.param("/pure_pursuit/param_l", param_l, float(0.5));
    // nh.param("/pure_pursuit/max_vel", max_vel, float(1.2));


    ROS_INFO("a:%.2f, b:%.2f, c:%.2f", param_k, max_lfc, param_l);
    ROS_INFO("d:%.2f", max_vel);

    path_update_flag = false;
    start_car_flag = false;
    car_steer = 0;
    car_vel = 0;

    timer1 = nh.createTimer(ros::Duration(0.05), &MyPurePursuit::info_get_cb, this);
    timer2 = nh.createTimer(ros::Duration(0.05), &MyPurePursuit::car_ctl_cb, this);

    car_pose_sub = nh.subscribe("/odom", 5, &MyPurePursuit::odom_cb, this);
    goal_sub = nh.subscribe("/move_base_simple/goal", 1, &MyPurePursuit::goal_cb, this);
    global_path_sub = nh.subscribe("/my_global_planner", 1, &MyPurePursuit::global_path_cb, this);

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("arrow_marker", 10);
}

// 获得了小车的x,y和yaw角
void MyPurePursuit::odom_cb(const nav_msgs::Odometry::ConstPtr& pose)
{
    nav_msgs::Odometry _odom_pose = *pose;
    geometry_msgs::PoseStamped odom_pose;
    geometry_msgs::PoseStamped target_pose;

    odom_pose.header = _odom_pose.header;
    odom_pose.header.stamp = ros::Time();
    odom_pose.pose.position = _odom_pose.pose.pose.position;
    odom_pose.pose.orientation = _odom_pose.pose.pose.orientation;

    tf_listener.waitForTransform("/map", "/odom", ros::Time(0), ros::Duration(0.5)); 
    try
    {
        tf_listener.transformPose("/map", odom_pose, target_pose);
        car_x = target_pose.pose.position.x;
        car_y = target_pose.pose.position.y;

        tf::Quaternion quat;
        double roll, pitch, yaw;
        tf::quaternionMsgToTF(target_pose.pose.orientation, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        car_yaw = yaw;

        // ROS_INFO("car_x:%.2f, car_y:%.2f, car_yaw:%.2f", car_x, car_y, car_yaw);
    }
    catch( tf::TransformException ex)
    {
        ROS_WARN("transfrom exception : %s",ex.what());
        return;
    }
}

void MyPurePursuit::goal_cb(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    geometry_msgs::PoseStamped goal_point;
    goal_point = *goalMsg;
    goal_x = goal_point.pose.position.x;
    goal_y = goal_point.pose.position.y;
    // ROS_INFO("goal_x= %.2f  goal_y=%.2f", goal_x, goal_y);
}

// 确保在找前向点时路径不会被更新
void MyPurePursuit::global_path_cb(const nav_msgs::Path::ConstPtr& pathMsg)
{
    if(path_update_flag)
        target_path = *pathMsg;
    path_update_flag = false;
}

int closest_index;
float closest_point_x, closest_point_y;
void MyPurePursuit::find_closest_point()
{
    float min_dis = 999999;
    float current_dis;
    float dx, dy;
    closest_index = 0;
    int num = target_path.poses.size();
    if(num != 0)
    {
        for(int i = 0; i < num; i++)
        {
            dx = target_path.poses[i].pose.position.x - car_x;
            dy = target_path.poses[i].pose.position.y - car_y;
            current_dis = sqrt(dx*dx+dy*dy);
            if(current_dis < min_dis) 
            {
                min_dis = current_dis;
                closest_index = i;
            }
        }
        closest_point_x = target_path.poses[closest_index].pose.position.x;
        closest_point_y = target_path.poses[closest_index].pose.position.y;
        // ROS_INFO("closest_point_x= %.2f  closest_point_y=%.2f", closest_point_x, closest_point_y);
    }
}

void MyPurePursuit::car_lfc_set()
{
    param_lfc = param_k*car_vel + 0.3;
    if(param_lfc>max_lfc) param_lfc = max_lfc;
    ROS_INFO("%.2f", param_lfc);
}

float forward_point_x, forward_point_y;
bool brake_flag = false;
void MyPurePursuit::find_forward_point()
{
    find_closest_point();
    car_lfc_set();
    float dx, dy;
    int num = target_path.poses.size();
    if(num != 0)
    {
        float current_dis;
        for(int i = closest_index; i < num; i++)
        {
            dx = target_path.poses[i].pose.position.x - target_path.poses[closest_index].pose.position.x;
            dy = target_path.poses[i].pose.position.y - target_path.poses[closest_index].pose.position.y;
            current_dis = sqrt(dx*dx+dy*dy);
            if(current_dis > param_lfc)
            {
                forward_point_x = target_path.poses[i].pose.position.x;
                forward_point_y = target_path.poses[i].pose.position.y;
                break;
            }
            else
            {
                forward_point_x = target_path.poses[i].pose.position.x;
                forward_point_y = target_path.poses[i].pose.position.y;
            }
            // 找不到前瞻就减速
            if(i == num-1) brake_flag = true;
            else brake_flag = false;
        }
        // ROS_INFO("forward_point_x= %.2f  forward_point_y=%.2f", forward_point_x, forward_point_y);
    }
}

float pi = 3.1415;
void MyPurePursuit::pure_pursuit_control()
{
    find_forward_point();
    float alpha = atan2(forward_point_y-car_y, forward_point_x-car_x) - car_yaw;
    normalize_angle(alpha);
    float param_lf = param_lfc;
    float kappa = 2*alpha/param_lf;
    car_steer = kappa*param_k;
    // if(car_steer>pi/4) car_steer=pi/4;
    // if(car_steer<-pi/4) car_steer =-pi/4;
    // ROS_INFO("car_steer:%.2f, alpha:%.2f, car_yaw:%.2f", car_steer/3.14*180, alpha/3.14*180, car_yaw/3.14*180);
}

void MyPurePursuit::normalize_angle(float& angle)
{
    while(angle > pi) angle -= 2*pi;
    while(angle < -pi) angle += 2*pi;
}

void MyPurePursuit::car_vel_control()
{
    // car_vel = cos(car_steer)*max_vel;
    car_vel = max_vel;
    // ROS_INFO("car_steer:%.2f, cos:%.2f", car_steer, cos(car_steer));
    if(car_vel < 0.1) car_vel = 0.1;
    if(brake_flag) car_vel = car_vel/2;
    ROS_INFO("vel:%.2f", car_vel);
}

void MyPurePursuit::start_car()
{
    float stop_buff = 0.2;
    int num = target_path.poses.size();
    float dx, dy;
    dx = goal_x - car_x;
    dy = goal_y - car_y;
    float goal_dis = sqrt(dx*dx+dy*dy);
    if(num==0)
        start_car_flag = false;
    else
    {
        if(goal_dis<stop_buff)
            start_car_flag = false;
        else
            start_car_flag = true;
    }     
}

void MyPurePursuit::info_get_cb(const ros::TimerEvent&)
{
    start_car();
    car_vel_control();
    pure_pursuit_control();
    visualization();
    path_update_flag = true;
}

void MyPurePursuit::car_ctl_cb(const ros::TimerEvent&)
{
    geometry_msgs::Twist cmd_vel;

    if(start_car_flag) 
    {
        cmd_vel.linear.x = car_vel;
        cmd_vel.angular.z = car_steer;
    }
    else
    {
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
    }

    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel_pub.publish(cmd_vel);
}

void MyPurePursuit::visualization()
{
    // 创建一个 Marker 消息
    visualization_msgs::Marker arrow_marker;
    arrow_marker.header.frame_id = "map";
    arrow_marker.header.stamp = ros::Time::now();
    arrow_marker.action = visualization_msgs::Marker::ADD;
    arrow_marker.type = visualization_msgs::Marker::ARROW;
    arrow_marker.scale.x = 0.1;
    arrow_marker.scale.y = 0.2;
    arrow_marker.scale.z = 0.0;
    arrow_marker.color.a = 1.0;  // 不透明
    arrow_marker.color.b = 1.0;  // 蓝色

    // 设置箭头的起点和终点坐标
    geometry_msgs::Point start_point;
    start_point.x = car_x;
    start_point.y = car_y;
    start_point.z = 0.0;

    geometry_msgs::Point end_point;
    end_point.x = forward_point_x;
    end_point.y = forward_point_y;
    end_point.z = 0.0;

    arrow_marker.points.push_back(start_point);
    arrow_marker.points.push_back(end_point);

    arrow_marker.header.stamp = ros::Time::now();
    marker_pub.publish(arrow_marker);
}