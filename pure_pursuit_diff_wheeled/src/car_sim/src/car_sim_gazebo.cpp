#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include"tf2/LinearMath/Quaternion.h"
#include <tf/transform_broadcaster.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Quaternion.h>

#define pi 3.14159
using namespace std;


float dt = 0.05;

void odom_pub_cb(const ros::TimerEvent& event)
{
    // 发布坐标变换
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0, 0, 0) );
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/odom"));
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "car_sim");
    ros::NodeHandle nh;
    ros::Timer timer = nh.createTimer(ros::Duration(dt), odom_pub_cb);

    ros::spin();
    return 0;
}

