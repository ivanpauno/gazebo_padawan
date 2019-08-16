#include <cmath>
#include <sstream>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

ros::Publisher cmd_vel_pub;
geometry_msgs::Pose goal;
geometry_msgs::Pose current_pose;

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    const double k = 5;
    current_pose = msg->pose.pose;
}

geometry_msgs::Twist update_cmd_vel(geometry_msgs::Pose goal, geometry_msgs::Pose current_pose)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 5 * (goal.position.x - current_pose.position.x);
    cmd_vel.linear.y = 5 * (goal.position.y - current_pose.position.y);
    return cmd_vel;
}

double get_distance(geometry_msgs::Pose lhs, geometry_msgs::Pose rhs)
{
    return std::sqrt((lhs.position.x-rhs.position.x)*(lhs.position.x-rhs.position.x)+(lhs.position.y-rhs.position.y)*(lhs.position.y-rhs.position.y));
}

geometry_msgs::Pose get_point_circle(double theta, double radius)
{
    geometry_msgs::Pose pose;
    pose.position.x = radius * cos(theta);
    pose.position.y = radius * sin(theta);
    return pose;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "draw_circle");

  ros::NodeHandle n;

  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber pose_sub = n.subscribe<nav_msgs::Odometry>("odom", 1000, poseCallback);

  ros::Rate loop_rate(10);

  int count = 0;
  double error = 0.01;
  double radius = 5.0;
  double theta = 0;
  double step = 0.01;

  while (ros::ok())
  {
    double distance = get_distance(goal, current_pose);
    if (distance < error) {
        theta = std::fmod(theta + step, 3.14);
        goal = get_point_circle(theta, radius);
    }
    geometry_msgs::Twist cmd_vel = update_cmd_vel(goal, current_pose);
    cmd_vel_pub.publish(cmd_vel);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
