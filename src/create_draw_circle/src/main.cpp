#include <cmath>
#include <sstream>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

geometry_msgs::Pose goal;
geometry_msgs::Pose current_pose;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "draw_circle");

  ros::NodeHandle n;
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  ros::Rate loop_rate(10);
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 1;
  cmd_vel.angular.z = 0.5;

  while (ros::ok())
  {
    cmd_vel_pub.publish(cmd_vel);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
