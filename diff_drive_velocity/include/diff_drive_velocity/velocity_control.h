#ifndef VELOCITY_CONTROL_H
#define VELOCITY_CONTROL_H

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <diff_drive_velocity/state.h>
#include <visualization_msgs/Marker.h>

#include <math.h>
#include <thread>


class VelocityControl
{
public:
  VelocityControl(ros::NodeHandle *nh);
  void Callback_UnicycleModel(const geometry_msgs::Twist msg);
  void ComputeOdometry(diff_drive_velocity::state* robot_state, double dt);

  void PublishOdometry(const diff_drive_velocity::state robot_state, const ros::Time current_time);
  void VisualizeOdometry(const diff_drive_velocity::state robot_state, const ros::Time current_time);

private:
  ros::Publisher joints_controller_pub;
  ros::Subscriber joystick_sub;

  ros::Publisher odometry_pub;
  ros::Publisher marker_pub;

  std_msgs::Float64MultiArray joints_command;
  ros::Time current_time;
  ros::Time last_time;


  diff_drive_velocity::state robot_state;

  struct RobotVelocity
  {
    double translation = 0;
    double angular = 0;
  };
  RobotVelocity robot_velocity;

  int marker_id = 0;

};


#endif // VELOCITY_CONTROL_H
