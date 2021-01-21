#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

double goal;
geometry_msgs::Pose goal_pose;
geometry_msgs::Pose robot_state;

void GoalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  goal_pose.position.x = msg->pose.position.x;
  goal_pose.position.y = msg->pose.position.y;
  goal = atan((msg->pose.position.y - robot_state.position.y)/
              (msg->pose.position.x - robot_state.position.x));
}

void RobotStatusCallback(const nav_msgs::OdometryConstPtr& msg)
{
  tf2::Quaternion q(
          msg->pose.pose.orientation.x,
          msg->pose.pose.orientation.y,
          msg->pose.pose.orientation.z,
          msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  robot_state.orientation.x = yaw;
  robot_state.position.x = msg->pose.pose.position.x;
  robot_state.position.y = msg->pose.pose.position.y;
  ROS_INFO("som v cieli \t robot: %f 't goal %f", robot_state.position.x, msg->pose.pose.position.x);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pid_go_to_goal");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(3);
  spinner.start();

  ros::Publisher pid_controller = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, GoalCallback);
  ros::Subscriber robot_state_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 1, RobotStatusCallback);

  ros::Publisher error_pub = nh.advertise<std_msgs::Float32>("/error", 1);
//  ros::Time current_time = ros::Time::now();;
//  ros::Time last_time = ros::Time::now();;

  geometry_msgs::Twist ref_speed;
  std_msgs::Float32 error_msg;
  ref_speed.linear.x = 0.2; //constant velocity

  // Controller Gains
  float Kp = 5;
  float Ki = 0;
  float Kd = 0;

  ros::Rate rate(100);
  while(ros::ok())
  {
    auto e = goal - robot_state.orientation.x;
    auto e_norm = atan2(sin(e), cos(e));
    ref_speed.angular.z = Kp * e_norm;

    error_msg.data = e_norm;
    error_pub.publish(error_msg);

    pid_controller.publish(ref_speed);
    if ( (abs(robot_state.position.x - goal_pose.position.x) <= 0.05) &&
         (abs(robot_state.position.y - goal_pose.position.y) <= 0.05) )
    {
      ref_speed.linear.x = 0;
//      ROS_INFO("som v cieli \t robot: %f 't goal %f", robot_state.position.x, goal_pose.position.x);
    }

//    last_time = current_time;
    rate.sleep();
  }
  ros::waitForShutdown();

  return 0;
}
