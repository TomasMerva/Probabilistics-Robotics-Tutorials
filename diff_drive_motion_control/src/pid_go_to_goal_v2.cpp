#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

class PID_Controller
{
public:
  PID_Controller(ros::NodeHandle *nh)
  {
    pid_controller_pub = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Publisher error_pub = nh->advertise<std_msgs::Float32>("/error", 1);

    ref_speed.linear.x = 0.2;
  }

  // Callback for goal pose
  void GoalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    goal_pose.position.x = msg->pose.position.x;
    goal_pose.position.y = msg->pose.position.y;
    goal_pose.orientation.x = atan( (goal_pose.position.y - robot_state.position.y) /
                                    (goal_pose.position.x - robot_state.position.x) );
  }

  // Callback for robot state's pose
  void RobotStateCallback(const nav_msgs::OdometryConstPtr& msg)
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
  }

  // Control loop function
  void ControlLoop()
  {
    if ( (abs(robot_state.position.x - goal_pose.position.x) > 0.05) &&
         (abs(robot_state.position.y - goal_pose.position.y) > 0.05) )
    {
      ref_speed.linear.x = 0.2;

      auto e = goal_pose.orientation.x - robot_state.orientation.x;
      auto e_norm = atan2(sin(e), cos(e));
      ref_speed.angular.z = Kp * e_norm;
      pid_controller_pub.publish(ref_speed);
    }
    else {
      ref_speed.linear.x = 0;
      pid_controller_pub.publish(ref_speed);
    }


//    error_msg.data = e_norm;
//    error_pub.publish(error_msg);
  }

private:
  ros::Publisher pid_controller_pub;
  ros::Publisher error_pub;

  geometry_msgs::Pose goal_pose;
  geometry_msgs::Pose robot_state;
  geometry_msgs::Twist ref_speed;

  std_msgs::Float32 error_msg;
  // Controller Gains
  double Kp = 5;
  double Ki = 0;
  double Kd = 0;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pid_go_to_goal_v2");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(3);
  spinner.start();

  PID_Controller controller(&nh);

  ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &PID_Controller::GoalCallback, &controller);
  ros::Subscriber robot_state_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 1, &PID_Controller::RobotStateCallback, &controller);

  ros::Rate rate(200);
  while(ros::ok())
  {
    controller.ControlLoop();
    rate.sleep();
  }

  ros::waitForShutdown();
  return 0;
}
