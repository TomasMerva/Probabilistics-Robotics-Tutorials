#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <thread>
#include <math.h>

struct state_struct
{
  double x;
  double y;
  double theta;
};

class VelocityControl
{
public:
  VelocityControl(ros::NodeHandle *nh) : odometry_thread(&VelocityControl::ComputeOdometry, this)
  {
    command_pub = nh->advertise<std_msgs::Float64MultiArray>("/joints_controller/command",1);
    sub_joystick = nh->subscribe("/cmd_vel", 1, &VelocityControl::Callback, this);


  }
  void Callback(const geometry_msgs::Twist msg){
    double velocity_right = (2*msg.linear.x+msg.angular.z*0.4)/(2*0.075);
    double velocity_left = (2*msg.linear.x-msg.angular.z*0.4)/(2*0.075);
    command.data.push_back(velocity_right);
    command.data.push_back(velocity_left);
    command_pub.publish(command);
    command.data.clear();
  }

  void ComputeOdometry()
  {
    double freq = 50;
    double per = 1/freq;
    ros::Rate rate(freq);
    while(ros::ok())
    {
      auto div = velocity_translational/velocity_angular;
      state.x = state.x + ( -div*sin(state.theta) + div*sin(state.theta + velocity_angular*per) );
      state.y = state.y + ( div*cos(state.theta) - div*cos(state.theta + velocity_angular*per) );
      state.theta = state.theta + ( velocity_angular*per );
      rate.sleep();
    }
  }

private:
  ros::Publisher command_pub;
  ros::Subscriber sub_joystick;
  std_msgs::Float64MultiArray command;
  std::thread odometry_thread;

  double velocity_translational = 0;
  double velocity_angular = 0;

  state_struct state;

};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_control");
  ros::NodeHandle nh;
  VelocityControl control(&nh);
  ROS_INFO("Hello world!");

  ros::spin();
}
