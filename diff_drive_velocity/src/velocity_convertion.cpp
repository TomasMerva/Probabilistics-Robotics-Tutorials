#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>



class VelocityControl
{
public:
  VelocityControl(ros::NodeHandle *nh)
  {
    command_pub = nh->advertise<std_msgs::Float64MultiArray>("/joints_controller/command",1);
    sub_joystick = nh->subscribe("/cmd_vel", 1, &VelocityControl::Callback, this);
  }
  void Callback(const geometry_msgs::Twist msg){
    double velocity_right = (2*msg.linear.x+msg.angular.z*0.4)/(2*0.075);
    double velocity_left = (2*msg.linear.x-msg.angular.z*0.4)/(2*0.075);
    std::cout << "velocity right: " << velocity_right << "  velocity_left: " << velocity_left << std::endl;
    command.data.push_back(velocity_right);
    command.data.push_back(velocity_left);
    command_pub.publish(command);
    command.data.clear();
  }

private:
  ros::Publisher command_pub;
  ros::Subscriber sub_joystick;
  std_msgs::Float64MultiArray command;

};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_control");
  ros::NodeHandle nh;
  VelocityControl control(&nh);
  ROS_INFO("Hello world!");

  ros::spin();
}
