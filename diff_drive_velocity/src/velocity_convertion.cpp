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
    std::cout << "som tu" << std::endl;
    command.data.push_back(msg.linear.x);
    command.data.push_back(msg.linear.x);
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
