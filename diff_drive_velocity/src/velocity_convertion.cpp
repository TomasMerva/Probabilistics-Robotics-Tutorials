#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <thread>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>


struct state_struct
{
  double x = 0;
  double y = 0;
  double theta = 0;
};

class VelocityControl
{
public:
  VelocityControl(ros::NodeHandle *nh) : odometry_thread(&VelocityControl::ComputeOdometry, this)
  {
    command_pub = nh->advertise<std_msgs::Float64MultiArray>("/joints_controller/command",1);
    sub_joystick = nh->subscribe("/cmd_vel", 1, &VelocityControl::Callback, this);

    odom_pub = nh->advertise<nav_msgs::Odometry>("odom", 1);
  }

  void Callback(const geometry_msgs::Twist msg){
    velocity_translation = msg.linear.x;
    velocity_angular = msg.angular.z;
    double velocity_right = (2*msg.linear.x+msg.angular.z*0.4)/(2*0.075);
    double velocity_left = (2*msg.linear.x-msg.angular.z*0.4)/(2*0.075);
    command.data.push_back(velocity_right);
    command.data.push_back(velocity_left);
    command_pub.publish(command);
    command.data.clear();
  }

  void ComputeOdometry()
  {
    static tf2_ros::TransformBroadcaster odom_broadcaster;
    double freq = 50;
    ros::Rate rate(freq);
    while(ros::ok())
    {
      current_time = ros::Time::now();
      double dt = (current_time - last_time).toSec();


      if (velocity_angular > 0)
      {
        state.x = state.x + velocity_translation*dt*cos(state.theta);
        state.y = state.y + velocity_translation*dt*sin(state.theta);
        state.theta = state.theta;
      }
      else
      {
//        if (velocity_angular < 1e-3 && velocity_angular > 0) velocity_angular = 1e-3;
//        if (velocity_angular < -1e-3 && velocity_angular < 0) velocity_angular = -1e-3;

        auto div = velocity_translation/velocity_angular;
        state.x = state.x + ( -div*sin(state.theta) + div*sin(state.theta + velocity_angular*dt) );
        state.y = state.y + ( div*cos(state.theta) - div*cos(state.theta + velocity_angular*dt) );
        state.theta = state.theta + ( velocity_angular*dt ); //radians
        if(state.theta >= 2*M_PI)
        {
          state.theta -= 2*M_PI;
        }
        else if (state.theta < 0)
        {
          state.theta += 2*M_PI;
        }
        else
        {

        }
      }



      odom_quat.setRPY(0, 0, state.theta);
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_footprint";

      odom_trans.transform.translation.x = state.x;
      odom_trans.transform.translation.y = state.y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation.x = odom_quat.getX();
      odom_trans.transform.rotation.y = odom_quat.getY();
      odom_trans.transform.rotation.z = odom_quat.getZ();
      odom_trans.transform.rotation.w = odom_quat.getW();
      odom_broadcaster.sendTransform(odom_trans);

      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";
      odom.pose.pose.position.x = state.x;
      odom.pose.pose.position.y = state.y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation.x = odom_quat.getX();
      odom.pose.pose.orientation.y = odom_quat.getY();
      odom.pose.pose.orientation.z = odom_quat.getZ();
      odom.pose.pose.orientation.w = odom_quat.getW();

      //set the velocity
      odom.child_frame_id = "base_footprint";
      odom.twist.twist.linear.x = velocity_translation;
      odom.twist.twist.angular.z = velocity_angular;

      //odom_pub.publish(odom);


      last_time = current_time;
      rate.sleep();
    }
  }

private:
  ros::Publisher command_pub;
  ros::Subscriber sub_joystick;
  std_msgs::Float64MultiArray command;
  std::thread odometry_thread;
  ros::Time current_time;
  ros::Time last_time;


  double velocity_translation = 0;
  double velocity_angular = 0;

  state_struct state;

  ros::Publisher odom_pub;
  geometry_msgs::TransformStamped odom_trans;
  tf2::Quaternion odom_quat;
  nav_msgs::Odometry odom;
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_control");
  ros::NodeHandle nh;
  VelocityControl control(&nh);
  ROS_INFO("Hello world!");
  std::cout << argv[0] << std::endl;
  ros::spin();
}
