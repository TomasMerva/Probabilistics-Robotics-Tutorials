#include <diff_drive_velocity/velocity_control.h>




VelocityControl::VelocityControl(ros::NodeHandle *nh)
{
  joints_controller_pub = nh->advertise<std_msgs::Float64MultiArray>("/joints_controller/command",1);
  joystick_sub = nh->subscribe("/cmd_vel", 1, &VelocityControl::Callback_UnicycleModel, this);

  odometry_pub = nh->advertise<nav_msgs::Odometry>("/odom",1);
  marker_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 100);

}


void VelocityControl::Callback_UnicycleModel(const geometry_msgs::Twist msg)
{

  robot_velocity.translation = msg.linear.x;
  robot_velocity.angular = msg.angular.z;
  //Unicycle Model
  double velocity_right = (2*robot_velocity.translation+robot_velocity.angular*0.4)/(2*0.075);
  double velocity_left = (2*robot_velocity.translation-robot_velocity.angular*0.4)/(2*0.075);
  //Publishing velocities for the right and left wheels
  joints_command.data.push_back(velocity_left);
  joints_command.data.push_back(velocity_right);
  joints_controller_pub.publish(joints_command);
  joints_command.data.clear();
}




void VelocityControl::ComputeOdometry(diff_drive_velocity::state* robot_state, double dt)
{
  if (abs(robot_velocity.angular) < 1e-3) robot_velocity.angular = 1e-3;
  auto div = robot_velocity.translation / robot_velocity.angular;
  robot_state->x = robot_state->x + ( -div*sin(robot_state->theta) + div*sin(robot_state->theta + robot_velocity.angular*dt) );
  robot_state->y = robot_state->y + ( div*cos(robot_state->theta) - div*cos(robot_state->theta + robot_velocity.angular*dt) );
  robot_state->theta = robot_state->theta + ( robot_velocity.angular*dt ); //radians

  // Normalize orientation of the robot
  if(robot_state->theta >= 2*M_PI)
  {
    robot_state->theta -= 2*M_PI;
  }
  else if (robot_state->theta < 0)
  {
    robot_state->theta += 2*M_PI;
  }
  else
  {

  }
}

void VelocityControl::PublishOdometry(const diff_drive_velocity::state robot_state, const ros::Time current_time)
{
  static tf2_ros::TransformBroadcaster odometry_broadcaster;
  tf2::Quaternion odometry_quat;
  nav_msgs::Odometry odometry_msg;
  geometry_msgs::TransformStamped odometry_transf;

  odometry_quat.setRPY(0, 0, robot_state.theta);
  odometry_transf.header.stamp = current_time;
  odometry_transf.header.frame_id = "odom";
  odometry_transf.child_frame_id = "base_footprint";

  odometry_transf.transform.translation.x = robot_state.x;
  odometry_transf.transform.translation.y = robot_state.y;
  odometry_transf.transform.translation.z = 0.0;

  odometry_transf.transform.rotation.x = odometry_quat.getX();
  odometry_transf.transform.rotation.y = odometry_quat.getY();
  odometry_transf.transform.rotation.z = odometry_quat.getZ();
  odometry_transf.transform.rotation.w = odometry_quat.getW();

  odometry_broadcaster.sendTransform(odometry_transf);

  // Publish odometry topic
  odometry_msg.header.stamp = current_time;

  odometry_msg.header.frame_id = "odom";
  odometry_msg.pose.pose.position.x = robot_state.x;
  odometry_msg.pose.pose.position.y = robot_state.y;
  odometry_msg.pose.pose.position.z = 0.0;
  odometry_msg.pose.pose.orientation.x = odometry_quat.getX();
  odometry_msg.pose.pose.orientation.y = odometry_quat.getY();
  odometry_msg.pose.pose.orientation.z = odometry_quat.getZ();
  odometry_msg.pose.pose.orientation.w = odometry_quat.getW();

  //set the velocity
  odometry_msg.child_frame_id = "base_footprint";
  odometry_msg.twist.twist.linear.x = robot_velocity.translation;
  odometry_msg.twist.twist.angular.z = robot_velocity.angular;

  odometry_pub.publish(odometry_msg);
}


void VelocityControl::VisualizeOdometry(const diff_drive_velocity::state robot_state, const ros::Time current_time)
{
  tf2::Quaternion odometry_quat;
  odometry_quat.setRPY(0, 0, robot_state.theta);

  //Publish marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/odom";
  marker.ns = "basic_shapes";
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  // Set the color
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  marker.id = 0;
  marker_id++;
  marker.header.stamp = current_time;

  marker.pose.position.x = robot_state.x;
  marker.pose.position.y = robot_state.y;
  marker.pose.position.z = 0.0;

  marker.pose.orientation.x = odometry_quat.getX();
  marker.pose.orientation.y = odometry_quat.getY();
  marker.pose.orientation.z = odometry_quat.getZ();
  marker.pose.orientation.w = odometry_quat.getW();

  marker_pub.publish(marker);

}
