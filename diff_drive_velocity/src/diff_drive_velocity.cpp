#include <ros/ros.h>
#include <diff_drive_velocity/velocity_control.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "diff_drive_velocity");
  ros::NodeHandle nh;

  VelocityControl control(&nh);

   ros::AsyncSpinner spinner(2);
   spinner.start();

  diff_drive_velocity::state robot_state;

  ros::Time current_time = ros::Time::now();;
  ros::Time last_time = ros::Time::now();;

  ros::Rate rate(50);
  while(ros::ok())
  {
    current_time = ros::Time::now();
    auto dt = 0.02;
    control.ComputeOdometry(&robot_state, dt);

    control.PublishOdometry(robot_state, current_time);
    //control.VisualizeOdometry(robot_state, current_time);  //NOT WORKING

    last_time = current_time;
    rate.sleep();
  }


  ros::waitForShutdown();
  return 0;
}
