/** Copyright 2020 Lucas Walter
 *
 */

#include <chrono>
#include <signal.h>
#include <thread>

#include <ros/ros.h>

using namespace std::chrono_literals;

// Replacement SIGINT handler
void my_sig_int_handler(int sig)
{
  std::cerr << "sig int" << std::endl;
  ROS_INFO_STREAM("sig int");
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ros_example");
  signal(SIGINT, my_sig_int_handler);
  ros::NodeHandle nh("~");
  // ros::start();

#if 0
  while (true) {
    std::this_thread::sleep_for(1000s);
  }
#endif

#if 1
  ros::spin();
#endif

#if 0
  while (ros::ok()) {
    ros::spinOnce();
  }
#endif

#if 0
  for (size_t i = 0; i < 100000; ++i) {
    ros::spinOnce();
  }
#endif

#if 0
  ros::Time t0 = ros::Time::now();
  while (ros::ok()) {
    ros::Duration(1.0).sleep();
    if ((ros::Time::now() - t0).toSec() > 1.0) {
      break;
    }
  }
#endif

  // if nothing at all is done also segfault on exit

  // ROS_INFO_STREAM("done with main");
  // ros::shutdown();
  return 0;
}
