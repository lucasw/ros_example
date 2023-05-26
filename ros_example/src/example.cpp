#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>


void callback(const geometry_msgs::PointStamped::ConstPtr& msg_a,
              const geometry_msgs::PointStamped::ConstPtr& msg_b)
{
  ROS_INFO_STREAM(msg_a->point.x << " " << msg_b->point.x);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "example");
  ros::NodeHandle nh;

  message_filters::Subscriber<geometry_msgs::PointStamped> a_sub(nh, "a", 10);
  message_filters::Subscriber<geometry_msgs::PointStamped> b_sub(nh, "b", 10);

  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped,
                                                          geometry_msgs::PointStamped> MySyncPolicy;

  message_filters::Synchronizer<MySyncPolicy> approx_sync(MySyncPolicy(10), a_sub, b_sub);
  approx_sync.registerCallback(std::bind(&callback, std::placeholders::_1, std::placeholders::_2));

  ROS_INFO_STREAM("set up sync subscribers");
  ros::spin();

  return 0;
}
