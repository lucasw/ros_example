#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>


void callback(const geometry_msgs::PointStamped::ConstPtr& msg_a,
              const geometry_msgs::PointStamped::ConstPtr& msg_b)
{
  const auto age = ros::Time::now() - msg_b->header.stamp;
  const auto stamp_diff = msg_a->header.stamp - msg_b->header.stamp;
  ROS_INFO_STREAM(age << "s " << stamp_diff << "s " << msg_a->point.x << " " << msg_b->point.x);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "example");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("");

  message_filters::Subscriber<geometry_msgs::PointStamped> a_sub(nh, "a", 10);
  message_filters::Subscriber<geometry_msgs::PointStamped> b_sub(nh, "b", 10);

  double max_interval = 0.002;
  private_nh.getParam("max_interval", max_interval);
  ROS_INFO_STREAM("approx sync " << max_interval);
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped,
                                                          geometry_msgs::PointStamped> MySyncPolicy;

  message_filters::Synchronizer<MySyncPolicy> approx_sync(MySyncPolicy(10), a_sub, b_sub);
  approx_sync.setMaxIntervalDuration(ros::Duration(max_interval));
  // without this low update rate messages will wait a full time period before triggering the callback
  approx_sync.setInterMessageLowerBound(0, ros::Duration(max_interval * 2.0));
  approx_sync.setInterMessageLowerBound(1, ros::Duration(max_interval * 2.0));
  approx_sync.registerCallback(std::bind(&callback, std::placeholders::_1, std::placeholders::_2));

  ROS_INFO_STREAM("set up sync subscribers");
  ros::spin();

  return 0;
}
