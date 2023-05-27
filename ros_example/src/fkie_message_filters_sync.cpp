#include <geometry_msgs/PointStamped.h>
#include <fkie_message_filters/combiner_policies/approximate_time.h>
#include <fkie_message_filters/combiner.h>
#include <fkie_message_filters/fkie_message_filters.h>
#include <fkie_message_filters/simple_user_filter.h>
#include <ros/ros.h>


// bool callback(const geometry_msgs::PointStamped::ConstPtr& msg_a,
//              const geometry_msgs::PointStamped::ConstPtr& msg_b)
bool callback(const geometry_msgs::PointStamped& msg_a,
              const geometry_msgs::PointStamped& msg_b)
{
  const auto age = ros::Time::now() - msg_b.header.stamp;
  const auto stamp_diff = msg_a.header.stamp - msg_b.header.stamp;
  ROS_INFO_STREAM(age << "s " << stamp_diff << "s " << msg_a.point.x << " " << msg_b.point.x);
  return true;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "fkie_message_filters_sync");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  namespace mf = fkie_message_filters;

  using Source = mf::Subscriber<geometry_msgs::PointStamped, mf::RosMessage>;

  bool exact = false;
  private_nh.getParam("exact", exact);

  using ApproxSync = mf::Combiner<mf::combiner_policies::ApproximateTime, Source::Output, Source::Output>;
  using ApproxSink = mf::SimpleUserFilter<ApproxSync::Output>;

  using ExactSync = mf::Combiner<mf::combiner_policies::ExactTime, Source::Output, Source::Output>;
  using ExactSink = mf::SimpleUserFilter<ExactSync::Output>;

  Source sub_a(nh, "a", 10);
  Source sub_b(nh, "b", 10);

  if (exact) {
    ROS_INFO_STREAM("exact sync");
    auto policy = ExactSync::Policy();
    policy.set_max_age(ros::Duration(4.0));
    // policy.set_max_timespan(ros::Duration(0.002));
    ExactSync combiner(policy);
    ExactSink snk;
    combiner.connect_to_sources(sub_a, sub_b);
    combiner.connect_to_sink(snk);

    snk.set_processing_function(&callback);
#if 0
    snk.set_processing_function(
      [&](const geometry_msgs::PointStamped& msg_a, const geometry_msgs::PointStamped& msg_b) -> bool
      {
        return callback(msg_a, msg_b);
      }
    );
#endif
    ros::spin();
  } else {
    double max_interval = 0.002;
    private_nh.getParam("max_interval", max_interval);
    ROS_INFO_STREAM("approx sync " << max_interval);
    auto policy = ApproxSync::Policy();
    policy.set_max_age(ros::Duration(1.5));
    policy.set_max_timespan(ros::Duration(max_interval));
    // without these will have to wait a full update time period for the sync to trigger
    // https://github.com/fkie/message_filters/issues/2
    policy.set_min_distance(0, ros::Duration(max_interval * 2.0));
    policy.set_min_distance(1, ros::Duration(max_interval * 2.0));
    ApproxSync combiner(policy);
    ApproxSink snk;
    combiner.connect_to_sources(sub_a, sub_b);
    combiner.connect_to_sink(snk);

    snk.set_processing_function(&callback);
    ros::spin();
  }

  return 0;
}
