#!/usr/bin/env python
# Lucas Walter
# May 2023
#
# publish two PointStamped messages exactly in sync or slightly out of sync for using
# with an exact or approximate sync subscriber

import rospy
from geometry_msgs.msg import PointStamped


if __name__ == "__main__":
    rospy.init_node("sync_pub")

    offset = rospy.get_param("~offset", 1e-6)

    pub_a = rospy.Publisher("a", PointStamped, queue_size=3)
    pub_b = rospy.Publisher("b", PointStamped, queue_size=3)

    rate = rospy.Rate(1.0)

    value = 0.0

    while not rospy.is_shutdown():
        stamp = rospy.Time.now()

        ps_a = PointStamped()
        ps_a.header.stamp = stamp
        ps_a.point.x = value
        pub_a.publish(ps_a)

        ps_b = PointStamped()
        ps_b.header.stamp = stamp + rospy.Duration(offset)
        ps_b.point.x = value
        pub_b.publish(ps_b)

        rate.sleep()
