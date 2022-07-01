#!/usr/bin/env
import rospy
# from std_msgs.msg import String
from anney_pkg.msg import Order


def callback(data):
    rospy.loginfo(f"Received Data: {data}")


def listener():
    rospy.init_node("subscriber_node", anonymous=True)
    rospy.Subscriber("order_topic", Order, callback)
    rospy.spin()


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
