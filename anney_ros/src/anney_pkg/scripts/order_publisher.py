#!/usr/bin/env
import rospy
# from std_msgs.msg import String
from anney_pkg.msg import Order
import re


def talk_to_me():
    pub = rospy.Publisher("order_topic", Order, queue_size=10)
    rospy.init_node("publisher_node", anonymous=True)
    rospy.loginfo("Publisher Node Started")

    while not rospy.is_shutdown():
        msg = Order()
        order_list = []
        unit_list = []

        table_num = int(input("Please enter your table number:"))
        order = input("Enter your order:")
        matches = re.findall("(([0-9]+) ([A-Za-z\s]+))", order)

        for match in matches:
            unit_list.append(int(match[1]))
            order_list.append(match[2].strip().replace(" ", "_"))

        msg.table_num = table_num
        msg.orders = order_list
        msg.units = unit_list

        pub.publish(msg)


if __name__ == "__main__":
    try:
        talk_to_me()
    except rospy.ROSInterruptException:
        pass
