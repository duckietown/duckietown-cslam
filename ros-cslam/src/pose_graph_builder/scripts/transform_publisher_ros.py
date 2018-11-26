#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header, Time
from geometry_msgs.msg import *

import random


def talker():
    pub = rospy.Publisher('transform_stamped', TransformStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    seq = 0
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        myTransformStamped = TransformStamped()
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        s0 = random.randint(0, 5)

        h.frame_id = "duckie_%d" % s0
        myTransformStamped.header = h
        d = random.randint(0, 5)

        rotation = Quaternion(random.random(), random.random(),
                              random.random(), random.random())
        translation = Vector3(d, d + random.random() *
                              3, d + 4 * random.random())

        myTransformStamped.transform = Transform(translation, rotation)
        myTransformStamped.child_frame_id = "duckie_%d" % d
        pub.publish(myTransformStamped)

        rate.sleep()
        seq += 1


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
