#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header, Time
from geometry_msgs.msg import *
import duckietown_cslam.duckietownGraphBuilder.duckietown_graph_builder as dGB
import g2o
import numpy as np

mygraph = dGB.duckietownGraphBuilder()


def callback(data):
    # global mygraph
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    t = [data.transform.translation.x,
         data.transform.translation.y, data.transform.translation.z]
    # to transform to a rotation matrix!
    q = data.transform.rotation

    transform = g2o.Isometry3d(np.identity(3), t)
    id0 = data.header.frame_id
    id1 = data.child_frame_id
    fixed = False
    time = Time(data.header.stamp)
    if id0 == "daddy":
        fixed = id0 = 0
    rospy.loginfo("Adding new edge")
    mygraph.add_edge(id0, id1, transform, time.data.secs)
    mygraph.optimize(15, output_name="/home/amaury/test.g2o")


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("transform_stamped", TransformStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def main():
    listener()


if __name__ == '__main__':
    main()
