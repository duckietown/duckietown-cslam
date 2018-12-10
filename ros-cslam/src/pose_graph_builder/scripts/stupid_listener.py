#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header, Time
from geometry_msgs.msg import *
import duckietown_cslam.duckietownGraphBuilder.duckietown_graph_builder as dGB
import g2o
import numpy as np
import geometry as g
import yaml
import tf_conversions
import tf2_ros
import threading
# mygraph = dGB.duckietownGraphBuilder(
#     initial_floor_april_tags="/home/amaury/duckietown/duckietown-cslam/lib-cslam/src/duckietown_cslam/duckietownGraphBuilder/robotarium1.yaml")
# old_stamps = {}

# id_map = {}
# # aprilstagDB = "/home/amaury/duckietown/duckietown-world/src/duckietown_world/data/apriltagsDB.yaml"
# n = 0


class transform_listener():
    def __init__(self):
        self.mygraph = None
        self.old_stamps = {}
        self.id_map = {}
        self.n = 0
        # self.lock = threading.Lock()

    def initialize_id_map(self):
        global id_map
        config_folder = rospy.get_param("config_folder")
        aprilstagDB = "%s/%s" % (config_folder, "apriltagsDB.yaml")
        with open(aprilstagDB, 'r') as stream:
            try:
                complete_dict = yaml.safe_load(stream)
                for myobject in complete_dict:
                    tag_id = myobject["tag_id"]
                    mytype = myobject['tag_type']
                    self.id_map[str(tag_id)] = mytype
                # print(id_map)
            except yaml.YAMLError as exc:
                print(exc)

    def find_vertex_name(self, id):
        if(self.id_map[id] == "Vehicle"):
            id = "duckie_%s" % id
        else:
            id = "apriltag_%s" % id
        # print(id)
        return id

    def handle_odometry_message(self, id, transform, time_stamp):
        old_time_stamp = 0

        if(id in self.old_stamps):
            old_time_stamp = self.old_stamps[id]
        self.old_stamps[id] = time_stamp
        self.mygraph.add_edge(id, id, transform,
                              time_stamp, old_time_stamp)

    def handle_watchtower_message(self, id0, id1, transform,
                                  time_stamp):
        node_type = id1.split("_")[0]
        if(node_type == "duckie"):
            # do some transform
            t = [0.1, 0.0, 0.1]
            z_angle = -90
            z_angle = np.deg2rad(z_angle)

            R_z = g.rotation_from_axis_angle(
                np.array([0, 0, 1]), z_angle)
            rectification = g2o.Isometry3d(R_z, t)
            transform = transform * rectification

        self.mygraph.add_edge(id0, id1, transform,
                              time_stamp)

    def handle_duckiebot_message(self, id0, id1, transform,
                                 time_stamp):
        node_type = id0.split("_")[0]
        if(node_type == "duckie"):
            # do some transform
            t = [0.1, 0.0, 0.1]
            y_angle = 105
            z_angle = -90
            y_angle = np.deg2rad(y_angle)
            z_angle = np.deg2rad(z_angle)
            R_y = g.rotation_from_axis_angle(
                np.array([0, 1, 0]), y_angle)

            R_z = g.rotation_from_axis_angle(
                np.array([0, 0, 1]), z_angle)
            R = np.matmul(R_y, R_z)
            rectification = g2o.Isometry3d(R, t)
            transform = transform * rectification
        else:
            print("This should not be here!")
        self.mygraph.add_edge(id0, id1, transform,
                              time_stamp)

    def filter_name(self, id):
        if(id == "donaldthegreat"):
            id = "duckie_88"

        elif(id.startswith("watchtower")):
            id = "watchtower_%d" % int(id.strip("watchtower"))

        elif(len(id.split("_")) == 1):
            id = self.find_vertex_name(id)

        return id

    def callback(self, data):
        # self.lock.acquire()
        a = rospy.get_time()
        id0 = data.header.frame_id
        id1 = data.child_frame_id

        t = [data.transform.translation.x,
             data.transform.translation.y, data.transform.translation.z]
        # to transform to a rotation matrix!
        q = [data.transform.rotation.w, data.transform.rotation.x,
             data.transform.rotation.y, data.transform.rotation.z]
        M = g.rotations.rotation_from_quaternion(np.array(q))
        print(M)
        # print(M)
        det = np.linalg.det(M)
        if(det < 0):
            print("det is %f" % det)

        node_pose = g2o.Isometry3d(M, t)
        # print(node_pose.R - M)
        self.tfbroadcast(id1, node_pose)

        # print("difference time is %f " % diff)

    def tfbroadcast(self, id, node_pose):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()

        t.header.frame_id = "map"
        t.child_frame_id = id
        t.transform.translation.x = node_pose.t[0]
        t.transform.translation.y = node_pose.t[1]
        t.transform.translation.z = node_pose.t[2]
        # print(node_pose.R)
        det = np.linalg.det(node_pose.R)
        if(det < 0):
            print("after optim : det = %f" % det)
        q = g.rotations.quaternion_from_rotation(node_pose.R)
        t.transform.rotation.w = q[0]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]

        br.sendTransform(t)

    def listen(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.

        rospy.init_node('listener', anonymous=True)
        initial_floor_april_tags = "%s/%s" % (rospy.get_param(
            "config_folder"), "robotarium1.yaml")
        self.mygraph = dGB.duckietownGraphBuilder(
            initial_floor_april_tags="")
        self.initialize_id_map()

        rospy.Subscriber("/poses_acquisition/poses",
                         TransformStamped, self.callback)

        rospy.Subscriber("pose_odometry",
                         TransformStamped, self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


def main():
    tflistener = transform_listener()
    tflistener.listen()


if __name__ == '__main__':
    main()
