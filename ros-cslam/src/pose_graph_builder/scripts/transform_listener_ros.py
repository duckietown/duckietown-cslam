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
        self.last_callback = rospy.get_time()
        self.optim_period = 0.5
        self.optim_period_counter = -10.0
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
            t = [0.0, 0.0, 0.1]
            z_angle = 90
            z_angle = np.deg2rad(z_angle)
            x_angle = np.deg2rad(180)
            R_z = g.rotation_from_axis_angle(
                np.array([0, 0, 1]), z_angle)
            R_x = g.rotation_from_axis_angle(
                np.array([1, 0, 0]), x_angle)
            R = np.matmul(R_x, R_z)
            H_apriltag_to_base = g2o.Isometry3d(R, t)
            transform = transform * H_apriltag_to_base

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
            H_base_to_camera = g2o.Isometry3d(R, t)
            transform = H_base_to_camera * transform
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
        self.n += 1
        self.optim_period_counter += a - self.last_callback
        self.last_callback = a
        id0 = data.header.frame_id
        id1 = data.child_frame_id

        id0 = self.filter_name(id0)
        id1 = self.filter_name(id1)

        if(id0 == "watchtower_5"):
            # self.lock.release()
            return 0

        isfromwatchtower = False
        if(id0.startswith("watchtower")):
            isfromwatchtower = True
            if(id1.startswith("watchtower")):
                # print(data)
                # self.lock.release()
                return 0

        t = [data.transform.translation.x,
             data.transform.translation.y, data.transform.translation.z]
        # to transform to a rotation matrix!
        # Careful, in Pygeometry, quaternion is (w, x, y, z)

        q = [data.transform.rotation.w, data.transform.rotation.x,
             data.transform.rotation.y, data.transform.rotation.z]
        M = g.rotations.rotation_from_quaternion(np.array(q))
        det = np.linalg.det(M)
        if(det < 0):
            print("det is %f" % det)

        transform = g2o.Isometry3d(M, t)
        time = Time(data.header.stamp)
        time_stamp = time.data.secs + time.data.nsecs * 10**(-9)
        if(id1 == id0):
            self.handle_odometry_message(id1, transform, time_stamp)
        elif(isfromwatchtower):
            self.handle_watchtower_message(id0, id1, transform,
                                           time_stamp)
        else:
            self.handle_duckiebot_message(id0, id1, transform, time_stamp)

        if(self.n > 60):
            self.mygraph.optimize(
                4,  save_result=True, verbose=True, output_name="/tmp/test2.g2o")
            self.optim_period_counter = 0.0
            self.n = 0
        # self.lock.release()
        # if(self.n % 50 == 0):
            pose_dict = self.mygraph.get_all_poses()

            for node_type, node_list in pose_dict.iteritems():
                for node_id, node_pose in node_list.iteritems():
                    self.tfbroadcast(node_type, node_id, node_pose)

        b = rospy.get_time()
        diff = b-a
        self.last_callback = rospy.get_time()
        # if(self.n == 0):
        #     print("listener frequency is %f " % (1.0/diff))

    def tfbroadcast(self, node_type, node_id, node_pose):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        if(node_type == "duckie"):
            t.header.frame_id = "map"
        else:
            t.header.frame_id = "map"
        t.child_frame_id = "%s_%s" % (node_type, node_id)
        t.transform.translation.x = node_pose.t[0]
        t.transform.translation.y = node_pose.t[1]
        t.transform.translation.z = node_pose.t[2]
        # print(node_pose.R)
        det = np.linalg.det(node_pose.R)
        if(det < 0):
            print("after optim : det = %f" % det)
        q = g.rotations.quaternion_from_rotation(node_pose.R)
        # Careful, in Pygeometry, quaternion is (w, x, y, z)
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

        initial_floor_april_tags = "%s/%s" % (rospy.get_param(
            "config_folder"), "robotarium1.yaml")
        self.mygraph = dGB.duckietownGraphBuilder(
            initial_floor_april_tags=initial_floor_april_tags)
        self.initialize_id_map()

        rospy.Subscriber("/poses_acquisition/poses",
                         TransformStamped, self.callback)

        rospy.Subscriber("/poses_acquisition/odometry",
                         TransformStamped, self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


def main():
    rospy.init_node('listener', anonymous=True)

    tflistener = transform_listener()
    tflistener.listen()


if __name__ == '__main__':
    main()
