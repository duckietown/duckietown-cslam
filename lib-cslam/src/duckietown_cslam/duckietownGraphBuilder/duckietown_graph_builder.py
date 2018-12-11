import numpy as np
import g2o
import duckietown_cslam.g2oGraphBuilder.g2ograph_builder as g2oBG
import geometry as g
import random
import yaml
import threading

time_step = 0.5


class duckietownGraphBuilder():

    def __init__(self,
                 initial_duckie_dict={},
                 initial_watchtower_dict={},
                 initial_april_dict={},
                 initial_floor_april_tags="",
                 retro_interpolate=True):
        self.graph = g2oBG.g2oGraphBuilder()
        self.types = ["duckie", "watchtower", "apriltag"]
        initial_dicts = [
            initial_duckie_dict, initial_watchtower_dict, initial_april_dict
        ]
        self.lists = dict(zip(self.types, initial_dicts))
        self.movable = ["duckie"]
        self.counters = dict()
        self.last_time_stamp = dict()
        self.lock = threading.Lock()
        self.retro_interpolate = retro_interpolate

        if (initial_floor_april_tags != ""):
            self.load_initial_floor_april_tags(initial_floor_april_tags)

    def load_initial_floor_april_tags(self, initial_floor_april_tag_file):
        with open(initial_floor_april_tag_file, 'r') as stream:
            try:
                complete_dict = yaml.safe_load(stream)
                vertexPose = g2o.Isometry3d(np.eye(3), [0, 0, 0])
                self.graph.add_vertex(0, vertexPose, fixed=True)

                for key, value in complete_dict.iteritems():
                    if key == "objects":
                        for myobject, object_value in value.iteritems():
                            if (object_value['kind'] == "floor_tag"):
                                tag_id = object_value['tag']['~TagInstance'][
                                    'tag_id']
                                position = object_value['pose'][
                                    '~SE2Transform']['p']
                                theta = 0
                                if ("theta_deg" in object_value['pose'][
                                        '~SE2Transform']):
                                    theta = object_value['pose'][
                                        '~SE2Transform']['theta_deg']
                                vertex_id = "apriltag_%d" % tag_id
                                self.add_vertex(
                                    vertex_id,
                                    theta,
                                    position,
                                    True,
                                    fixed=True)
                print(self.lists)
            except yaml.YAMLError as exc:
                print(exc)

    def add_node_to_list(self, node_type, node_id, time_stamp):
        vertexId = "%s_%s" % (node_type, node_id)
        if node_type not in self.counters:
            self.counters[node_type] = dict()
            self.last_time_stamp[node_type] = dict()
        if node_id not in self.counters[node_type]:
            self.counters[node_type][node_id] = 0
            self.last_time_stamp[node_type][node_id] = 0.0

        count = self.counters[node_type][node_id]

        if node_type not in self.lists:
            self.lists[node_type] = dict()
        if node_id not in self.lists[node_type]:
            self.lists[node_type][node_id] = dict()
        if time_stamp not in self.lists[node_type][node_id] and (
                node_type in self.movable or count == 0):
            self.lists[node_type][node_id][time_stamp] = count
            self.counters[node_type][
                node_id] = self.counters[node_type][node_id] + 1

            if (time_stamp > self.last_time_stamp[node_type][node_id]):
                self.last_time_stamp[node_type][node_id] = time_stamp
            else:
                if (self.retro_interpolate):
                    sorted_time_stamp = sorted(
                        self.lists[node_type][node_id].keys())
                    time_stamp_index = sorted_time_stamp.index(time_stamp)
                    if (time_stamp_index > 0 and
                            time_stamp_index < len(sorted_time_stamp) - 1):

                        before = self.convert_names_to_int(
                            vertexId, sorted_time_stamp[time_stamp_index - 1])
                        after = self.convert_names_to_int(
                            vertexId, sorted_time_stamp[time_stamp_index + 1])
                        transform = self.graph.get_transform(before, after)
                        if (transform != 0):
                            print("Will perform retro-interpolation")

                            self.interpolate(
                                sorted_time_stamp[time_stamp_index - 1],
                                sorted_time_stamp[time_stamp_index + 1],
                                node_type, node_id, transform)
                        else:
                            print(
                                "will not perform retro_interpolation with %d and %d "
                                % (before, after))
            # print("counter is %d and len(time_stamp_list) is %d" % (
            #     self.counters[node_type][node_id], len(self.lists[node_type][node_id])))
            # print(self.lists[node_type][node_id])
            return True
        return False

    def add_vertex(self,
                   vertex_id,
                   theta,
                   p,
                   isinitialfloortag=False,
                   fixed=False,
                   time_stamp=0.0):
        [node_type, node_id] = vertex_id.split("_")
        added = self.add_node_to_list(node_type, node_id, time_stamp)
        if (not added):
            print(
                "add_vertex did not add : node %s at time %f as already there" %
                (vertex_id, time_stamp))
        else:
            p = [p[0], p[1], 0.0]
            R = g.rotation_from_axis_angle(
                np.array([0, 0, 1]), np.deg2rad(theta))
            if (isinitialfloortag):
                R2 = g.rotation_from_axis_angle(
                    np.array([1, 0, 0]), np.deg2rad(180))
                R = np.matmul(R, R2)
            # print(R, theta)
            vertexPose = g2o.Isometry3d(R, p)
            vertex_id = self.convert_names_to_int(vertex_id, time_stamp)
            self.graph.add_vertex(vertex_id, vertexPose, fixed=fixed)
            if (isinitialfloortag):
                self.graph.add_edge(0, vertex_id, vertexPose)

    def convert_names_to_int(self, id, time_stamp):
        [node_type, node_id] = id.split("_")
        a = 0
        b = int(node_id) % 1000

        if (node_type == "duckie"):
            a = 1
        elif (node_type == "watchtower"):
            a = 2
        elif (node_type == "apriltag"):
            a = 3

        if (node_type in self.movable):
            c = self.lists[node_type][node_id][time_stamp]
            if (c >= 100000):
                print("overflow of the time_stamp list")
        else:
            c = 0

        result = a * 10**8 + b * 10**5 + c
        return result

    def interpolate(self, old_time_stamp, new_time_stamp, node_type, node_id,
                    measure):
        vertexId = "%s_%s" % (node_type, node_id)
        to_interpolate = {
            time_stamp: self.lists[node_type][node_id][time_stamp]
            for time_stamp in self.lists[node_type][node_id].keys()
            if (time_stamp >= old_time_stamp and time_stamp <= new_time_stamp)
        }
        # print("interpolating one odometry measure on %d nodes" %
        #       len(to_interpolate))
        sorted_time_stamps = sorted(to_interpolate.keys())

        total_delta_t = float(sorted_time_stamps[-1] - sorted_time_stamps[0])
        if (total_delta_t == 0.0):
            print("in interpolate, delta t is 0.0, with %s %s and list is:" %
                  (node_type, node_id))
            print(to_interpolate)
            print("new_time_stamp is %f and old time stamp is %f" %
                  (old_time_stamp, new_time_stamp))
            print(self.lists[node_type][node_id])
        R = measure.R
        t = measure.t
        q = g.SE3_from_rotation_translation(R, t)
        vel = g.SE3.algebra_from_group(q)
        check = 0.0
        for i in range(0, len(sorted_time_stamps) - 1):
            partial_delta_t = float(
                sorted_time_stamps[i + 1] - sorted_time_stamps[i])
            alpha = partial_delta_t / total_delta_t
            check += alpha
            rel = g.SE3.group_from_algebra(vel * alpha)
            newR, newt = g.rotation_translation_from_SE3(rel)
            interpolated_measure = g2o.Isometry3d(newR, newt)
            vertex0Id_int = self.convert_names_to_int(vertexId,
                                                      sorted_time_stamps[i])
            vertex1Id_int = self.convert_names_to_int(vertexId,
                                                      sorted_time_stamps[i + 1])
            self.graph.add_edge(vertex0Id_int, vertex1Id_int,
                                interpolated_measure)
        # print("end interpolation")
        if (check != 1.0):
            # print("damn, check is %.4f instead of 1.0" % check)
            pass

    def optimize(self,
                 number_of_steps,
                 verbose=True,
                 save_result=True,
                 output_name="output.g2o"):
        self.graph.optimize(
            number_of_steps,
            verbose=verbose,
            save_result=save_result,
            output_name=output_name)

    def add_edge(self,
                 vertex0Id,
                 vertex1Id,
                 measure,
                 time_stamp,
                 old_time_stamp=0):
        # print(self.lists)
        if (vertex0Id != vertex1Id):
            for vertexId in [vertex0Id, vertex1Id]:
                if (len(vertexId.split("_")) == 1):
                    print("Error, vertexname is %s. Exiting" % vertexId)
                    exit(-1)
                [node_type, node_id] = vertexId.split("_")
                added = self.add_node_to_list(node_type, node_id, time_stamp)
                if not added:
                    pass
            vertex0Id_int = self.convert_names_to_int(vertex0Id, time_stamp)
            vertex1Id_int = self.convert_names_to_int(vertex1Id, time_stamp)

            self.graph.add_edge(vertex0Id_int, vertex1Id_int, measure)
        else:
            [node_type, node_id] = vertex0Id.split("_")
            added = self.add_node_to_list(node_type, node_id, time_stamp)

            if (node_type in self.movable):
                if (old_time_stamp == 0):
                    old_time_stamp = sorted(
                        self.lists[node_type][node_id].keys())[0]
                if (old_time_stamp != time_stamp):
                    self.interpolate(old_time_stamp, time_stamp, node_type,
                                     node_id, measure)
                else:
                    self.add_vertex(
                        vertex0Id,
                        0, [0, 0],
                        isinitialfloortag=False,
                        fixed=False,
                        time_stamp=time_stamp)
            else:

                print(
                    "Node type %s should be movable if given odometry transform"
                    % node_type)
                exit(-1)

    def get_all_poses(self):
        result_dict = {}
        for node_type, listdict in self.lists.iteritems():
            result_dict[node_type] = {}
            for node_id, time_stamp_dict in listdict.iteritems():
                last_time_stamp = self.last_time_stamp[node_type][node_id]
                vertex_id = "%s_%s" % (node_type, node_id)
                if (self.convert_names_to_int(vertex_id, last_time_stamp) not in
                        self.graph.optimizer.vertices()):
                    # print("len(time_stamp_dict) = %i for %s %s" %
                    #       (len(time_stamp_dict), node_type, node_id))
                    if (node_type in self.movable):
                        last_time_stamp = sorted(time_stamp_dict.keys())[-2]

                result_dict[node_type][node_id] = self.graph.vertex_pose(
                    self.convert_names_to_int(vertex_id, last_time_stamp))
        # print(result_dict)
        return result_dict
