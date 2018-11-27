import numpy as np
import g2o
import duckietown_cslam.g2oGraphBuilder.g2ograph_builder as g2oBG
import geometry as g
import random

time_step = 0.5


class duckietownGraphBuilder():
    def __init__(self, initial_duckie_dict={}, initial_watchtower_dict={}, initial_april_dict={}, smallest_time_step=0.010):
        self.graph = g2oBG.g2oGraphBuilder()
        self.types = ["duckie", "watchtower", "apriltag"]
        initial_dicts = [initial_duckie_dict,
                         initial_watchtower_dict, initial_april_dict]
        self.lists = dict(zip(self.types, initial_dicts))
        self.movable = ["duckie"]
        self.smallest_time_step = smallest_time_step

    def convert_names_to_int(self, s, time_stamp):
        items = s.split("_")
        a = 0
        b = int(items[1]) % 100
        c = int((time_stamp * 100))
        if(items[0] == "duckie"):
            a = 8
        elif(items[0] == "watchtower"):
            a = 2
            c = 0
        elif(items[0] == "apriltag"):
            a = 3
            c = 0

        result = a * 10**8 + b * 10**6 + c
        return result

    def convert_int_to_name(self, i):
        time_stamp = 123456789
        return ["duckie_5", time_stamp]

    def interpolate(self, old_stamp_index, node_type, node_id, measure, vertexId):
        to_interpolate = self.lists[node_type][node_id][old_stamp_index:]
        print("interpolating one odometry measure on %d nodes" %
              len(to_interpolate))
        total_delta_t = float(to_interpolate[-1] - to_interpolate[0])
        R = measure.R
        t = measure.t
        q = g.SE3_from_rotation_translation(R, t)
        vel = g.SE3.algebra_from_group(q)
        check = 0.0
        for i in range(0, len(to_interpolate)-1):
            partial_delta_t = float(to_interpolate[i+1] - to_interpolate[i])
            alpha = partial_delta_t/total_delta_t
            check += alpha
            rel = g.SE3.group_from_algebra(vel * alpha)
            newR, newt = g.rotation_translation_from_SE3(rel)
            interpolated_measure = g2o.Isometry3d(newR, newt)
            vertex0Id_int = self.convert_names_to_int(
                vertexId, to_interpolate[i])
            vertex1Id_int = self.convert_names_to_int(
                vertexId, to_interpolate[i+1])
            self.graph.add_edge(
                vertex0Id_int, vertex1Id_int, interpolated_measure)

        if(check != 1.0):
            print("damn, check is %.4f instead of 1.0" % check)

    def optimize(self, number_of_steps, verbose=True, save_result=True, output_name="output.g2o"):
        self.graph.optimize(number_of_steps, verbose=verbose,
                            save_result=save_result, output_name=output_name)

    def add_edge(self, vertex0Id, vertex1Id, measure, time_stamp, old_time_stamp=0):
        # print(self.lists)
        if(vertex0Id != vertex1Id):
            for vertexId in [vertex0Id, vertex1Id]:
                [node_type, node_id] = vertexId.split("_")
                if(node_type in self.lists):
                    if(node_id not in self.lists[node_type]):
                        self.lists[node_type][node_id] = [time_stamp]
                    else:
                        if(node_type in self.movable):
                            last_update = self.lists[node_type][node_id][-1]
                            if(abs(time_stamp - last_update) > self.smallest_time_step):
                                self.lists[node_type][node_id].append(
                                    time_stamp)
                else:
                    print("ERROR : could not read node type well")
                    exit(-1)
            vertex0Id_int = self.convert_names_to_int(vertex0Id, time_stamp)
            vertex1Id_int = self.convert_names_to_int(vertex1Id, time_stamp)

            self.graph.add_edge(vertex0Id_int, vertex1Id_int, measure)
        else:
            old_time_created = False
            if(old_time_stamp == 0):
                print(
                    "Should be given an old_time_stamp when receiving odometry transform. Assuming time step is %f seconds" % time_step)
                old_time_stamp = time_stamp - 0.5
                old_time_created = True
            [node_type, node_id] = vertex0Id.split("_")
            if(node_type in self.lists):
                if(node_id not in self.lists[node_type]):
                    self.lists[node_type][node_id] = [time_stamp]

                if(node_type in self.movable):
                    if(old_time_created):
                        self.lists[node_type][node_id].append(
                            old_time_stamp)
                        self.lists[node_type][node_id].sort()
                    if(old_time_stamp not in self.lists[node_type][node_id]):
                        print("not in!! %f " % old_time_stamp)
                    old_stamp_index = self.lists[node_type][node_id].index(
                        old_time_stamp)
                    self.lists[node_type][node_id].append(
                        time_stamp)
                    self.interpolate(
                        old_stamp_index, node_type, node_id, measure, vertex0Id)
                else:
                    print(
                        "Node type should be movable if given odometry transform")
                    exit(-1)
            else:
                print("ERROR : could not read node type well")
                exit(-1)

    def get_all_poses(self):
        result_dict = {}
        for mytype, listdict in self.lists.iteritems():
            result_dict[mytype] = {}
            for node_id, time_stamp_list in listdict.iteritems():
                last_time_stamp = time_stamp_list[-1]
                vertex_id = "%s_%s" % (mytype, node_id)
                result_dict[mytype][node_id] = self.graph.vertex_pose(
                    self.convert_names_to_int(vertex_id, last_time_stamp))
        # print(result_dict)
        return result_dict
