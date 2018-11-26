import numpy as np
import g2o
import duckietown_cslam.g2oGraphBuilder.g2ograph_builder as g2oBG
import geometry as g
import random


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
        return int(int(items[1]) + 10 * int(time_stamp % 10000))

    def convert_int_to_name(self, i):
        time_stamp = 123456789
        return ["duckie_5", time_stamp]

    def interpolate(self, old_stamp_index, node_type, node_id, measure, vertexId):
        to_interpolate = self.lists[node_type][node_id][old_stamp_index:-1]
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
            if(old_time_stamp == 0):
                print(
                    "Should be given an old_time_stamp when receiving odometry transform")
                exit(-1)
            [node_type, node_id, _] = vertex0Id.split("_")
            if(node_type in self.lists):
                if(node_id not in self.lists[node_type]):
                    print("This is absurd")
                    exit(-1)
                else:
                    if(node_type in self.movable):
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
