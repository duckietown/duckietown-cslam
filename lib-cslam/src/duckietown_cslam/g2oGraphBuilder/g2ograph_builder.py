import numpy as np
import g2o
import geometry as g


class g2oGraphBuilder():

    def __init__(self):
        self.optimizer = g2o.SparseOptimizer()
        self.solver = g2o.BlockSolverX(g2o.LinearSolverDenseX())
        self.algorithm = g2o.OptimizationAlgorithmLevenberg(self.solver)
        self.optimizer.set_algorithm(self.algorithm)
        self.already_initialized = False
        self.last_lost = 0
        self.set_of_new_edges = set()
        self.set_of_new_vertex = set()

    def add_vertex(self, vertex_id, vertexPose, fixed=False):
        # vertexPose has to be Isometry3D
        vc = g2o.VertexSE3()
        vc.set_id(vertex_id)
        vc.set_estimate(vertexPose)

        if fixed:
            vc.set_fixed(True)
        self.optimizer.add_vertex(vc)
        self.set_of_new_vertex.add(vc)

    def add_edge(self, vertex0Id, vertex1Id, measure, measure_information=None):
        '''
        Helper function to add an edge
        vertex 0 and 1 are valid IDs of vertex inside optimizer.
        measure is a Isometry3d that map the transform from vertex0 to vertex1
        '''
        # print(optimizer.vertices())
        if vertex0Id in self.optimizer.vertices():
            if vertex1Id not in self.optimizer.vertices():
                vc1 = g2o.VertexSE3()
                vc1.set_id(vertex1Id)
                vc1.set_estimate(self.optimizer.vertex(
                    vertex0Id).estimate() * measure)
                vc1.set_fixed(False)
                self.optimizer.add_vertex(vc1)
                self.set_of_new_vertex.add(vc1)

            edge = g2o.EdgeSE3()
            edge.set_vertex(0, self.optimizer.vertex(vertex0Id))
            edge.set_vertex(1, self.optimizer.vertex(vertex1Id))
            edge.set_measurement(measure)
            # r = abs(g.rotation_from_axis_angle(
            #     np.array([1, 0, 0]), np.deg2rad(2)))
            # ligne1 = np.concatenate((r, r), axis=1)
            # ligne2 = np.concatenate((r, r), axis=1)
            # r_final = np.concatenate((ligne1, ligne2), axis=0)

            # edge.set_information(r_final)
            if(measure_information):
                edge.set_information(measure_information)
            finished = self.optimizer.add_edge(edge)
            self.set_of_new_edges.add(edge)

            if(not finished):
                print("Adding edge in g2o is not finished")

        else:
            if vertex1Id in self.optimizer.vertices():
                vc0 = g2o.VertexSE3()
                vc0.set_id(vertex0Id)
                vc0.set_estimate(self.optimizer.vertex(
                    vertex1Id).estimate() * measure.inverse())
                vc0.set_fixed(False)
                self.optimizer.add_vertex(vc0)
                self.set_of_new_vertex.add(vc0)

                edge = g2o.EdgeSE3()
                edge.set_vertex(0, self.optimizer.vertex(vertex0Id))
                edge.set_vertex(1, self.optimizer.vertex(vertex1Id))
                edge.set_measurement(measure)
                if(measure_information):
                    edge.set_information(measure_information)
                # edge.set_information(np.eye(6) * 2)
                finished = self.optimizer.add_edge(edge)
                self.set_of_new_edges.add(edge)

            else:
                vc0 = g2o.VertexSE3()
                vc0.set_id(vertex0Id)
                vc0.set_fixed(False)
                self.optimizer.add_vertex(vc0)
                self.set_of_new_vertex.add(vc0)

                self.add_edge(vertex0Id, vertex1Id, measure)

    def vertices_and_edges(self):
        return (self.optimizer.vertices(), self.optimizer.edges())

    def vertex_pose(self, vertexId):
        if(vertexId not in self.optimizer.vertices()):
            print("Vertex %i is not in the g2o graph" % vertexId)
            if(self.last_lost != 0 and self.last_lost in self.optimizer.vertices()):
                print("Vertex %i wasn't but is now in g2o graph" %
                      self.last_lost)
            elif(self.last_lost != 0):
                print("Vertex %i wasn't and still isn't in g2o graph" %
                      self.last_lost)
            self.last_lost = vertexId
        return (self.optimizer.vertex(vertexId).estimate())

    def get_transform(self, vertex1, vertex2):
        if(vertex1 not in self.optimizer.vertices() or vertex2 not in self.optimizer.vertices()):
            print("Requesting transform between non existant vertices")
            return 0
        vc1 = self.optimizer.vertex(vertex1).estimate()
        vc2 = self.optimizer.vertex(vertex2).estimate()
        transfom = vc1.inverse() * vc2
        return transfom

    def remove_vertex(self, vertexId):
        self.optimizer.remove_vertex(vertexId)

        # optimizing
    def optimize(self, number_of_steps, verbose=True, save_result=True, output_name="output.g2o"):
        self.optimizer.set_verbose(verbose)
        if(not self.already_initialized):

            self.optimizer.initialize_optimization()
            # self.set_of_new_edges = set()
            self.already_initialized = True
        else:
            self.optimizer.update_initialization(
                self.set_of_new_vertex, self.set_of_new_edges)
            self.set_of_new_edges = set()
            self.set_of_new_vertex = set()
            # self.optimizer.compute_initial_guess()
        self.optimizer.compute_active_errors()
        if(verbose):
            print('Optimization:')
            print('Initial chi2 = %f' % self.optimizer.chi2())
        self.optimizer.optimize(number_of_steps)
        if(save_result):
            self.optimizer.save(output_name)
        batch_stat = self.optimizer.batch_statistics
        # print(batch_stat)
