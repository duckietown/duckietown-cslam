import numpy as np
import g2o


class g2oGraphBuilder():

    def __init__(self):
        self.optimizer = g2o.SparseOptimizer()
        self.solver = g2o.BlockSolverX(g2o.LinearSolverDenseX())
        self.algorithm = g2o.OptimizationAlgorithmLevenberg(self.solver)
        self.optimizer.set_algorithm(self.algorithm)

    def add_vertex(self, vertex_id, vertexPose, fixed=False):
        # vertexPose has to be Isometry3D
        vc = g2o.VertexSE3()
        vc.set_id(vertex_id)
        vc.set_estimate(vertexPose)

        if fixed:
            vc.set_fixed(True)
        self.optimizer.add_vertex(vc)

    def add_edge(self, vertex0Id, vertex1Id, measure):
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
                self.optimizer.add_vertex(vc1)

            edge = g2o.EdgeSE3()
            edge.set_vertex(0, self.optimizer.vertex(vertex0Id))
            edge.set_vertex(1, self.optimizer.vertex(vertex1Id))
            edge.set_measurement(measure)

            self.optimizer.add_edge(edge)
        else:
            if vertex1Id in self.optimizer.vertices():
                self.add_edge(vertex1Id,
                              vertex0Id, measure.inverse())
            else:
                vc0 = g2o.VertexSE3()
                vc0.set_id(vertex0Id)
                self.optimizer.add_vertex(vc0)
                self.add_edge(vertex0Id, vertex1Id, measure)

    def vertices_and_edges(self):
        return (self.optimizer.vertices(), self.optimizer.edges())

    def remove_vertex(self, vertexId):
        self.optimizer.remove_vertex(vertexId)

        # optimizing
    def optimize(self, number_of_steps, verbose=True, save_result=True, output_name="output.g2o"):

        self.optimizer.initialize_optimization()
        self.optimizer.compute_active_errors()
        if(verbose):
            print('Optimization:')
            print('Initial chi2 =', self.optimizer.chi2())

        self.optimizer.set_verbose(verbose)
        self.optimizer.optimize(100)
        if(save_result):
            self.optimizer.save(output_name)
