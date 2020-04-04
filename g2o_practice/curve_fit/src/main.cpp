// Eigen
#include <eigen3/Eigen/Core>
// g2o
#include "curve_graph.h"
#include <g2o/stuff/sampler.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
// c++
#include <iostream>
#include <cmath>

using namespace std;

int main(int argc, char** argv) {

    // generate random data
    int numPoints = 50;
    int maxIterate = 100;
    double a = 2;
    double b = 0.4;
    double lambda = 0.2;
    Eigen::Vector2d* points = new Eigen::Vector2d[numPoints];
    cout << "True value: " << endl;
    for (int i = 0; i < numPoints; ++i) {
        double x = g2o::Sampler::uniformRand(0, 10);
        double y = a * exp(-lambda * x) + b;
        // add gaussian noise 
        y += g2o::Sampler::gaussRand(0, 0.02);
        cout << i << " : [ " << x << ", " << y << " ]" << endl;
        points[i].x() = x;
        points[i].y() = y;
    }

    // typedef
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic> > MyBlockSolver;
    typedef g2o::LinearSolverDense<MyBlockSolver::PoseMatrixType> MyLinearSolver;

    // algorithm
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<MyBlockSolver>(
            g2o::make_unique<MyLinearSolver>()
        )
    );
    
    // sparse optimization
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    // build the optimization problem given the points
    VertexParams* params = new VertexParams();
    params->setId(0);
    params->setEstimate(Eigen::Vector3d(1, 1, 1));
    optimizer.addVertex(params);
    for (int i = 0; i < numPoints; ++i) {
        EdgePointOnCureve* e = new EdgePointOnCureve;
        e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        e->setVertex(0, params);
        e->setMeasurement(points[i]);
        optimizer.addEdge(e);
    }

    // perform the optimization 
    optimizer.initializeOptimization();
    optimizer.optimize(maxIterate);

    // get the optimized result 
    Eigen::Vector3d result = params->estimate();
    cout << "Estimated result:" << endl;
    cout << "a, b, c = [" << result[0] << ", " << result[1] << ", " << result[2] << "]" << endl;


    delete[] points;

    return 0;
}