#include <iostream>
#include "circle_graph.h"
#include <g2o/stuff/sampler.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

using namespace std;

double ComputeErrorofSolution(int numPoints, Eigen::Vector2d* points, const Eigen::Vector3d& circle);

int main(int argc, char** argv) {

    int numPoints = 100;
    int maxIteration = 10;
    bool verbose = true;

    // generate random data
    Eigen::Vector2d center(4.0, 2.0);
    double radius = 2.0;
    Eigen::Vector2d* points = new Eigen::Vector2d[numPoints];

    g2o::Sampler::seedRand();
    for (int i = 0; i < numPoints; ++i) {
        double r = g2o::Sampler::gaussRand(radius, 0.05);
        double angle = g2o::Sampler::uniformRand(0.0, 2.0*M_PI);
        points[i].x() = center.x() + r * cos(angle);
        points[i].y() = center.y() + r * sin(angle);
    }

    // typedefs 
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic> > MyBlockSolver;
    typedef g2o::LinearSolverCSparse<MyBlockSolver::PoseMatrixType> MyLinearSolver;

    // setup the solver 
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(verbose);
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(
        g2o::make_unique<MyBlockSolver>(
            g2o::make_unique<MyLinearSolver>()
        )
    );
    optimizer.setAlgorithm(solver);

    // build the optimization problem given the points 
    VertexCircle* circle = new VertexCircle();
    circle->setId(0);
    circle->setEstimate(
        Eigen::Vector3d(3.0, 3.0, 3.0)
    );
    optimizer.addVertex(circle);

    for (int i = 0; i < numPoints; ++i) {
        EdgeOnCircle* e = new EdgeOnCircle();
        e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        e->setVertex(0, circle);
        e->setMeasurement(points[i]);
        optimizer.addEdge(e);
    }

    // perform the optimization
    optimizer.initializeOptimization();
    optimizer.optimize(maxIteration); 

    if (verbose) {
        cout << endl;
    }

    // print the result
    cout << "Original center is: " << center.transpose() << ", radius is: " << radius << endl;
    cout << "Iterative last squares solution" << endl;
    cout << "center of the cirle: " << circle->estimate().head<2>().transpose() << ", radius of the circle: " << circle->estimate()(2) << endl;
    cout << "Total error is: " << ComputeErrorofSolution(numPoints, points, circle->estimate()) << endl;
    return 0;
}

double ComputeErrorofSolution(int numPoints, Eigen::Vector2d* points, const Eigen::Vector3d& circle) {
    Eigen::Vector2d center = circle.head<2>();
    double radius = circle(2);
    double error = 0;
    for (int i = 0; i < numPoints; ++i) {
        double d = (points[i] - center).norm() - radius;
        error += d*d;
    }
    return error;
}