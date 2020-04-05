// c++
#include <iostream>
// Eigen
#include <Eigen/Core>
// g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/icp/types_icp.h>

using namespace std;
using namespace Eigen;
using namespace g2o;

class Sample {
    static default_random_engine gen_real;
    static default_random_engine gen_int;
    public:
        static int uniform(int from, int to);
        static double uniform();
        static double gaussian(double sigma);
};

default_random_engine Sample::gen_real;
default_random_engine Sample::gen_int;

int Sample::uniform(int from, int to) {
    uniform_int_distribution<int> unif(from, to);
    int sam = unif(gen_int);
    return sam;
}

double Sample::uniform() {
    uniform_real_distribution<double> unif(0.0, 1.0);
    double sam = unif(gen_real);
    return sam;
}

double Sample::gaussian(double sigma) {
    normal_distribution<double> gauss(0.0, sigma);
    double sam = gauss(gen_real);
    return sam;
}

int main(int argc, char** argv) {

    // noise in position
    double euc_noise = 0.01;
    // points number
    int numPoints = 1000;

    // optimization
    SparseOptimizer optimizer;
    optimizer.setVerbose(false);

    // solver
    OptimizationAlgorithmLevenberg* solver = new OptimizationAlgorithmLevenberg(
        make_unique<BlockSolverX>(
            make_unique<LinearSolverDense<BlockSolverX::PoseMatrixType> >()
        )
    );
    optimizer.setAlgorithm(solver);

    vector<Vector3d> true_points;
    for (int i = 0; i < numPoints; ++i) {
        true_points.push_back(Vector3d(
            (Sample::uniform()-0.5)*3,
            Sample::uniform()-0.5,
            Sample::uniform()+10
        ));
    }

    // set up two pose 
    int vertex_id = 0;
    for (int i = 0; i < 2; ++i) {
        Vector3d t(0, 0,  i);
        Quaterniond q;
        q.setIdentity();

        Isometry3d cam;
        cam = q;
        cam.translation() = t;

        VertexSE3* vc = new VertexSE3();
        vc->setEstimate(cam);
        vc->setId(i);

        if (i == 0) {
            vc->setFixed(true);
        }

        optimizer.addVertex(vc);
    }

    // set points matching 
    for (int i = 0; i < true_points.size(); ++i) {
        // get two camera pose 
        VertexSE3* vp0 = dynamic_cast<VertexSE3*>(optimizer.vertices().find(0)->second);
        VertexSE3* vp1 = dynamic_cast<VertexSE3*>(optimizer.vertices().find(1)->second);

        // calculate the relative points
        Vector3d pt0, pt1;
        pt0 = vp0->estimate().inverse() * true_points[i];
        pt1 = vp1->estimate().inverse() * true_points[i];

        // add point noise 
        pt0 += Vector3d(Sample::gaussian(euc_noise),
                        Sample::gaussian(euc_noise),
                        Sample::gaussian(euc_noise));
        pt1 += Vector3d(Sample::gaussian(euc_noise),
                        Sample::gaussian(euc_noise),
                        Sample::gaussian(euc_noise));
        
        Vector3d nm0, nm1;
        nm0 << 0, i, 1;
        nm1 << 0, i, 1;
        nm0.normalize();
        nm1.normalize();

        Edge_V_V_GICP* e = new Edge_V_V_GICP();
        e->setVertex(0, vp0);
        e->setVertex(1, vp1);

        EdgeGICP meas;
        meas.pos0 = pt0;
        meas.pos1 = pt1;
        meas.normal0 = nm0;
        meas.normal1 = nm1;
        e->setMeasurement(meas);
        meas = e->measurement();
        e->information() = meas.prec0(0.01);
        optimizer.addEdge(e);
    }

    VertexSE3* vc = dynamic_cast<VertexSE3*>(optimizer.vertices().find(1)->second);
    Eigen::Isometry3d cam = vc->estimate();
    cam.translation() = Vector3d(0, 0, 0.2);
    vc->setEstimate(cam);

    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    cout << "Initial chi2 = " << FIXED(optimizer.chi2()) << endl;
    optimizer.setVerbose(true);
    optimizer.optimize(10);

    cout << endl << "Second vertex should be near 0, 0, 1." << endl;
    cout << "first pose: " <<  dynamic_cast<VertexSE3*>(optimizer.vertices().find(0)->second)->estimate().translation().transpose() << endl;
    cout << "second pose: " << dynamic_cast<VertexSE3*>(optimizer.vertices().find(1)->second)->estimate().translation().transpose() << endl;

    return 0;
}