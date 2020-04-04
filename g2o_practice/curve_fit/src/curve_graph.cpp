#include "curve_graph.h"
#include <iostream>
#include <cmath>

// vertex
VertexParams::VertexParams() {

}

bool VertexParams::read(std::istream&) {
    std::cerr << "read function is not implemented yet." << std::endl;
    return false;
}

bool VertexParams::write(std::ostream&) const {
    std::cerr << "write function is not implemented yet." << std::endl;
    return false;
} 

void VertexParams::setToOriginImpl() {
    std::cerr << "setToOriginImpl function is not implemented yet." << std::endl;
}

void VertexParams::oplusImpl(const double* update) {
    Eigen::Vector3d::ConstMapType v(update);
    _estimate += v;
}

// edge
EdgePointOnCureve::EdgePointOnCureve() {

}

bool EdgePointOnCureve::read(std::istream&) {
    std::cerr << "read function is not implemented yet." << std::endl;
    return false;
}

bool EdgePointOnCureve::write(std::ostream&) const {
    std::cerr << "write function is not implemented yet." << std::endl;
    return false;
}

void EdgePointOnCureve::computeError() {
    const VertexParams* params = static_cast<const VertexParams*>(vertex(0));
    const double& a = params->estimate()(0);
    const double& b = params->estimate()(1);
    const double& lambda = params->estimate()(2);
    double fval = a * std::exp(-lambda * measurement()(0)) + b;
    _error(0) = fval - measurement()(1);
}