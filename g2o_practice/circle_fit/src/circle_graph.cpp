#include "circle_graph.h"

// vertex
VertexCircle::VertexCircle() {

}

bool VertexCircle::read(std::istream&) {
    return false;
}

bool VertexCircle::write(std::ostream&) const {
    return false;
}

void VertexCircle::setToOriginImpl() {
    
}

void VertexCircle::oplusImpl(const double* update) {
    Eigen::Vector3d::ConstMapType v(update);
    _estimate += v;
}


// edge
EdgeOnCircle::EdgeOnCircle() {

}

bool EdgeOnCircle::read(std::istream&) {
    return false;
}

bool EdgeOnCircle::write(std::ostream&) const {
    return false;
}

void EdgeOnCircle::computeError() {
    const VertexCircle* circle = static_cast<VertexCircle*>(vertex(0));

    const Eigen::Vector2d& center = circle->estimate().head<2>();
    const double& radius = circle->estimate()(2);

    _error(0) = (measurement() - center).norm() - radius;
}