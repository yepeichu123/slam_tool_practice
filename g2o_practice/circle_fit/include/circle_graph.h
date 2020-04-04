#ifndef CIRCLE_GRAPH_H
#define CIRCLE_GRAPH_H

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>

// vertex
class VertexCircle: public g2o::BaseVertex<3, Eigen::Vector3d> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        VertexCircle();

        virtual bool read(std::istream&);

        virtual bool write(std::ostream&) const;

        virtual void setToOriginImpl();

        virtual void oplusImpl(const double* update);
};

// edge
// 1 means error dimension
// Eigen::Vector2d means input data type
// VertexCircle means the data type we need to estimate
class EdgeOnCircle: public g2o::BaseUnaryEdge<1, Eigen::Vector2d, VertexCircle> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeOnCircle();

        virtual bool read(std::istream&);

        virtual bool write(std::ostream&) const;

        void computeError();
};

#endif // CIRCLE_GRAPH_H