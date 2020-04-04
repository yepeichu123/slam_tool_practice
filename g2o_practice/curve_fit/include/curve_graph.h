#ifndef CURVE_GRAPH_H
#define CURVE_GRAPH_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>

class VertexParams: public g2o::BaseVertex<3, Eigen::Vector3d> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        VertexParams();

        virtual bool read(std::istream&);

        virtual bool write(std::ostream&) const;

        virtual void setToOriginImpl();

        virtual void oplusImpl(const double* update);
};

class EdgePointOnCureve: public g2o::BaseUnaryEdge<1, Eigen::Vector2d, VertexParams> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgePointOnCureve();

        virtual bool read(std::istream&);

        virtual bool write(std::ostream&) const;

        void computeError();
};


#endif // CURVE_GRAPH_H
