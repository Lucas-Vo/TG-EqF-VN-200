#include "SE23xse23.hpp"
#include <unsupported/Eigen/MatrixFunctions> // needed for .exp()

// Semidirect-product exponential:
SE23xse23 ExpSE23xse23(const se23xse23 &xi)
{
    SE23xse23 out;

    out.pose = SE23::exp(SE23::vee(xi.pose));
    out.bias = SE23::wedge(SE23::leftJacobian(SE23::vee(xi.pose)) * SE23::vee(xi.bias));
    return out;
}

SE23xse23 MulSE23xse23(const SE23xse23 &X, const SE23xse23 &Y)
{
    SE23xse23 out;
    out.pose = X.pose * Y.pose;

    Vec9 a = SE23::vee(X.bias);
    Vec9 b = SE23::vee(Y.bias);
    Vec9 out_bias = a + X.pose.Adjoint() * b;

    out.bias = SE23::wedge(out_bias);
    return out;
}
