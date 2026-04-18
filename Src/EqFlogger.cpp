#include "EqFlogger.hpp"

#include <iomanip>

namespace {
    constexpr const char* kAnsiLightBlue = "\033[94m";
    constexpr const char* kAnsiGreen = "\033[32m";
    constexpr const char* kAnsiPink = "\033[95m";
    constexpr const char* kAnsiReset = "\033[0m";

    template <typename Derived>
    void printVec3Line(std::ostream& os, const char* label, const Eigen::MatrixBase<Derived>& value)
    {
        os << label << "=[" << value(0) << ", " << value(1) << ", " << value(2) << "]\n";
    }

    template <typename Derived>
    void printMat3Block(std::ostream& os, const char* label, const Eigen::MatrixBase<Derived>& value)
    {
        os << label << "=\n"
        << "  [" << value(0, 0) << ", " << value(0, 1) << ", " << value(0, 2) << "]\n"
        << "  [" << value(1, 0) << ", " << value(1, 1) << ", " << value(1, 2) << "]\n"
        << "  [" << value(2, 0) << ", " << value(2, 1) << ", " << value(2, 2) << "]\n";
    }
}

void printMeasurements(const EqFparserResult& result)
{
    std::cout << std::fixed << std::setprecision(6)
              << kAnsiLightBlue
              << "Measurements\n";
    printVec3Line(std::cout, "  result.magData", result.magData);
    printVec3Line(std::cout, "  result.gnssData", result.gnssData);
    std::cout << "  result.baroData=" << result.baroData << '\n'
              << kAnsiReset;
}

void printINSEstimate(const vectornavData& data)
{
    const Vec3 positionNed = setEcefReference(data)
        ? ecefToNed(data.InsPosEcef.cast<double>())
        : Vec3::Constant(std::numeric_limits<double>::quiet_NaN());

    std::cout << std::fixed << std::setprecision(6)
              << kAnsiGreen
              << "VN-200 Estimate\n";
    printMat3Block(std::cout, "  DCM^T", data.Dcm.transpose().cast<double>());
    printVec3Line(std::cout, "  positionNED", positionNed);
    printVec3Line(std::cout, "  velNED", data.InsVelNed.cast<double>());
    printVec3Line(std::cout, "  Accel", data.Accel.cast<double>());
    std::cout << "  PosU=" << data.InsPosU << '\n'
              << "  VelU=" << data.InsVelU << '\n'
              << kAnsiReset;
}

void printEqFEstimate(const EqFOutput& output, const vectornavData& data)
{
    (void)data;

    const Vec9 bHat = -output.Xhat.pose.invAdjoint() * SE23::vee(output.Xhat.bias);
    const Vec3 accelMinusBiasV =  data.UncompAccel.cast<double>() - bHat.segment<3>(3);
    const double velTraceSqrt =
        std::sqrt(std::max(0.0, output.Sigma.block<3, 3>(3, 3).trace()));
    const double posTraceSqrt =
        std::sqrt(std::max(0.0, output.Sigma.block<3, 3>(6, 6).trace()));

    std::cout << std::fixed << std::setprecision(6)
              << kAnsiPink
              << "EqF Estimate\n";
    printMat3Block(std::cout, "  pose.R()", output.Xhat.pose.R());
    printVec3Line(std::cout, "  pose.p()", output.Xhat.pose.p());
    printVec3Line(std::cout, "  pose.v()", output.Xhat.pose.v());
    printVec3Line(std::cout, "  accelMinusBiasV", accelMinusBiasV);
    std::cout << "  sqrt(trace(Sigma(3-5)))=" << velTraceSqrt << '\n'
              << "  sqrt(trace(Sigma(6-8)))=" << posTraceSqrt << '\n'
              << kAnsiReset;
}
