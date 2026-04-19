#include "EqFlogger.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>

namespace {
    constexpr const char* kAnsiLightBlue = "\033[94m";
    constexpr const char* kAnsiGreen = "\033[32m";
    constexpr const char* kAnsiPink = "\033[95m";
    constexpr const char* kAnsiReset = "\033[0m";
    constexpr const char* kEstimateCsvHeader =
        "timestamp,angle_axis_x,angle_axis_y,angle_axis_z,angle_axis_cos,angle_axis_sin,"
        "pos_N,pos_E,pos_D,vel_N,vel_E,vel_D,accel_x,accel_y,accel_z,pos_uncert,vel_uncert";
    constexpr double kNsToS = 1.0e-9;
    constexpr double kTwoPi = 6.28318530717958647692;

    struct AngleAxisSample
    {
        Vec3 axis = Vec3::UnitX();
        double angle = 0.0;
    };

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

    AngleAxisSample toAngleAxis(const Mat3& rotation)
    {
        if (!rotation.allFinite())
        {
            const double nan = std::numeric_limits<double>::quiet_NaN();
            return {Vec3::Constant(nan), nan};
        }

        const Eigen::AngleAxisd angleAxis(rotation);
        AngleAxisSample sample{};
        sample.axis = angleAxis.axis();
        sample.angle = angleAxis.angle();

        if (!sample.axis.allFinite() || !std::isfinite(sample.angle))
        {
            const double nan = std::numeric_limits<double>::quiet_NaN();
            return {Vec3::Constant(nan), nan};
        }

        sample.angle = std::fmod(sample.angle, kTwoPi);
        if (sample.angle < 0.0)
        {
            sample.angle += kTwoPi;
        }
        sample.angle = std::clamp(sample.angle, 0.0, std::nextafter(kTwoPi, 0.0));

        if (sample.angle < 1.0e-12)
        {
            sample.axis = Vec3::UnitX();
            sample.angle = 0.0;
        }

        return sample;
    }

    AngleAxisSample fromTwoVectorsAngleAxis(const Vec3& from, const Vec3& to)
    {
        if (!from.allFinite() || !to.allFinite() || from.norm() <= 0.0 || to.norm() <= 0.0)
        {
            const double nan = std::numeric_limits<double>::quiet_NaN();
            return {Vec3::Constant(nan), nan};
        }

        const Eigen::Quaterniond rotation =
            Eigen::Quaterniond::FromTwoVectors(from.normalized(), to.normalized());
        return toAngleAxis(rotation.toRotationMatrix());
    }

    void printAngleAxis(std::ostream& os, const char* label, const AngleAxisSample& value)
    {
        os << label
           << "=[" << value.axis(0) << ", " << value.axis(1) << ", " << value.axis(2) << "]"
           << ", cos=" << std::cos(value.angle)
           << ", sin=" << std::sin(value.angle) << '\n';
    }

    std::ofstream openCsvStream(const std::filesystem::path& path)
    {
        std::filesystem::create_directories(path.parent_path());
        const bool needsHeader =
            !std::filesystem::exists(path) || std::filesystem::file_size(path) == 0;

        std::ofstream stream(path, std::ios::app);
        if (stream && needsHeader)
        {
            stream << kEstimateCsvHeader << '\n';
        }
        return stream;
    }
}

void printMeasurements(const EqFparserResult& result, const vectornavData& data, const Vec3& magneticField)
{
    const AngleAxisSample angleAxis = fromTwoVectorsAngleAxis(magneticField, result.magData);

    std::cout << std::fixed << std::setprecision(6)
              << kAnsiLightBlue
              << "Measurements\n";
    printAngleAxis(std::cout, "  angleAxis", angleAxis);
    printVec3Line(std::cout, "  positionNED", result.gnssPosData);
    printVec3Line(std::cout, "  velNED", result.gnssVelData);
    printVec3Line(std::cout, "  Accel", data.UncompAccel.cast<double>());
    std::cout << kAnsiReset;
}

void printVNEstimate(const vectornavData& data)
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

void printTGEqFEstimate(const EqFOutput& output, const vectornavData& data)
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

void logMeasurements(const EqFparserResult& result, const vectornavData& data, const Vec3& magneticField)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const AngleAxisSample angleAxis = fromTwoVectorsAngleAxis(magneticField, result.magData);

    std::ofstream stream = openCsvStream("log/Measurements.csv");
    if (!stream)
    {
        return;
    }

    stream << std::fixed << std::setprecision(3)
           << result.time << ','
           << angleAxis.axis(0) << ',' << angleAxis.axis(1) << ',' << angleAxis.axis(2) << ','
           << std::cos(angleAxis.angle) << ',' << std::sin(angleAxis.angle) << ','
           << result.gnssPosData(0) << ',' << result.gnssPosData(1) << ',' << result.gnssPosData(2) << ','
           << result.gnssVelData(0) << ',' << result.gnssVelData(1) << ',' << result.gnssVelData(2) << ','
           << static_cast<double>(data.UncompAccel(0)) << ','
           << static_cast<double>(data.UncompAccel(1)) << ','
           << static_cast<double>(data.UncompAccel(2)) << ','
           << nan << ','
           << nan << '\n';
}

void logVNEstimate(const vectornavData& data)
{
    const Mat3 rotation = data.Dcm.transpose().cast<double>();
    const AngleAxisSample angleAxis = toAngleAxis(rotation);
    const Vec3 positionNed = setEcefReference(data)
        ? ecefToNed(data.InsPosEcef.cast<double>())
        : Vec3::Constant(std::numeric_limits<double>::quiet_NaN());

    std::ofstream stream = openCsvStream("log/VNEstimate.csv");
    if (!stream)
    {
        return;
    }

    stream << std::fixed << std::setprecision(3)
           << static_cast<double>(data.TimeStartup) * kNsToS << ','
           << angleAxis.axis(0) << ',' << angleAxis.axis(1) << ',' << angleAxis.axis(2) << ','
           << std::cos(angleAxis.angle) << ',' << std::sin(angleAxis.angle) << ','
           << positionNed(0) << ',' << positionNed(1) << ',' << positionNed(2) << ','
           << static_cast<double>(data.InsVelNed(0)) << ','
           << static_cast<double>(data.InsVelNed(1)) << ','
           << static_cast<double>(data.InsVelNed(2)) << ','
           << static_cast<double>(data.Accel(0)) << ','
           << static_cast<double>(data.Accel(1)) << ','
           << static_cast<double>(data.Accel(2)) << ','
           << static_cast<double>(data.InsPosU) << ','
           << static_cast<double>(data.InsVelU) << '\n';
}

void logTGEqFEstimate(const EqFOutput& output, const vectornavData& data)
{
    const Vec9 bHat = -output.Xhat.pose.invAdjoint() * SE23::vee(output.Xhat.bias);
    const Vec3 accelMinusBiasV = data.UncompAccel.cast<double>() - bHat.segment<3>(3);
    const double velTraceSqrt =
        std::sqrt(std::max(0.0, output.Sigma.block<3, 3>(3, 3).trace()));
    const double posTraceSqrt =
        std::sqrt(std::max(0.0, output.Sigma.block<3, 3>(6, 6).trace()));
    const AngleAxisSample angleAxis = toAngleAxis(output.Xhat.pose.R());

    std::ofstream stream = openCsvStream("log/TGEqFEstimate.csv");
    if (!stream)
    {
        return;
    }

    stream << std::fixed << std::setprecision(3)
           << static_cast<double>(data.TimeStartup) * kNsToS << ','
           << angleAxis.axis(0) << ',' << angleAxis.axis(1) << ',' << angleAxis.axis(2) << ','
           << std::cos(angleAxis.angle) << ',' << std::sin(angleAxis.angle) << ','
           << output.Xhat.pose.p()(0) << ',' << output.Xhat.pose.p()(1) << ',' << output.Xhat.pose.p()(2) << ','
           << output.Xhat.pose.v()(0) << ',' << output.Xhat.pose.v()(1) << ',' << output.Xhat.pose.v()(2) << ','
           << accelMinusBiasV(0) << ',' << accelMinusBiasV(1) << ',' << accelMinusBiasV(2) << ','
           << posTraceSqrt << ','
           << velTraceSqrt << '\n';
}
