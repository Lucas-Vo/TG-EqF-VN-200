#include "EqFlogger.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <unordered_set>

namespace {
    constexpr const char* kAnsiRed = "\033[91m";
    constexpr const char* kAnsiOrange = "\033[38;5;208m";
    constexpr const char* kAnsiGreen = "\033[32m";
    constexpr const char* kAnsiPurple = "\033[94m";
    constexpr const char* kAnsiReset = "\033[0m";
    constexpr const char* kEstimateCsvHeader =
        "timestamp,R00,R01,R02,R11,R12,R22,"
        "pos_N,pos_E,pos_D,vel_N,vel_E,vel_D,accel_x,accel_y,accel_z,pos_uncert,vel_uncert";
    constexpr double kNsToS = 1.0e-9;

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

    void writeUpperTriangularRotation(std::ostream& os, const Mat3& rotation)
    {
        os << rotation(0, 0) << ','
           << rotation(0, 1) << ','
           << rotation(0, 2) << ','
           << rotation(1, 1) << ','
           << rotation(1, 2) << ','
           << rotation(2, 2);
    }

    std::ofstream openCsvStream(const std::filesystem::path& path)
    {
        std::filesystem::create_directories(path.parent_path());
        static std::unordered_set<std::string> initializedPaths;
        const std::string key = path.string();
        const bool firstWriteThisRun = initializedPaths.insert(key).second;

        std::ofstream stream(
            path,
            firstWriteThisRun ? (std::ios::out | std::ios::trunc) : (std::ios::out | std::ios::app));
        if (stream && firstWriteThisRun)
        {
            stream << kEstimateCsvHeader << '\n';
        }
        return stream;
    }
}

void printRaw(const EqFparserResult& result)
{
    std::cout << std::fixed << std::setprecision(6)
              << kAnsiOrange
              << "Raw\n";
    printMat3Block(std::cout, "  magData", result.magData);
    std::cout << "  baroData=" << result.baroData << '\n'
              << kAnsiReset;
}

void printMeasurements(const EqFparserResult& result, const vectornavData& data)
{
    std::cout << std::fixed << std::setprecision(6)
              << kAnsiGreen
              << "Measurements\n";
    printMat3Block(std::cout, "  magData", result.magData);
    printVec3Line(std::cout, "  positionNED", result.gnssPosData);
    printVec3Line(std::cout, "  velNED", result.gnssVelData);
    printVec3Line(std::cout, "  Accel", data.UncompAccel.cast<double>());
    std::cout << kAnsiReset;
}

void printVNEstimate(const vectornavData& data)
{
    const Vec3 positionNed = hasEcefReference()
        ? ecefToNed(data.InsPosEcef.cast<double>())
        : Vec3::Constant(std::numeric_limits<double>::quiet_NaN());

    std::cout << std::fixed << std::setprecision(6)
              << kAnsiRed
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
              << kAnsiPurple
              << "EqF Estimate\n";
    printMat3Block(std::cout, "  pose.R()", output.Xhat.pose.R());
    printVec3Line(std::cout, "  pose.p()", output.Xhat.pose.p());
    printVec3Line(std::cout, "  pose.v()", output.Xhat.pose.v());
    printVec3Line(std::cout, "  accelMinusBiasV", accelMinusBiasV);
    std::cout << "  sqrt(trace(Sigma(3-5)))=" << velTraceSqrt << '\n'
              << "  sqrt(trace(Sigma(6-8)))=" << posTraceSqrt << '\n'
              << kAnsiReset;
}

void logMeasurements(const EqFparserResult& result, const vectornavData& data)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();

    std::ofstream stream = openCsvStream("log/Measurements.csv");
    if (!stream)
    {
        return;
    }

    stream << std::fixed << std::setprecision(3)
           << result.time << ',';
    writeUpperTriangularRotation(stream, result.magData);
    stream << ','
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
    const Vec3 positionNed = hasEcefReference()
        ? ecefToNed(data.InsPosEcef.cast<double>())
        : Vec3::Constant(std::numeric_limits<double>::quiet_NaN());

    std::ofstream stream = openCsvStream("log/VNEstimate.csv");
    if (!stream)
    {
        return;
    }

    stream << std::fixed << std::setprecision(3)
           << static_cast<double>(data.TimeStartup) * kNsToS << ',';
    writeUpperTriangularRotation(stream, rotation);
    stream << ','
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
    const Mat3 rotation = output.Xhat.pose.R();

    std::ofstream stream = openCsvStream("log/TGEqFEstimate.csv");
    if (!stream)
    {
        return;
    }

    stream << std::fixed << std::setprecision(3)
           << static_cast<double>(data.TimeStartup) * kNsToS << ',';
    writeUpperTriangularRotation(stream, rotation);
    stream << ','
           << output.Xhat.pose.p()(0) << ',' << output.Xhat.pose.p()(1) << ',' << output.Xhat.pose.p()(2) << ','
           << output.Xhat.pose.v()(0) << ',' << output.Xhat.pose.v()(1) << ',' << output.Xhat.pose.v()(2) << ','
           << accelMinusBiasV(0) << ',' << accelMinusBiasV(1) << ',' << accelMinusBiasV(2) << ','
           << posTraceSqrt << ','
           << velTraceSqrt << '\n';
}
