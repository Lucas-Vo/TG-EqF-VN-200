#include "utils.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <ostream>

namespace
{
constexpr double kNsToS = 1.0e-9;
constexpr double kKelvinOffset = 273.15;
constexpr double kRd = 287.05;
constexpr double kG = 9.80665;
constexpr const char* kAnsiGreen = "\033[32m";
constexpr const char* kAnsiPink = "\033[95m";
constexpr const char* kAnsiReset = "\033[0m";

// ---------------- Baro reference ----------------
double gRefPressurekPa = 0.0;
double gRefTempK = 0.0;
bool gBaroInitialized = false;

// ---------------- GNSS / NED reference ----------------
// We define a local NED frame by latching the first valid 3D-fix ECEF position.
Vec3 gRefEcef = Vec3::Zero();
Mat3 gRen = Mat3::Identity();   // ECEF -> NED rotation at the reference point
bool gNedInitialized = false;

// Small helper: build ECEF->NED rotation directly from reference ECEF.
bool computeEcefToNedFromReference(const Vec3& ecefRef, Mat3& Ren)
{
    const double x = ecefRef(0);
    const double y = ecefRef(1);
    const double z = ecefRef(2);

    const double lon = std::atan2(y, x);
    const double rho = std::sqrt(x * x + y * y);

    // Geocentric latitude from ECEF direction.
    // For local tangent-frame construction this is usually good enough.
    const double lat = std::atan2(z, rho);

    const double sLat = std::sin(lat);
    const double cLat = std::cos(lat);
    const double sLon = std::sin(lon);
    const double cLon = std::cos(lon);

    Ren <<
        -sLat * cLon, -sLat * sLon,  cLat,
             -sLon,          cLon,   0.0,
        -cLat * cLon, -cLat * sLon, -sLat;

    return std::isfinite(Ren.sum());
}

bool isFiniteVec3(const Vec3& v)
{
    return std::isfinite(v(0)) && std::isfinite(v(1)) && std::isfinite(v(2));
}

bool ensureNedReference(const vectornavData& data)
{
    if (gNedInitialized)
    {
        return true;
    }

    if (!gpsFixHasPosition(data.Fix))
    {
        return false;
    }

    const Vec3 ecef = data.GnssPosEcef.cast<double>();
    if (!isFiniteVec3(ecef) || ecef.norm() <= 1.0)
    {
        return false;
    }

    gRefEcef = ecef;
    gNedInitialized = computeEcefToNedFromReference(gRefEcef, gRen);
    return gNedInitialized;
}

Vec3 invalidVec3()
{
    return Vec3::Constant(std::numeric_limits<double>::quiet_NaN());
}

Vec3 ecefToNed(const Vec3& ecef)
{
    if (!gNedInitialized || !isFiniteVec3(ecef) || ecef.norm() <= 1.0)
    {
        return invalidVec3();
    }

    return gRen * (ecef - gRefEcef);
}

double sqrtTraceClamped(const Mat3& block)
{
    return std::sqrt(std::max(0.0, block.trace()));
}

template <typename Derived>
void printVec3Line(std::ostream& os,
                   const char* label,
                   const Eigen::MatrixBase<Derived>& value)
{
    os << label << "=[" << value(0) << ", " << value(1) << ", " << value(2) << "]\n";
}

template <typename Derived>
void printMat3Block(std::ostream& os,
                    const char* label,
                    const Eigen::MatrixBase<Derived>& value)
{
    os << label << "=\n"
       << "  [" << value(0, 0) << ", " << value(0, 1) << ", " << value(0, 2) << "]\n"
       << "  [" << value(1, 0) << ", " << value(1, 1) << ", " << value(1, 2) << "]\n"
       << "  [" << value(2, 0) << ", " << value(2, 1) << ", " << value(2, 2) << "]\n";
}
} // namespace

std::ostream& operator<<(std::ostream& os, const EqFparserResult& data)
{
    os << "EqFparserResult{"
       << "time=" << data.time
       << ", baroData=" << data.baroData
       << ", magData=[" << data.magData(0) << ", " << data.magData(1) << ", " << data.magData(2) << "]"
       << ", gyroData=[" << data.gyroData(0) << ", " << data.gyroData(1) << ", " << data.gyroData(2) << "]"
       << ", accData=[" << data.accData(0) << ", " << data.accData(1) << ", " << data.accData(2) << "]"
       << ", gnssData=[" << data.gnssData(0) << ", " << data.gnssData(1) << ", " << data.gnssData(2) << "]"
       << "}";
    return os;
}

EqFparserResult EqFparser(const vectornavData& data)
{
    EqFparserResult result{};

    // Magnetmometer: normalize to unit vector
    Vec3 mag = data.Mag.cast<double>();
    const double magNormSq = mag.squaredNorm();
    if (magNormSq > 0.0)
    {
        mag *= 1.0 / std::sqrt(magNormSq);
    }
    result.magData = mag;

    // Barometer: calculate negative altitude (NED)
    const double pressurekPa = static_cast<double>(data.Pres);
    const double tempK = static_cast<double>(data.Temp) + kKelvinOffset;
    result.baroData = 0.0;

    if (pressurekPa > 0.0 && tempK > 0.0 &&
        std::isfinite(pressurekPa) && std::isfinite(tempK))
    {
        if (!gBaroInitialized)
        {
            gRefPressurekPa = pressurekPa;
            gRefTempK = tempK;
            gBaroInitialized = true;
        }

        const double avgTempK = 0.5 * (gRefTempK + tempK);

        if (gRefPressurekPa > 0.0 && avgTempK > 0.0)
        {
            result.baroData = - (kRd * avgTempK / kG) *
                              std::log(gRefPressurekPa / pressurekPa);
        }
    }

    // IMU: use as is
    result.gyroData = data.UncompGyro.cast<double>();
    result.accData  = data.UncompAccel.cast<double>();

    // GNSS: convert ECEF to NED
    result.gnssData = Vec3::Zero();

    if (ensureNedReference(data))
    {
        const Vec3 ecef = data.GnssPosEcef.cast<double>();
        const Vec3 ned = ecefToNed(ecef);
        if (isFiniteVec3(ned))
        {
            result.gnssData = ned;
        }
    }

    // ---------------- Timestamp ----------------
    result.time = static_cast<double>(data.TimeStartup) * kNsToS;

    return result;
}

void printINSEstimate(const vectornavData& data)
{
    const Vec3 positionNed = ensureNedReference(data)
        ? ecefToNed(data.InsPosEcef.cast<double>())
        : invalidVec3();

    std::cout << std::fixed << std::setprecision(6)
              << kAnsiGreen
              << "INS Estimate\n";
    printMat3Block(std::cout, "  DCM", data.Dcm.cast<double>());
    printVec3Line(std::cout, "  positionNED", positionNed);
    printVec3Line(std::cout, "  velNED", data.InsVelNed.cast<double>());
    printVec3Line(std::cout, "  AccelNed", data.AccelNed.cast<double>());
    std::cout << "  PosU=" << data.InsPosU << '\n'
              << "  VelU=" << data.InsVelU << '\n'
              << kAnsiReset;
}

void printEqFEstimate(const EqFOutput& output, const vectornavData& data)
{
    const Vec3 biasVPlusAccel =
        SE23::vee(output.Xhat.bias).segment<3>(3) + data.UncompAccel.cast<double>();
    const double velTraceSqrt = sqrtTraceClamped(output.Sigma.block<3, 3>(3, 3));
    const double posTraceSqrt = sqrtTraceClamped(output.Sigma.block<3, 3>(6, 6));

    std::cout << std::fixed << std::setprecision(6)
              << kAnsiPink
              << "EqF Estimate\n";
    printMat3Block(std::cout, "  pose.R()", output.Xhat.pose.R());
    printVec3Line(std::cout, "  pose.p()", output.Xhat.pose.p());
    printVec3Line(std::cout, "  pose.v()", output.Xhat.pose.v());
    printVec3Line(std::cout, "  bias.v()+UncompAccel", biasVPlusAccel);
    std::cout << "  sqrt(trace(Sigma(3-5)))=" << velTraceSqrt << '\n'
              << "  sqrt(trace(Sigma(6-8)))=" << posTraceSqrt << '\n'
              << kAnsiReset;
}
