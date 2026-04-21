#include "EqFparser.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

constexpr double kNsToS = 1.0e-9;
constexpr double kKelvinOffset = 273.15;
constexpr double kRd = 287.05;
constexpr double kG = 9.80665;
constexpr double kTwoPi = 6.28318530717958647692;

// ---------------- Baro reference ----------------
double gRefPressurekPa = 0.0;   // first valid pressure
double gRefTempK = 0.0;         // first valid temperature
bool gBaroInitialized = false;

bool setBaroReference(double pressurekPa, double tempK)
{
    if (gBaroInitialized)
    {
        return true;
    }

    if (!(pressurekPa > 0.0 && tempK > 0.0 &&
          std::isfinite(pressurekPa) && std::isfinite(tempK)))
    {
        return false;
    }

    gRefPressurekPa = pressurekPa;
    gRefTempK = tempK;
    gBaroInitialized = true;
    return true;
}

double pressureToAltitude(double pressurekPa, double tempK)
{
    const double avgTempK = 0.5 * (gRefTempK + tempK);
    if (gRefPressurekPa <= 0.0 || avgTempK <= 0.0)
    {
        return 0.0;
    }

    return -(kRd * avgTempK / kG) * std::log(gRefPressurekPa / pressurekPa);
}

// ---------------- GNSS reference ----------------
Vec3 gRefEcef = Vec3::Zero();   // first valid 3D-fix ECEF position
Mat3 gRen = Mat3::Identity();   // ECEF -> NED rotation at the reference point
bool gNedInitialized = false;

// ---------------- Mag reference ----------------
const Vec3 gRefMagneticField = [] {
    Vec3 magneticField = Vec3::Zero();
    magneticField << 0.147066, -0.082766, 0.985658;
    return magneticField;
}();

Mat3 skewSymmetric(const Vec3& value)
{
    Mat3 skew = Mat3::Zero();
    skew <<
        0.0, -value(2),  value(1),
        value(2), 0.0, -value(0),
        -value(1), value(0), 0.0;
    return skew;
}

Mat3 fromTwoVectorsRotation(const Vec3& from, const Vec3& to)
{
    if (!from.allFinite() || !to.allFinite() || from.norm() <= 0.0 || to.norm() <= 0.0)
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        return Mat3::Constant(nan);
    }

    const Vec3 fromUnit = from.normalized();
    const Vec3 toUnit = to.normalized();
    std::cout << "fromTwoVectorsRotation normalized vector=["
              << toUnit(0) << ", "
              << toUnit(1) << ", "
              << toUnit(2) << "]\n";
    const double cosine = std::clamp(fromUnit.dot(toUnit), -1.0, 1.0);

    if (cosine > 1.0 - 1.0e-12)
    {
        return Mat3::Identity();
    }

    if (cosine < -1.0 + 1.0e-12)
    {
        return Eigen::AngleAxisd(0.5 * kTwoPi, fromUnit.unitOrthogonal()).toRotationMatrix();
    }

    const Vec3 cross = fromUnit.cross(toUnit);
    const Mat3 crossSkew = skewSymmetric(cross);
    return (Mat3::Identity() + crossSkew + (crossSkew * crossSkew) / (1.0 + cosine)).transpose();
}

bool calculateEcefToNedRotation(const Vec3& ecefRef, Mat3& Ren)
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

bool setEcefReference(const vectornavData& data)
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
    if (!(std::isfinite(ecef(0)) && std::isfinite(ecef(1)) && std::isfinite(ecef(2))) ||
        ecef.norm() <= 1.0)
    {
        return false;
    }

    gRefEcef = ecef;
    gNedInitialized = calculateEcefToNedRotation(gRefEcef, gRen);
    return gNedInitialized;
}

bool hasEcefReference()
{
    return gNedInitialized;
}

Vec3 ecefToNed(const Vec3& ecef)
{
    if (!hasEcefReference() ||
        !(std::isfinite(ecef(0)) && std::isfinite(ecef(1)) && std::isfinite(ecef(2))) ||
        ecef.norm() <= 1.0)
    {
        return Vec3::Constant(std::numeric_limits<double>::quiet_NaN());
    }

    return gRen * (ecef - gRefEcef);
}

// ----------------- Parser ----------------

EqFparserResult EqFparser(const vectornavData& data)
{
    EqFparserResult result{};
    const double nan = std::numeric_limits<double>::quiet_NaN();

    // ---------------- timestamp ----------------
    result.time = static_cast<double>(data.TimeStartup) * kNsToS;
    result.hasGnssMeasurement = false;

    // ---------------- IMU: use as is ----------------
    result.gyroData = data.UncompGyro.cast<double>();
    result.accData = data.UncompAccel.cast<double>();

    // ---------------- Mag: convert field direction into attitude estimate ----------------
    result.magData = Mat3::Constant(nan);

    const Vec3 mag = data.Mag.cast<double>();
    if (mag.allFinite() && mag.squaredNorm() > 0.0)
    {
        result.magData = fromTwoVectorsRotation(gRefMagneticField, mag);
    }

    // ---------------- Baro: calculate altitude in NED ----------------
    const double pressurekPa = static_cast<double>(data.Pres);
    const double tempK = static_cast<double>(data.Temp) + kKelvinOffset;
    result.baroData = 0.0;

    if (pressurekPa > 0.0 && tempK > 0.0 &&
        std::isfinite(pressurekPa) && std::isfinite(tempK) &&
        setBaroReference(pressurekPa, tempK))
    {
        result.baroData = pressureToAltitude(pressurekPa, tempK);
    }

    // ---------------- GNSS: convert ECEF to NED ----------------
    result.gnssPosData = Vec3::Constant(nan);
    result.gnssVelData = Vec3::Constant(nan);

    bool hasGnssPos = false;
    bool hasGnssVel = false;

    if (setEcefReference(data))
    {
        const Vec3 ned = ecefToNed(data.GnssPosEcef.cast<double>());
        if (std::isfinite(ned(0)) && std::isfinite(ned(1)) && std::isfinite(ned(2)))
        {
            result.gnssPosData = ned;
            hasGnssPos = true;
        }
    }

    const Vec3 gnssVel = data.GnssVelNed.cast<double>();
    if (gpsFixHasPosition(data.Fix) &&
        std::isfinite(gnssVel(0)) && std::isfinite(gnssVel(1)) && std::isfinite(gnssVel(2)))
    {
        result.gnssVelData = gnssVel;
        hasGnssVel = true;
    }

    result.hasGnssMeasurement = hasGnssPos && hasGnssVel;

    return result;
}
