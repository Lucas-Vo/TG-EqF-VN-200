#pragma once

#include "VectorNav.hpp"
#include "linalgtypes.hpp"

typedef struct {
    Vec3 magData; // attitude with |m| = 1
    double baroData; // negative altitude in meter
    Vec3 gyroData; // rad/s
    Vec3 accData; // ms⁻²
    Vec3 gnssPosData; // m NED
    Vec3 gnssVelData; // ms⁻1 NED
    double time; // s
    bool hasGnssMeasurement; // true when both position and velocity are valid
}EqFparserResult;

EqFparserResult EqFparser(const vectornavData& data);

bool setEcefReference(const vectornavData& data);
Vec3 ecefToNed(const Vec3& ecef);
