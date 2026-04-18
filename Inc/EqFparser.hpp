#pragma once

#include "VectorNav.hpp"
#include "linalgtypes.hpp"

typedef struct {
    Vec3 magData; // attitude with |m| = 1
    double baroData; // negative altitude in meter
    Vec3 gyroData; // rad/s
    Vec3 accData; // ms⁻²
    Vec3 gnssData; // m
    double time; // s
}EqFparserResult;

EqFparserResult EqFparser(const vectornavData& data);

bool setEcefReference(const vectornavData& data);
Vec3 ecefToNed(const Vec3& ecef);

