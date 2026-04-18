#pragma once

#include <iosfwd>
#include <tuple>

#include "VectorNav.hpp"
#include "linalgtypes.hpp"
#include "SE23xse23.hpp"

typedef struct {
    Vec3 magData; // attitude with |m| = 1
    double baroData; // negative altitude in meter
    Vec3 gyroData; // rad/s
    Vec3 accData; // ms⁻²
    Vec3 gnssData; // m
    double time; // s
}EqFparserResult;

EqFparserResult EqFparser(const vectornavData& data);
std::ostream& operator<<(std::ostream& os, const EqFparserResult& data);

// OUTPUT
typedef struct
{
    SE23xse23 Xhat;
    Mat18 Sigma;
} EqFOutput;

void printINSEstimate(const vectornavData& data);
void printEqFEstimate(const EqFOutput& output, const vectornavData& data);
