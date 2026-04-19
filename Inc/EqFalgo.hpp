#pragma once

#include "linalgtypes.hpp"
#include "SE23xse23.hpp"

typedef struct
{
    SE23xse23 Xhat;
    Mat18 Sigma;
} EqFOutput;

class TGEqF
{
    // constants
    const Vec3 g = {0, 0, 9.81};
    const Vec3 m;

    SE23xse23 Xhat;     // State estimate
    Mat18 Sigma;        // state covariance

    // propagation time keeping
    double dt; //
    double last_time;

    // measurement and process noise
    Mat3 Qmag;     // measurement noise of mag
    double Qbaro;  // measurement noise of baro
    Mat6 Qgnss;    // measurement noise of gnss position and velocity
    Mat18 P;       // process noise

    // model matrices
    Mat18 A;
    se23xse23 Lift;

    void calculateA(Vec3 gyro, Vec3 acc);
    void calculateLift(Vec3 gyro, Vec3 acc);

    static Mat3 defaultQmag();
    static double defaultQbaro();
    static Mat6 defaultQgnss();
    static Mat18 defaultSigma0();
    static Mat18 defaultP();

public:
    TGEqF(const Vec3& magneticField, Mat3 Qmag, double Qbaro, Mat6 Qgnss, Mat18 Sigma0, Mat18 P);
    explicit TGEqF(const Vec3& magneticField); // tuned for VN200

    void IMUpropagagte(Vec3 gyro, Vec3 acc, double time);
    void MagUpdate(Vec3 mag);
    void BaroUpdate(double baro);
    void GnssUpdate(Vec3 gnssPos, Vec3 gnssVel);
    EqFOutput GetEqFOutput();
};
