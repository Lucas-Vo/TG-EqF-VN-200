#include "EqFalgo.hpp"
#include <unsupported/Eigen/MatrixFunctions>

/*
 * Public API
 */

TGEqF::TGEqF(Mat3 Qmag, double Qbaro, Mat6 Qgnss, Mat18 Sigma0, Mat18 P, bool applyReset)
    : applyReset(applyReset)
{
    this->Qmag = Qmag;
    this->Qbaro = Qbaro;
    this->Qgnss = Qgnss;
    this->Sigma = Sigma0;
    this->P = P;
    Xhat.pose = SE23{};
    Xhat.bias = se23::Zero();
    dt = 0;
    last_time = 0;
}

TGEqF::TGEqF(bool applyReset)
    : TGEqF(defaultQmag(), defaultQbaro(), defaultQgnss(), defaultSigma0(), defaultP(), applyReset)
{
}


void TGEqF::IMUpropagagte(Vec3 gyro, Vec3 acc, double time) {

    dt = time - last_time;
    last_time = time;
    // make sure dt is good
    if (dt > 0.4) {
        return;
    }
    calculateA(gyro, acc);
    calculateLift(gyro, acc);
    Mat18 Phi = (A*dt).exp();
    Mat18 B = Mat18::Zero();
    B.block<9, 9>(0, 0) = Xhat.pose.Adjoint(); 
    B.block<9, 9>(9, 9) = Xhat.pose.Adjoint(); 

    Lift.pose *= dt;
    Lift.bias *= dt;
    Xhat =MulSE23xse23(Xhat, ExpSE23xse23(Lift));

    Sigma = Phi * Sigma * Phi.transpose() + B * P * B.transpose() * dt;

}

void TGEqF::MagUpdate(const Mat3& mag) {
    if (!mag.allFinite())
    {
        return;
    }

    const Vec3 delta = SO3::log(SO3(mag) * SO3(Xhat.pose.R()).inv());

    Mat3x18 C = Mat3x18::Zero();
    C.block<3,3>(0,0) = Mat3::Identity();

    Mat3 Sinv = (C * Sigma * C.transpose() + Qmag).inverse();
    Mat18x3 K = Sigma * C.transpose() * Sinv;
    Vec18 DeltaVee = K * delta;
    se23xse23 Delta;
    Delta.pose = SE23::wedge(DeltaVee.segment<9>(0));
    Delta.bias = SE23::wedge(DeltaVee.segment<9>(9));

    Mat9 ad_Delta1 = SE23::adjoint(DeltaVee.segment<9>(0));
    Mat9 ad_Delta2 = SE23::adjoint(DeltaVee.segment<9>(9));
    Mat18 ad_Delta = Mat18::Zero();
    ad_Delta.block<9,9>(0,0) = ad_Delta1;
    ad_Delta.block<9,9>(9,0) = ad_Delta2;
    ad_Delta.block<9,9>(9,9) = ad_Delta1;

    Mat18 GammaExp = (0.5*ad_Delta).exp();

    Xhat = MulSE23xse23(ExpSE23xse23(Delta), Xhat);
    const Mat18 innovationUpdate = Mat18::Identity() - K * C;
    if (applyReset)
    {
        Sigma = GammaExp * innovationUpdate * Sigma * GammaExp.transpose();
    }
    else
    {
        Sigma = innovationUpdate * Sigma;
    }
}

void TGEqF::BaroUpdate(double baro) {
    const double delta = Xhat.pose.p().z() - baro;

    Eigen::Matrix<double, 1, 18> C = Eigen::Matrix<double, 1, 18>::Zero();
    C.block<1,3>(0,0) = Vec3{0, 0, 1}.transpose() * SO3::wedge(0.5 * (Xhat.pose.p() + Vec3{0, 0, baro}));
    C(0, 8) = -1.0;

    const double Sinv = 1.0 / ((C * Sigma * C.transpose())(0, 0) + Qbaro);
    const Eigen::Matrix<double, 18, 1> K = Sigma * C.transpose() * Sinv;
    Vec18 DeltaVee = K * delta;
    se23xse23 Delta;
    Delta.pose = SE23::wedge(DeltaVee.segment<9>(0));
    Delta.bias = SE23::wedge(DeltaVee.segment<9>(9));

    Mat9 ad_Delta1 = SE23::adjoint(DeltaVee.segment<9>(0));
    Mat9 ad_Delta2 = SE23::adjoint(DeltaVee.segment<9>(9));
    Mat18 ad_Delta = Mat18::Zero();
    ad_Delta.block<9,9>(0,0) = ad_Delta1;
    ad_Delta.block<9,9>(9,0) = ad_Delta2;
    ad_Delta.block<9,9>(9,9) = ad_Delta1;

    Mat18 GammaExp = (0.5 * ad_Delta).exp();

    Xhat = MulSE23xse23(ExpSE23xse23(Delta), Xhat);
    const Mat18 innovationUpdate = Mat18::Identity() - K * C;
    if (applyReset)
    {
        Sigma = GammaExp * innovationUpdate * Sigma * GammaExp.transpose();
    }
    else
    {
        Sigma = innovationUpdate * Sigma;
    }

}

void TGEqF::GnssUpdate(Vec3 gnssPos, Vec3 gnssVel) {
    Vec6 delta = Vec6::Zero();
    delta.segment<3>(0) = gnssPos - Xhat.pose.p();
    delta.segment<3>(3) = gnssVel - Xhat.pose.v();

    Mat6x18 C = Mat6x18::Zero();
    C.block<3,3>(0,0) = -SO3::wedge(Xhat.pose.p());
    C.block<3,3>(0,6) = Mat3::Identity();
    C.block<3,3>(3,0) = -SO3::wedge(Xhat.pose.v());
    C.block<3,3>(3,3) = Mat3::Identity();

    Mat6 Sinv = (C * Sigma * C.transpose() + Qgnss).inverse();
    Mat18x6 K = Sigma * C.transpose() * Sinv;
    Vec18 DeltaVee = K * delta;
    se23xse23 Delta;
    Delta.pose = SE23::wedge(DeltaVee.segment<9>(0));
    Delta.bias = SE23::wedge(DeltaVee.segment<9>(9));

    Mat9 ad_Delta1 = SE23::adjoint(DeltaVee.segment<9>(0));
    Mat9 ad_Delta2 = SE23::adjoint(DeltaVee.segment<9>(9));
    Mat18 ad_Delta = Mat18::Zero();
    ad_Delta.block<9,9>(0,0) = ad_Delta1;
    ad_Delta.block<9,9>(9,0) = ad_Delta2;
    ad_Delta.block<9,9>(9,9) = ad_Delta1;

    Mat18 GammaExp = (0.5*ad_Delta).exp(); // check if this should be negative ref https://arxiv.org/pdf/2209.04965 (44)

    Xhat = MulSE23xse23(ExpSE23xse23(Delta), Xhat);
    const Mat18 innovationUpdate = Mat18::Identity() - K * C;
    if (applyReset)
    {
        Sigma = GammaExp * innovationUpdate * Sigma * GammaExp.transpose();
    }
    else
    {
        Sigma = innovationUpdate * Sigma;
    }

}


EqFOutput TGEqF::GetEqFOutput() {
    EqFOutput res;
    res.Xhat = Xhat;
    res.Sigma = Sigma;
    return res;
}


/*
 * Unexported methods
 */

void TGEqF::calculateA(Vec3 gyro, Vec3 acc)
{
    const Vec9 beta_hat = SE23::vee(Xhat.bias);
    const Vec9 b_hat = -Xhat.pose.invAdjoint() * beta_hat;

    Vec9 w = Vec9::Zero();
    w.segment<3>(0) = gyro;
    w.segment<3>(3) = acc;
    w.segment<3>(6) = b_hat.segment<3>(6);
    Mat3 g_wedge = SO3::wedge(g);

    const se23 w_wedge = SE23::wedge(w);
    
    se23 f10 = se23::Zero();
    f10.block<3, 1>(0, 4) = Xhat.pose.v();
    
    Vec9 w0 = Xhat.pose.Adjoint() * w + SE23::vee(Xhat.bias) + SE23::vee(f10);
    Vec9 w0_with_g = w0;
    w0_with_g.segment<3>(3) += g;
    
    A = Mat18::Zero();
    A.block<3, 3>(3, 0) = g_wedge;
    A.block<3, 3>(6, 3) = Mat3::Identity();
    A.block<9, 9>(0, 9) = Mat9::Identity(); // see if this needs to be negative?
    A.block<9, 9>(9, 0) = Mat9::Zero();
    A.block<9, 9>(9, 9) = SE23::adjoint(w0_with_g);
}

void TGEqF::calculateLift(Vec3 gyro, Vec3 acc)
{
    const Vec9 beta_hat = SE23::vee(Xhat.bias);
    const Vec9 b_hat = -Xhat.pose.invAdjoint() * beta_hat;

    Vec9 w = Vec9::Zero();
    w.segment<3>(0) = gyro;
    w.segment<3>(3) = acc;
    w.segment<3>(6) = b_hat.segment<3>(6);

    se23 f10g = se23::Zero();
    f10g.block<3, 1>(0, 3) = g;
    f10g.block<3, 1>(0, 4) = Xhat.pose.v();

    const Vec9 lift_pose_vec = w - b_hat + SE23::vee(Xhat.pose.inv() * f10g);
    Lift.pose = SE23::wedge(lift_pose_vec);
    Lift.bias = SE23::wedge(SE23::adjoint(b_hat) * lift_pose_vec);
}

/*
 * Default static matrices
 */

Mat3 TGEqF::defaultQmag()
{
    Mat3 Q = Mat3::Zero();

    // Magnetometer xyz
    Q(0, 0) = 1;
    Q(1, 1) = 1;
    Q(2, 2) = 1;

    return Q;
}

double TGEqF::defaultQbaro()
{
    return 2.0 * 2.0;
}

Mat6 TGEqF::defaultQgnss()
{
    Mat6 Q = Mat6::Zero();

    // higher means following measuremnts a lot
    // lower means means smoother, 1e-5 -> velU = 1.1,  1e-4 -> velU = 0.5 1e-3 -> velU = 0.5
    Q.block<3, 3>(0, 0) = 1 * Mat3::Identity();

    // higher means more jagged
    // lower means following measured perfectly, 10-> posU = 1.1,100->PosU = 2.9, 40-> PosU = 2.2
    Q.block<3, 3>(3, 3) = 9 * Mat3::Identity();


    return Q;
}

Mat18 TGEqF::defaultSigma0()
{
    Mat18 S = Mat18::Zero();

    // Assumed order:
    // [rot(3), vel(3), pos(3), gyro_bias(3), accel_bias(3), virtual_bias(3)]

    S.block<3, 3>(0, 0) = 2.0 * 2.0 * Mat3::Identity();     // attitude
    S.block<3, 3>(3, 3) = 0.5 * 0.5 * Mat3::Identity();     // velocity
    S.block<3, 3>(6, 6) = 5.0 * 5.0 * Mat3::Identity();     // position
    S.block<3, 3>(9, 9) = 0.02 * 0.02 * Mat3::Identity();   // gyro bias
    S.block<3, 3>(12, 12) = 2 * 2 * Mat3::Identity(); // accel bias
    S.block<3, 3>(15, 15) = 0.50 * 0.50 * Mat3::Identity(); // virtual bias

    return S;
}

Mat18 TGEqF::defaultP()
{
    Mat18 P = Mat18::Zero();

    // Continuous-time process covariance placeholders

    P.block<3, 3>(0, 0) = 1e-1 * Mat3::Identity();   // rot process

    // higher means following position measurements perfectly, convergence of accel bias is slower
    P.block<3, 3>(3, 3) = 1e-9 * Mat3::Identity();   // vel process

    // does nothing
    P.block<3, 3>(6, 6) = 1 * Mat3::Identity();   // pos process

    P.block<3, 3>(9, 9) = 1e-3 * Mat3::Identity();   // gyro bias RW

    // higher means follows measurements more, bias converges faster, higher PosU
    // lower means bias convergence is slow, allows constant offset in velocity and position 1->4sec convergence, lower PosU
    P.block<3, 3>(12, 12) = 0.1 * Mat3::Identity(); // accel bias RW 

    // doesnt do anything
    P.block<3, 3>(15, 15) = 1e-9 * Mat3::Identity(); // virtual bias RW

    return P;
}
