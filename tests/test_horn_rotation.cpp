// Analytic rotation regressions: identity used to divide by a zero axis norm.
#include "HornRotation.h"
#include <iostream>
#include <limits>

int main() {
    using ORB_SLAM3::RotationFromHornQuaternion;
    for (float sign : {-1.f, 1.f}) {
        auto identity = RotationFromHornQuaternion(Eigen::Vector4f(sign,0,0,0));
        if (!identity.allFinite() || !identity.isApprox(Eigen::Matrix3f::Identity(),1e-6f)) return 1;
        Eigen::Matrix3f halfTurn = Eigen::Matrix3f::Identity();
        halfTurn(0,0) = halfTurn(1,1) = -1;
        if (!RotationFromHornQuaternion(Eigen::Vector4f(0,0,0,sign)).isApprox(halfTurn,1e-6f)) return 2;
    }
    const float h = std::sqrt(.5f);
    Eigen::Matrix3f quarter;
    quarter << 0,-1,0, 1,0,0, 0,0,1;
    if (!RotationFromHornQuaternion(Eigen::Vector4f(h,0,0,h)*3).isApprox(quarter,1e-5f)) return 3;
    try { RotationFromHornQuaternion(Eigen::Vector4f::Zero()); return 4; }
    catch (const std::invalid_argument&) {}
    try { RotationFromHornQuaternion(Eigen::Vector4f(std::numeric_limits<float>::quiet_NaN(),0,0,1)); return 5; }
    catch (const std::invalid_argument&) {}
    std::cout << "HORN_ROTATION_REGRESSIONS_OK\n";
}
