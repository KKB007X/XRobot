#pragma once

#include <string>

namespace xrobot
{
class Part
{
public:

    std::string name;

    double mass;

    double ixx;
    double ixy;
    double ixz;

    double iyy;
    double iyz;

    double izz;

    std::string stlPath;

    std::string rgba = "0.7 0.7 0.7 1.0";

    double initialPx;
    double initialPy;
    double initialPz;

    double initialQx;
    double initialQy;
    double initialQz;
    double initialQw;

    double initialRoll;
    double initialPitch;
    double initialYaw;
};
}