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

    double initialPx;
    double initialPy;
    double initialPz;

    double initialQx;
    double initialQy;
    double initialQz;
    double initialQw;
};
}