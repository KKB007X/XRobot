#pragma once

#include <string>
#include <gz/sim/Entity.hh>

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

    double density;

    double comX = 0.0;
    double comY = 0.0;
    double comZ = 0.0;

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

    gz::sim::Entity entity = gz::sim::kNullEntity;
};
}