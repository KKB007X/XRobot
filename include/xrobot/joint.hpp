#pragma once

#include <string>
#include <gz/sim/Link.hh>
#include <gz/transport/Node.hh>
#include <atomic>

namespace xrobot
{
class Joint
{
public:

    std::string name;//

    std::string type;//

    std::string control = "";//
    std::atomic<double> target_cmd{0.0};

    std::string parent;//
    std::string child;//

    double ppx;//
    double ppy;//
    double ppz;//

    double paxisX;//
    double paxisY;//
    double paxisZ;//
    
    double cpx;
    double cpy;
    double cpz;

    double caxisX;
    double caxisY;
    double caxisZ;

    double kLinear;//
    double kAngular;//

    double linearTolerance;//
    double angularTolerance;//

    double lowerLimit;//
    double upperLimit;//

    double speed;//
    double effort;//

    double dLinear;//
    double dAngular;//

    gz::sim::Link parentLink;
    gz::sim::Link childLink;

    gz::math::Vector3d pPerp;
    gz::math::Vector3d cPerp;

};
}