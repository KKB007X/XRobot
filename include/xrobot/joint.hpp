#pragma once

#include <string>
#include <gz/sim/Link.hh>

namespace xrobot
{
class Joint
{
public:

    std::string name;//

    std::string type;//

    std::string control;//

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

    double cqx;
    double cqy;
    double cqz;
    double cqw;

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

};
}