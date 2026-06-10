#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>

#include "xrobot/part.hpp"
#include "xrobot/joint.hpp"

namespace xrobot
{
class Model
{
public:

    std::string geometryType;
    std::string geometryPath;
    double resolution;
    double scale;

    std::vector<
        std::unique_ptr<Part>
    > parts;

    std::vector<
        std::unique_ptr<Joint>
    > joints;

    std::unordered_map<
        std::string,
        Part*> partMap;

    std::unordered_map<
        std::string,
        Joint*> jointMap;

    void AddPart(
        std::unique_ptr<Part> part);

    void AddJoint(
        std::unique_ptr<Joint> joint);

    Part* FindPart(
        const std::string& name);

    Joint* FindJoint(
        const std::string& name);
};
}