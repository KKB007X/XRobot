#include "xrobot/model.hpp"
#include "xrobot/part.hpp"
#include "xrobot/joint.hpp"

void xrobot::Model::AddPart(
    std::unique_ptr<xrobot::Part> part)
{
    partMap[part->name] =
        part.get();

    parts.push_back(
        std::move(part));
}

void xrobot::Model::AddJoint(
    std::unique_ptr<xrobot::Joint> joint)
{
    jointMap[joint->name] =
        joint.get();

    joints.push_back(
        std::move(joint));
}

xrobot::Part*
xrobot::Model::FindPart(
    const std::string& name)
{
    auto it =
        partMap.find(name);

    if (it == partMap.end())
    {
        return nullptr;
    }

    return it->second;
}

xrobot::Joint*
xrobot::Model::FindJoint(
    const std::string& name)
{
    auto it =
        jointMap.find(name);

    if (it == jointMap.end())
    {
        return nullptr;
    }

    return it->second;
}