#include "xrobot/parser.hpp"
#include "xrobot/model.hpp"
#include "xrobot/part.hpp"
#include "xrobot/joint.hpp"
#include "xrobot/step_loader.hpp"

#include <iostream>
#include <memory>

#include <tinyxml2.h>

bool xrobot::Parser::Load(const char *filename, Model& model)
{
    tinyxml2::XMLDocument doc;

    auto result = doc.LoadFile(filename);

    if (result != tinyxml2::XML_SUCCESS)
    {
        std::cout
            << "Failed to load file"
            << std::endl;

        return false;
    }

    auto root = doc.FirstChildElement("xrobot");

    if (!root)
    {
        std::cout
            << "No xrobot tag found"
            << std::endl;

        return false;
    }

    std::cout
        << "Found xrobot root"
        << std::endl;

    const char *version = root->Attribute("version");

    if (version)
    {
        std::cout
            << "Version: "
            << version
            << std::endl;
    }

    auto geometry = root->FirstChildElement("geometry");

    if (!geometry)
    {
        std::cout
            << "Geometry not found"
            << std::endl;

        return false;
    }

    const char *type = geometry->Attribute("type");

    if (!type)
    {
        type = "step";
    }

    const char *path = geometry->Attribute("path");

    if (!path)
    {
        std::cout
            << "Geometry path missing"
            << std::endl;

        return false;
    }

    double resolution = geometry->DoubleAttribute("resolution", 0.1);

    model.geometryType = type;
    model.geometryPath = path;
    model.resolution = resolution;

    std::cout
        << "Geometry Type: "
        << type
        << std::endl;

    std::cout
        << "Geometry Path: "
        << path
        << std::endl;

    for (
        auto part = root->FirstChildElement("part");
        part;
        part = part->NextSiblingElement("part"))
    {
        const char *name = part->Attribute("name");

        if (!name)
        {
            std::cout
                << "Part missing name"
                << std::endl;

            return false;
        }

        auto newPart = std::make_unique<xrobot::Part>();

        newPart->name = name;

        newPart->mass = part->DoubleAttribute("mass", 0.0);

        newPart->ixx = part->DoubleAttribute("ixx", 0.0);

        newPart->ixy = part->DoubleAttribute("ixy", 0.0);

        newPart->ixz = part->DoubleAttribute("ixz", 0.0);

        newPart->iyy = part->DoubleAttribute("iyy", 0.0);

        newPart->iyz = part->DoubleAttribute("iyz", 0.0);

        newPart->izz = part->DoubleAttribute("izz", 0.0);

        std::cout
            << "Added part: "
            << name
            << std::endl;

        model.AddPart(std::move(newPart));
    }

    for (
        auto joint = root->FirstChildElement("joint");
        joint;
        joint = joint->NextSiblingElement("joint"))
    {
        const char *name = joint->Attribute("name");

        if (!name)
        {
            std::cout
                << "Joint missing name"
                << std::endl;

            return false;
        }

        auto newJoint = std::make_unique<xrobot::Joint>();

        newJoint->name = name;

        std::cout
            << "Added joint: "
            << name
            << std::endl;

        model.AddJoint(
            std::move(
                newJoint));
    }

    xrobot::StepLoader stepLoader;

    if (!stepLoader.Load(path, model))
    {
        return false;
    }

    return true;

    return true;
}