#include "xrobot/parser.hpp"
#include "xrobot/step_loader.hpp"
#include "xrobot/model.hpp"
#include <iostream>
#include <string>
#include <sstream>
#include <filesystem>

#include <gz/transport/Node.hh>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/boolean.pb.h>

bool Spawn(xrobot::Model& model)
{
    gz::transport::Node node;
    std::vector<std::string> services;
    node.ServiceList(services);

    std::string world;
    for (const auto& service : services)
    {
        if (service.find("/world/") == 0 && service.find("/create") != std::string::npos)
        {
            world = service;
            break;
        }
    }

    std::stringstream sdf;

    sdf << "<sdf version='1.10'>";
    sdf << "<model name='xrobot'>";
    for (auto& partPtr : model.parts)
    {
        auto& part = *partPtr;

        sdf << "<link name='" << part.name << "'>";

        sdf << "<pose>"
            << part.initialPx << " "
            << part.initialPy << " "
            << part.initialPz << " "
            << part.initialRoll << " "
            << part.initialPitch << " "
            << part.initialYaw << " "
            << "</pose>";

        sdf << "<visual name='visual'>";

        sdf << "<geometry>";
        sdf << "<mesh>";
        sdf << "<uri>file://"
            << std::filesystem::absolute(part.stlPath).string()
            << "</uri>";
        sdf << "<scale> " << model.scale <<" "<< model.scale <<" "<< model.scale <<" </scale>";
        sdf << "</mesh>";
        sdf << "</geometry>";
        sdf << "<material>"
            << "<ambient>" << part.rgba << "</ambient>"
            << "<diffuse>" << part.rgba << "</diffuse>"
            << "<specular>0.2 0.2 0.2 1</specular>"
            << "</material>";

        sdf << "</visual>";

        sdf << "<collision name='visual'>";

        sdf << "<geometry>";
        sdf << "<mesh>";
        sdf << "<uri>file://"
            << std::filesystem::absolute(part.stlPath).string()
            << "</uri>";
        sdf << "<scale> " << model.scale <<" "<< model.scale <<" "<< model.scale <<" </scale>";
        sdf << "</mesh>";
        sdf << "</geometry>";

        sdf << "</collision>";

        sdf << "<inertial>";

        sdf << "<mass>"
            << part.mass
            << "</mass>";

        sdf << "<inertia>";

        sdf << "<ixx>" << part.ixx << "</ixx>";
        sdf << "<ixy>" << part.ixy << "</ixy>";
        sdf << "<ixz>" << part.ixz << "</ixz>";

        sdf << "<iyy>" << part.iyy << "</iyy>";
        sdf << "<iyz>" << part.iyz << "</iyz>";

        sdf << "<izz>" << part.izz << "</izz>";

        sdf << "</inertia>";

        sdf << "</inertial>";

        sdf << "</link>";
    }
    sdf << "</model>";
    sdf << "</sdf>";

    gz::msgs::EntityFactory req;
    req.set_sdf(sdf.str());

    gz::msgs::Boolean rep;
    bool result;

    bool executed = node.Request(world, req, 5000, rep, result);

    return executed && result;
}

int main()
{
    xrobot::Parser parser;
    xrobot::Model model;

    bool success =
        parser.Load("../examples/gripper.xrobot", model);

    if (success)
    {
        std::cout
            << "XRobot file loaded"
            << std::endl;
    }
    else
    {
        std::cout
            << "Failed to load XRobot file"
            << std::endl;
    }

    if (Spawn(model))
    {
        std::cout
            << "Spawn successful"
            << std::endl;
    }
    else
    {
        std::cout
            << "Spawn failed"
            << std::endl;
    }

    return 0;
}



