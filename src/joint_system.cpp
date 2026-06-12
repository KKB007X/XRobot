#include "xrobot/joint_system.hpp"
#include "xrobot/parser.hpp"
#include "xrobot/model.hpp"

#include <iostream>

#include <gz/plugin/Register.hh>

void xrobot::JointSystem::Configure(
    const gz::sim::Entity&,
    const std::shared_ptr<const sdf::Element>& sdf,
    gz::sim::EntityComponentManager&,
    gz::sim::EventManager&)
{
    if (!sdf->HasElement("xrobot_file"))
    {
        std::cout
            << "Missing xrobot_file"
            << std::endl;

        return;
    }

    std::string filename = sdf->Get<std::string>("xrobot_file");

    xrobot::Parser parser;
    xrobot::Model model;
    this->model = model&;

    if (!parser.LoadJoints(filename.c_str(), model))
    {
        std::cout
            << "Failed to load joints"
            << std::endl;

        return;
    }

    std::cout
        << "Loaded "
        << model.joints.size()
        << " joints"
        << std::endl;

    loaded = true;
}

void xrobot::JointSystem::PreUpdate(
    const gz::sim::UpdateInfo&,
    gz::sim::EntityComponentManager&)
{
    if (!loaded)
    {
        return;
    }

    static bool printed = false;

    if (!printed)
    {
        for (auto& jointPtr : model.joints)
        {
            std::cout
                << "Joint: "
                << jointPtr->name
                << std::endl;
        }

        printed = true;
    }
}

GZ_ADD_PLUGIN(
    xrobot::JointSystem,
    gz::sim::System,
    xrobot::JointSystem::ISystemConfigure,
    xrobot::JointSystem::ISystemPreUpdate)