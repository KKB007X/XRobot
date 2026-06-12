#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>

#include "xrobot/model.hpp"

namespace xrobot
{
class JointSystem :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:

    void Configure(
        const gz::sim::Entity& entity,
        const std::shared_ptr<const sdf::Element>& sdf,
        gz::sim::EntityComponentManager& ecm,
        gz::sim::EventManager& eventMgr) override;

    void PreUpdate(
        const gz::sim::UpdateInfo& info,
        gz::sim::EntityComponentManager& ecm) override;

private:

    Model model;

    bool loaded = false;
};
}