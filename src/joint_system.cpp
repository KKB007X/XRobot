#include "xrobot/joint_system.hpp"
#include "xrobot/parser.hpp"
#include "xrobot/model.hpp"

#include <iostream>

#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/Link.hh>

void xrobot::JointSystem::Configure(
    const gz::sim::Entity&,
    const std::shared_ptr<const sdf::Element>& sdf,
    gz::sim::EntityComponentManager& ecm,
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

    if (!parser.LoadJoints(filename.c_str(), this->model))
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
    
    ecm.Each<gz::sim::components::Name>([&](const gz::sim::Entity &entity,const gz::sim::components::Name *name){
        auto part = model.FindPart(name->Data());
        if (part){
            part->entity = entity;
            std::cout << "Found: " << name->Data() << std::endl;
        }
        return true;
    });

    for (auto& jointPtr : model.joints){

        auto& joint = *jointPtr;
        auto parent =  model.FindPart(joint.parent)->entity;
        auto child =  model.FindPart(joint.child)->entity;

        if (parent == gz::sim::kNullEntity || child == gz::sim::kNullEntity){
            std::cout << joint.name << " : component not found!" << std::endl;
            return;
        }
        gz::sim::Link parentLink(parent);
        gz::sim::Link childLink(child);

        joint.parentLink = parentLink;
        joint.childLink = childLink;

        parentLink.EnableVelocityChecks(ecm);
        childLink.EnableVelocityChecks(ecm);

        auto parentpose = *(parentLink.WorldPose(ecm));
        auto childpose = *(childLink.WorldPose(ecm));

        gz::math::Vector3d point(joint.ppx, joint.ppy, joint.ppz);
        gz::math::Vector3d axis(joint.paxisX, joint.paxisY, joint.paxisZ);

        gz::math::Vector3d pointChild = childpose.Rot().Inverse().RotateVector(parentpose.Pos() 
                                      - childpose.Pos() + parentpose.Rot().RotateVector(point));
        gz::math::Vector3d axisChild = childpose.Rot().Inverse().RotateVector(parentpose.Rot().RotateVector(axis));
        gz::math::Quaterniond childRotInParent = parentpose.Rot().Inverse() * childpose.Rot();

        joint.cpx = pointChild.X();
        joint.cpy = pointChild.Y();
        joint.cpz = pointChild.Z();

        joint.caxisX = axisChild.X();
        joint.caxisY = axisChild.Y();
        joint.caxisZ = axisChild.Z();

        joint.cqx = childRotInParent.X();
        joint.cqy = childRotInParent.Y();
        joint.cqz = childRotInParent.Z();
        joint.cqw = childRotInParent.W();

    }

    loaded = true;
}

void xrobot::JointSystem::PreUpdate(
    const gz::sim::UpdateInfo&,
    gz::sim::EntityComponentManager& ecm)
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



    for (auto& jointPtr : model.joints){
        auto& joint = *jointPtr;

        auto parentpose = *(joint.parentLink.WorldPose(ecm));
        auto childpose  = *(joint.childLink.WorldPose(ecm));

        if (joint.type == "revolute"){
            gz::math::Vector3d localp(joint.ppx, joint.ppy, joint.ppz);
            gz::math::Vector3d localc(joint.cpx, joint.cpy, joint.cpz);

            gz::math::Vector3d axisp(joint.paxisX, joint.paxisY, joint.paxisZ);
            gz::math::Vector3d axisc(joint.caxisX, joint.caxisY, joint.caxisZ);

            auto worldlp = parentpose.Pos() + parentpose.Rot().RotateVector(localp);
            auto worldlc = childpose.Pos() + childpose.Rot().RotateVector(localc);

            auto worldap = parentpose.Rot().RotateVector(axisp);
            auto worldac = childpose.Rot().RotateVector(axisc);

            auto parentVel = joint.parentLink.WorldLinearVelocity(ecm, localp);
            auto childVel = joint.childLink.WorldLinearVelocity(ecm, localc);

            if (!parentVel || !childVel)
            {
                continue;
            }
            
            auto errorl = worldlp - worldlc;
            auto velol = *parentVel - *childVel;

            auto force = - joint.kLinear * errorl - joint.dLinear * velol;

            auto crossAxis = worldap.Cross(worldac);
            double crossLen = crossAxis.Length();

            gz::math::Vector3d axisError(0, 0, 0);
            if (crossLen > 1e-8)
            {
                axisError = crossAxis / crossLen;
            }

            double cosTheta = std::clamp(worldap.Dot(worldac),-1.0,1.0);

            double angleError = -acos(cosTheta);

            auto torque = -joint.kAngular * angleError * axisError;

            auto parentOmega = joint.parentLink.WorldAngularVelocity(ecm);
            auto childOmega = joint.childLink.WorldAngularVelocity(ecm);

            if (parentOmega && childOmega)
            {
                auto omegaRel = *parentOmega - *childOmega;
                torque -= joint.dAngular * omegaRel;
            }

            joint.parentLink.AddWorldWrench(ecm, force, torque, localp);
            joint.childLink.AddWorldWrench(ecm, -force, -torque, localc);
        }
    }
}

GZ_ADD_PLUGIN(
    xrobot::JointSystem,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate)