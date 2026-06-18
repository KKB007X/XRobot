#include "xrobot/joint_system.hpp"
#include "xrobot/parser.hpp"
#include "xrobot/model.hpp"

#include <iostream>

#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/Link.hh>
#include <cmath>

void xrobot::JointSystem::Configure(
    const gz::sim::Entity&,
    const std::shared_ptr<const sdf::Element>& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager&)
{
    if (!sdf->HasElement("xrobot_file"))
    {
        std::cout << "Missing xrobot_file" << std::endl;
        return;
    }

    std::string filename = sdf->Get<std::string>("xrobot_file");
    xrobot::Parser parser;

    if (!parser.LoadJoints(filename.c_str(), this->model))
    {
        std::cout << "Failed to load joints" << std::endl;
        return;
    }

    std::cout << "Loaded " << model.joints.size() << " joints" << std::endl;
    
    ecm.Each<gz::sim::components::Name>([&](const gz::sim::Entity &entity, const gz::sim::components::Name *name){
        auto part = model.FindPart(name->Data());
        if (part){
            part->entity = entity;
            std::cout << "Found: " << name->Data() << std::endl;
        }
        return true;
    });

    for (auto& jointPtr : model.joints){
        auto& joint = *jointPtr;
        auto parent = model.FindPart(joint.parent)->entity;
        auto child = model.FindPart(joint.child)->entity;

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

        gz::math::Vector3d helper(1, 0, 0);
        if (std::abs(axis.X()) > 0.9)
        {
            helper = gz::math::Vector3d(0, 1, 0);
        }

        joint.pPerp = axis.Cross(helper).Normalize();

        gz::math::Vector3d pointChild = childpose.Rot().Inverse().RotateVector(parentpose.Pos() 
                                      - childpose.Pos() + parentpose.Rot().RotateVector(point));
        gz::math::Vector3d axisChild = childpose.Rot().Inverse().RotateVector(parentpose.Rot().RotateVector(axis));
        joint.cPerp = childpose.Rot().Inverse().RotateVector(parentpose.Rot().RotateVector(joint.pPerp));

        joint.cpx = pointChild.X();
        joint.cpy = pointChild.Y();
        joint.cpz = pointChild.Z();

        joint.caxisX = axisChild.X();
        joint.caxisY = axisChild.Y();
        joint.caxisZ = axisChild.Z();

        if (!joint.control.empty()){
            std::string topicName = "/model/xrobot/joint/" + joint.name + "/cmd";

            // Standard raw lambda callback to write incoming commands straight to our atomic variable
            auto cb = [&joint](const gz::msgs::Double &_msg) {
                joint.target_cmd.store(_msg.data());
            };

            // 1. node.Subscribe returns a bool (true if successful)
            bool success = this->transport_node.Subscribe<gz::msgs::Double>(topicName, cb);
            
            if (success)
            {
                std::cout << "[xrobot] Successfully registered command channel: " << topicName << std::endl;
            }
            else
            {
                std::cerr << "[xrobot] Failed to register command channel: " << topicName << std::endl;
            }
        }
    }

    loaded = true;
}

void xrobot::JointSystem::PreUpdate(
    const gz::sim::UpdateInfo&,
    gz::sim::EntityComponentManager& ecm)
{
    if (!loaded) return;

    static bool printed = false;
    if (!printed)
    {
        for (auto& jointPtr : model.joints)
        {
            std::cout << "Joint: " << jointPtr->name << std::endl;
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

            auto worldpp = parentpose.Rot().RotateVector(joint.pPerp);
            auto worldcp = childpose.Rot().RotateVector(joint.cPerp);

            auto parentVel = joint.parentLink.WorldLinearVelocity(ecm, localp);
            auto childVel = joint.childLink.WorldLinearVelocity(ecm, localc);

            if (!parentVel || !childVel) continue;
            
            auto errorl = worldlp - worldlc;
            auto velol = *parentVel - *childVel;

            if (errorl.Length() < joint.linearTolerance){
                errorl = gz::math::Vector3d::Zero;
            }

            auto force = -joint.kLinear * errorl - joint.dLinear * velol;

            auto crossAxis = worldap.Cross(worldac);
            double crossLen = crossAxis.Length();

            gz::math::Vector3d axisError(0, 0, 0);
            if (crossLen > 1e-8)
            {
                axisError = crossAxis / crossLen;
            }

            double cosTheta = std::clamp(worldap.Dot(worldac), -1.0, 1.0);
            double angleError = -acos(cosTheta);

            auto torque = -joint.kAngular * angleError * axisError;

            if (std::abs(angleError) < joint.angularTolerance){
                torque = gz::math::Vector3d::Zero;
            }

            auto parentOmega = joint.parentLink.WorldAngularVelocity(ecm);
            auto childOmega = joint.childLink.WorldAngularVelocity(ecm);
            
            gz::math::Vector3d omegaRel(0, 0, 0);
            if (parentOmega && childOmega){
                omegaRel = *parentOmega - *childOmega;
                torque -= joint.dAngular * omegaRel;
            }

            // Create a clean, unit-length functional joint axis
            gz::math::Vector3d jointAxis = (worldap + worldac).Normalize();

            double y = worldcp.Dot(worldpp.Cross(jointAxis)); 
            double x = worldcp.Dot(worldpp);               
            double angle = std::atan2(y, x);

            // Joint Limit Constraints
            if (angle > joint.upperLimit){
                torque += joint.kAngular * (joint.upperLimit - angle) * jointAxis;
            } else if (angle < joint.lowerLimit){
                torque += joint.kAngular * (joint.lowerLimit - angle) * jointAxis;
            }

            if (joint.control == "position"){
                double current_target = joint.target_cmd.load();
                double current_speed = omegaRel.Dot(jointAxis);
                double ae = std::abs(angle-current_target);
                
                double effort = joint.kAngular*ae;
                if (effort>joint.effort){
                    effort = joint.effort;
                }

                if (angle > current_target){//&& current_speed >= joint.speed
                    torque -= effort * jointAxis;

                } else if (angle < current_target){//&& -current_speed >= joint.speed
                    torque += effort * jointAxis;
                }
            }

            joint.parentLink.AddWorldWrench(ecm, force, torque, localp);
            joint.childLink.AddWorldWrench(ecm, -force, -torque, localc);
        }
        if (joint.type == "continuous"){
            gz::math::Vector3d localp(joint.ppx, joint.ppy, joint.ppz);
            gz::math::Vector3d localc(joint.cpx, joint.cpy, joint.cpz);

            gz::math::Vector3d axisp(joint.paxisX, joint.paxisY, joint.paxisZ);
            gz::math::Vector3d axisc(joint.caxisX, joint.caxisY, joint.caxisZ);

            auto worldlp = parentpose.Pos() + parentpose.Rot().RotateVector(localp);
            auto worldlc = childpose.Pos() + childpose.Rot().RotateVector(localc);

            auto worldap = parentpose.Rot().RotateVector(axisp);
            auto worldac = childpose.Rot().RotateVector(axisc);

            auto worldpp = parentpose.Rot().RotateVector(joint.pPerp);
            auto worldcp = childpose.Rot().RotateVector(joint.cPerp);

            auto parentVel = joint.parentLink.WorldLinearVelocity(ecm, localp);
            auto childVel = joint.childLink.WorldLinearVelocity(ecm, localc);

            if (!parentVel || !childVel) continue;
            
            auto errorl = worldlp - worldlc;
            auto velol = *parentVel - *childVel;

            if (errorl.Length() < joint.linearTolerance){
                errorl = gz::math::Vector3d::Zero;
            }

            auto force = -joint.kLinear * errorl - joint.dLinear * velol;

            auto crossAxis = worldap.Cross(worldac);
            double crossLen = crossAxis.Length();

            gz::math::Vector3d axisError(0, 0, 0);
            if (crossLen > 1e-8)
            {
                axisError = crossAxis / crossLen;
            }

            double cosTheta = std::clamp(worldap.Dot(worldac), -1.0, 1.0);
            double angleError = -acos(cosTheta);

            auto torque = -joint.kAngular * angleError * axisError;

            if (std::abs(angleError) < joint.angularTolerance){
                torque = gz::math::Vector3d::Zero;
            }

            auto parentOmega = joint.parentLink.WorldAngularVelocity(ecm);
            auto childOmega = joint.childLink.WorldAngularVelocity(ecm);
            
            gz::math::Vector3d omegaRel(0, 0, 0);
            if (parentOmega && childOmega){
                omegaRel = *parentOmega - *childOmega;
                torque -= joint.dAngular * omegaRel;
            }

            // Create a clean, unit-length functional joint axis
            gz::math::Vector3d jointAxis = (worldap + worldac).Normalize();

            double y = worldcp.Dot(worldpp.Cross(jointAxis)); 
            double x = worldcp.Dot(worldpp);               
            double angle = std::atan2(y, x);

            if (joint.control == "position"){
                double current_target = joint.target_cmd.load();
                double current_speed = omegaRel.Dot(jointAxis);
                double ae = std::abs(angle-current_target);
                
                double effort = joint.kAngular*ae;
                if (effort>joint.effort){
                    effort = joint.effort;
                }

                if (angle > current_target){//&& current_speed >= joint.speed
                    torque -= effort * jointAxis;

                } else if (angle < current_target){//&& -current_speed >= joint.speed
                    torque += effort * jointAxis;
                }
            }

            joint.parentLink.AddWorldWrench(ecm, force, torque, localp);
            joint.childLink.AddWorldWrench(ecm, -force, -torque, localc);
        }
        if (joint.type == "fixed"){
            gz::math::Vector3d localp(joint.ppx, joint.ppy, joint.ppz);
            gz::math::Vector3d localc(joint.cpx, joint.cpy, joint.cpz);

            gz::math::Vector3d axisp(joint.paxisX, joint.paxisY, joint.paxisZ);
            gz::math::Vector3d axisc(joint.caxisX, joint.caxisY, joint.caxisZ);

            auto worldlp = parentpose.Pos() + parentpose.Rot().RotateVector(localp);
            auto worldlc = childpose.Pos() + childpose.Rot().RotateVector(localc);

            auto worldap = parentpose.Rot().RotateVector(axisp);
            auto worldac = childpose.Rot().RotateVector(axisc);

            auto worldpp = parentpose.Rot().RotateVector(joint.pPerp);
            auto worldcp = childpose.Rot().RotateVector(joint.cPerp);

            auto parentVel = joint.parentLink.WorldLinearVelocity(ecm, localp);
            auto childVel = joint.childLink.WorldLinearVelocity(ecm, localc);

            if (!parentVel || !childVel) continue;
            
            auto errorl = worldlp - worldlc;
            auto velol = *parentVel - *childVel;

            if (errorl.Length() < joint.linearTolerance){
                errorl = gz::math::Vector3d::Zero;
            }

            auto force = -joint.kLinear * errorl - joint.dLinear * velol;

            auto crossAxis = worldap.Cross(worldac);
            double crossLen = crossAxis.Length();

            gz::math::Vector3d axisError(0, 0, 0);
            if (crossLen > 1e-8)
            {
                axisError = crossAxis / crossLen;
            }

            double cosTheta = std::clamp(worldap.Dot(worldac), -1.0, 1.0);
            double angleError = -acos(cosTheta);

            auto torque = -joint.kAngular * angleError * axisError;

            if (std::abs(angleError) < joint.angularTolerance){
                torque = gz::math::Vector3d::Zero;
            }

            auto parentOmega = joint.parentLink.WorldAngularVelocity(ecm);
            auto childOmega = joint.childLink.WorldAngularVelocity(ecm);
            
            gz::math::Vector3d omegaRel(0, 0, 0);
            if (parentOmega && childOmega){
                omegaRel = *parentOmega - *childOmega;
                torque -= joint.dAngular * omegaRel;
            }

            // Create a clean, unit-length functional joint axis
            gz::math::Vector3d jointAxis = (worldap + worldac).Normalize();

            double y = worldcp.Dot(worldpp.Cross(jointAxis)); 
            double x = worldcp.Dot(worldpp);               
            double angle = std::atan2(y, x);

            if (std::abs(angle) >= joint.angularTolerance)
            {torque -= joint.kAngular * (angle) * jointAxis;}
            
            joint.parentLink.AddWorldWrench(ecm, force, torque, localp);
            joint.childLink.AddWorldWrench(ecm, -force, -torque, localc);
        }
        if (joint.type == "prismatic"){
            gz::math::Vector3d localp(joint.ppx, joint.ppy, joint.ppz);
            gz::math::Vector3d localc(joint.cpx, joint.cpy, joint.cpz);

            gz::math::Vector3d axisp(joint.paxisX, joint.paxisY, joint.paxisZ);
            gz::math::Vector3d axisc(joint.caxisX, joint.caxisY, joint.caxisZ);

            auto worldlp = parentpose.Pos() + parentpose.Rot().RotateVector(localp);
            auto worldlc = childpose.Pos() + childpose.Rot().RotateVector(localc);

            auto worldap = parentpose.Rot().RotateVector(axisp);
            auto worldac = childpose.Rot().RotateVector(axisc);

            auto worldpp = parentpose.Rot().RotateVector(joint.pPerp);
            auto worldcp = childpose.Rot().RotateVector(joint.cPerp);

            auto parentVel = joint.parentLink.WorldLinearVelocity(ecm, localp);
            auto childVel = joint.childLink.WorldLinearVelocity(ecm, localc);

            if (!parentVel || !childVel) continue;
            
            auto errorl = worldlp - worldlc;
            auto velol = *parentVel - *childVel;

            if (errorl.Length() < joint.linearTolerance){
                errorl = gz::math::Vector3d::Zero;
            }

            auto force = -joint.kLinear * errorl;

            auto crossAxis = worldap.Cross(worldac);
            double crossLen = crossAxis.Length();

            gz::math::Vector3d axisError(0, 0, 0);
            if (crossLen > 1e-8)
            {
                axisError = crossAxis / crossLen;
            }

            double cosTheta = std::clamp(worldap.Dot(worldac), -1.0, 1.0);
            double angleError = -acos(cosTheta);

            auto torque = -joint.kAngular * angleError * axisError;

            if (std::abs(angleError) < joint.angularTolerance){
                torque = gz::math::Vector3d::Zero;
            }

            auto parentOmega = joint.parentLink.WorldAngularVelocity(ecm);
            auto childOmega = joint.childLink.WorldAngularVelocity(ecm);
            
            gz::math::Vector3d omegaRel(0, 0, 0);
            if (parentOmega && childOmega){
                omegaRel = *parentOmega - *childOmega;
                torque -= joint.dAngular * omegaRel;
            }

            // Create a clean, unit-length functional joint axis
            gz::math::Vector3d jointAxis = (worldap + worldac).Normalize();

            double y = worldcp.Dot(worldpp.Cross(jointAxis)); 
            double x = worldcp.Dot(worldpp);               
            double angle = std::atan2(y, x);

            if (std::abs(angle) >= joint.angularTolerance)
            {torque -= joint.kAngular * (angle) * jointAxis;}

            force -= jointAxis*(force.Dot(jointAxis));
            force -= joint.dLinear * velol;

            double dis = errorl.Dot(jointAxis);

            if (dis > joint.upperLimit){
                force += joint.kLinear * (joint.upperLimit - dis) * jointAxis;
            } else if (dis < joint.lowerLimit){
                force += joint.kLinear * (joint.lowerLimit - dis) * jointAxis;
            }
            if (joint.control == "position"){
                double current_target = joint.target_cmd.load();
                double current_speed = velol.Dot(jointAxis);
                double ae = std::abs(dis-current_target);
                
                double effort = joint.kLinear*ae;
                if (effort>joint.effort){
                    effort = joint.effort;
                }

                if (dis > current_target){//&& current_speed >= joint.speed
                    force -= effort * jointAxis;

                } else if (dis < current_target){//&& -current_speed >= joint.speed
                    force += effort * jointAxis;
                }
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