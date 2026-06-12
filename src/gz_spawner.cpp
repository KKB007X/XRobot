#include <gz/transport/Node.hh>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/boolean.pb.h>
#include "xrobot/gz_spawner.hpp"

#include "xrobot/model.hpp"
#include <filesystem>
#include <sstream>

std::string xrobot::gzSpawner::FindWorld(gz::transport::Node& node)
{
    std::vector<std::string> services;
    node.ServiceList(services);

    for (const auto& service : services)
    {
        if (service.find("/world/") == 0 && service.find("/create") != std::string::npos)
        {
            std::size_t start = std::string("/world/").size();
            std::size_t end = service.find("/create");
            return service.substr(start, end - start);
        }
    }
    return "";
}

bool xrobot::gzSpawner::PauseWorld(std::string world)
{
    gz::transport::Node node;

    if (world.empty())
    {
        world = FindWorld(node);
    }
    

    if (world.empty())
    {
        std::cout
            << "No Gazebo world found"
            << std::endl;
            return false;
    }

    gz::msgs::WorldControl req;
    req.set_pause(true);

    gz::msgs::Boolean rep;
    bool result;

    bool executed = node.Request("/world/" + world + "/control", req, 5000, rep, result);

    return executed && result;
}

bool xrobot::gzSpawner::ResumeWorld(std::string world)
{
    gz::transport::Node node;

    if (world.empty())
    {
        world = FindWorld(node);
    }
    

    if (world.empty())
    {
        std::cout
            << "No Gazebo world found"
            << std::endl;
            return false;
    }

    gz::msgs::WorldControl req;
    req.set_pause(false);

    gz::msgs::Boolean rep;
    bool result;

    bool executed = node.Request("/world/" + world + "/control", req, 5000, rep, result);

    return executed && result;
}

bool xrobot::gzSpawner::Spawn(xrobot::Model& model, const char *filename, const SpawnOptions& options)
{
    gz::transport::Node node;
    

    std::string world = options.world;

    if (world.empty())
    {
        world = FindWorld(node);
    }
    

    if (world.empty())
    {
        std::cout
            << "No Gazebo world found"
            << std::endl;

        return false;
    }

    std::stringstream sdf;

    sdf << "<sdf version='1.10'>";
    sdf << "<model name='xrobot1'>";

    sdf << "<pose>"
        << options.x << " "
        << options.y << " "
        << options.z << " "
        << options.roll << " "
        << options.pitch << " "
        << options.yaw
        << "</pose>";
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
            << "<specular>1 1 1 1</specular>"
            << "<pbr><metal><roughness>0.4</roughness><metalness>0.5</metalness></metal></pbr>"
            << "</material>"
            << "<cast_shadows>true</cast_shadows>";
        sdf << "</visual>";

        sdf << "<collision name='collision'>";

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

        sdf << "<pose>"
            << part.comX << " "
            << part.comY << " "
            << part.comZ << " "
            << "0 0 0"
            << "</pose>";

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

    sdf << "<plugin "
    << "filename = \"libxrobot_joint_system.so\" "
    << "name = \"xrobot::JointSystem\">"
    << "<xrobot_file>"
    << std::filesystem::absolute(filename).string()
    << "</xrobot_file>"
    << "</plugin>";

    sdf << "</sdf>";

    // std::cout << sdf.str();

    gz::msgs::EntityFactory req;
    req.set_sdf(sdf.str());

    gz::msgs::Boolean rep;
    bool result;

    bool executed = node.Request("/world/" + world + "/create", req, 5000, rep, result);

    return executed && result;
}