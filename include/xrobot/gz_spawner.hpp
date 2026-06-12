#pragma once
#include <string>
#include "xrobot/model.hpp"
#include <gz/transport/Node.hh>

namespace xrobot
{
    struct SpawnOptions
    {
        std::string world = "";

        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
    };
    class gzSpawner
    {
    public:

        bool PauseWorld(std::string world = "");

        bool ResumeWorld(std::string world = "");

        bool Spawn(Model& model, const char *filename, const SpawnOptions& options = SpawnOptions());

    private:

        std::string FindWorld(gz::transport::Node& node);
    };
}