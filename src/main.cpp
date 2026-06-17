#include "xrobot/parser.hpp"
#include "xrobot/step_loader.hpp"
#include "xrobot/gz_spawner.hpp"

#include "xrobot/model.hpp"

#include <iostream>
#include <string>
#include <sstream>



int main()
{
    xrobot::Parser parser;
    xrobot::StepLoader step_loader;
    xrobot::gzSpawner gz_spawner;
    xrobot::Model model;
    bool success;
    const char *filename = "../examples/gripper.xrobot";

    success = parser.LoadParts(filename, model);

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

    success = step_loader.Load(model);

    if (success)
    {
        std::cout
            << "step file loaded"
            << std::endl;
    }
    else
    {
        std::cout
            << "Failed to load step file"
            << std::endl;
    }

    xrobot::SpawnOptions options;
    success = gz_spawner.PauseWorld() && gz_spawner.Spawn(model, filename, options);// && gz_spawner.ResumeWorld();

    if (success)
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



