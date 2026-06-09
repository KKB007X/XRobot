#include "xrobot/parser.hpp"
#include "xrobot/step_loader.hpp"
#include <iostream>

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

    return 0;
}