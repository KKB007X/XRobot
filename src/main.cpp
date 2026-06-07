#include <iostream>

#include "xrobot/part.hpp"

int main()
{
    xrobot::Part R14;

    R14.name = "R14";
    R14.mass = 0.273;

    std::cout
        << "Part: "
        << R14.name
        << std::endl;

    std::cout
        << "Mass: "
        << R14.mass
        << std::endl;

    return 0;
}