#pragma once
#include <string>
#include "xrobot/model.hpp"

namespace xrobot
{
class Parser
{
public:

    bool LoadParts(const char *filename, Model& model);
    bool LoadJoints(const char *filename, Model& model);
};

}