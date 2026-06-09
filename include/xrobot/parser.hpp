#pragma once
#include <string>
#include "xrobot/model.hpp"

namespace xrobot
{
class Parser
{
public:

    bool Load(const char *filename, Model& model);
};

}