#pragma once

#include <stdint.h>
#include <Master.h>
#include <Kinematic.h>
#include "AICommander.h"

namespace RhobanSSL
{
class AICommanderReal : public AICommander
{
public:
    AICommanderReal(bool yellow);

    virtual void flush();

protected:
    Master master;
    Kinematic kinematic;
};
}
