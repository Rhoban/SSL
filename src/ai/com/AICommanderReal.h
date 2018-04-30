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
    virtual void kick();

    Master *getMaster();

    virtual ~AICommanderReal();

protected:
    bool kicking;
    Master master;
    Kinematic kinematic;
};
}
