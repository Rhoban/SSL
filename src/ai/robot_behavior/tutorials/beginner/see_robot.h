/*
    This file is part of SSL.

    Copyright 2019 RomainPC (romainpc.lechat@laposte.net)

    SSL is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SSL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with SSL.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __ROBOT_BEHAVIOR__TUTORIALS__BEGINNER__SEE_ROBOT__H__
#define __ROBOT_BEHAVIOR__TUTORIALS__BEGINNER__SEE_ROBOT__H__

#include "robot_behavior/factory.h"
#include "robot_behavior/robot_behavior.h"

namespace RhobanSSL {
namespace Robot_behavior {
namespace Beginner {

class See_Robot : public RobotBehavior {
   private:
    int target_robot_id;
    ConsignFollower *follower;
    RhobanSSLAnnotation::Annotations annotations;

   public:
    See_Robot(Ai::AiData &ai_data);
    // REVIEW AB : Use just one constructor with a default parameter.
    See_Robot(Ai::AiData &ai_data, int target_id);

    virtual void update(
        double time,
        const Ai::Robot &robot,
        const Ai::Ball &ball
    );

    virtual Control control() const;

    void set_robot_id_to_see (int id);

    // REVIEW AB : This function should be declared as const function.
    int get_robot_id_to_see();

    virtual RhobanSSLAnnotation::Annotations get_annotations() const;

    virtual ~See_Robot();
};

};  // namespace Beginner
};  // namespace Robot_behavior
};  // namespace RhobanSSL

#endif
