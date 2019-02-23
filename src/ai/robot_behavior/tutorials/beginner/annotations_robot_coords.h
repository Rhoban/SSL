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

#ifndef __ROBOT_BEHAVIOR__TUTORIALS__BEGINNER__ANNOTATIONS_ROBOT_COORDS__H__
#define __ROBOT_BEHAVIOR__TUTORIALS__BEGINNER__ANNOTATIONS_ROBOT_COORDS__H__

#include "robot_behavior/factory.h"
#include "robot_behavior/robot_behavior.h"

namespace RhobanSSL {
namespace Robot_behavior {
namespace Beginner {

class Annotations_Robot_Coords : public RobotBehavior {
   private:
    RhobanSSLAnnotation::Annotations annotations;

   public:
    Annotations_Robot_Coords(Ai::AiData &ai_data);

    virtual void update(
        double time,
        const Ai::Robot &robot,
        const Ai::Ball &ball
    );

    virtual Control control() const;

    virtual RhobanSSLAnnotation::Annotations get_annotations() const;

    virtual ~Annotations_Robot_Coords();
};

};  // namespace Beginner
};  // namespace Robot_behavior
};  // namespace RhobanSSL

#endif
