#include "mobile.h"
#include <physic/movement_on_new_frame.h>
#include <physic/factory.h>
#include <data.h>

namespace rhoban_ssl
{
namespace data
{
Mobile::~Mobile()
{
  delete movement;
}

using rhoban_geometry::Point;
using rhoban_utils::Angle;

void Mobile::update(double time, const Point& linear_position)
{
  update(time, linear_position, movement_sample[0].angular_position);
}

void Mobile::update(double time, const Point& linear_position, const Angle& angular_position)
{
  ContinuousAngle angle(movement_sample[0].angular_position);
  angle.setToNearest(angular_position);
  update(time, linear_position, angle);
}

void Mobile::update(double time, const Point& linear_position, const ContinuousAngle& angular_position)
{
  if (time <= movement_sample.time(0))
  {
    return;
  }
  last_update = rhoban_utils::TimeStamp::now();
  movement_sample.insert(PositionSample(time, linear_position, angular_position));
  updateVisionData();
}

double Mobile::age() const
{
  return diffSec(last_update, rhoban_utils::TimeStamp::now());
}

bool Mobile::isTooOld() const
{
  return age() > 4.0;
}

void Mobile::changeFrame(const rhoban_geometry::Point& origin, const Vector2d& v1, const Vector2d& v2)
{
  static_cast<MovementOnNewFrame*>(movement)->setFrame(origin, v1, v2);
}

void Mobile::setMovement(Movement* movement)
{
  if (this->movement)
  {
    assert(movement != static_cast<MovementOnNewFrame*>(this->movement)->getOriginalMovement());
    delete this->movement;
  }
  // We change the frame according referee informatiosns
  this->movement = new MovementOnNewFrame(movement);
}

const Movement& Mobile::getMovement() const
{
  return *movement;
}

void Mobile::updateVisionData()
{
  this->movement->setSample(movement_sample);
}

bool Mobile::isActive() const
{
  return age() < 2.0;
}

Mobile::Mobile() : last_update(rhoban_utils::TimeStamp::now()), movement_sample(history_size), movement(nullptr)
{
}

void Mobile::initMovement()
{
  if (movement == nullptr)
  {
    movement = physic::Factory::movement();
    for (int i = 0; i < history_size; i++)
    {
      movement_sample[i].time = -i;
    }
    updateVisionData();
  }
}

bool Mobile::isInsideTheField()
{
  return Data::get()->field.isInside(movement->linearPosition(Data::get()->ai_data.time));
}

}  // namespace data
}  // namespace rhoban_ssl
