#include "mobile.h"


namespace rhoban_ssl {

namespace data {




Mobile::~Mobile()
{
}

using rhoban_geometry::Point;
using rhoban_utils::Angle;


void Mobile::update(double time, const Point& linear_position)
{
  update(time, linear_position, movement[0].angular_position);
}

void Mobile::update(double time, const Point& linear_position, const Angle& angular_position)
{
  ContinuousAngle angle(movement[0].angular_position);
  angle.setToNearest(angular_position);
  update(time, linear_position, angle);
}

void Mobile::update(double time, const Point& linear_position, const ContinuousAngle& angular_position)
{
  if (time <= movement.time(0))
  {
    return;
  }
  last_update = rhoban_utils::TimeStamp::now();
  movement.insert(PositionSample(time, linear_position, angular_position));
}

double Mobile::age() const
{
  return diffSec(last_update, rhoban_utils::TimeStamp::now());
}

bool Mobile::isTooOld() const
{
  return age() > 4.0;
}

bool Mobile::isOk() const
{
  return age() < 2.0;
}

Mobile::Mobile() : last_update(rhoban_utils::TimeStamp::fromMS(0)),movement(history_size)
{
  for (int i = 0; i < history_size; i++)
  {
    movement[i].time = -i;
  }
}

}

}
