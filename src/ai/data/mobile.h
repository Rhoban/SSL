#pragma once

#include "physic/movement_sample.h"
#include <rhoban_utils/timing/time_stamp.h>

namespace rhoban_ssl {

namespace data {

//!TODO: remove hisotry_size ...
static const int history_size = 10;


class Mobile
{
  /*!
   * \brief last_update corresponds to last call to an update method... not related to game timing information.
   */
  rhoban_utils::TimeStamp last_update;
  /*!
   * \brief age return the time elapsed since last call to an update method
   * \return a duration in seconds
   */
  double age() const;

public:
  MovementSample movement;

  void update(double time, const rhoban_geometry::Point& linear_position, const rhoban_utils::Angle& angular_position);
  void update(double time, const rhoban_geometry::Point& linear_position, const ContinuousAngle& angular_position);
  void update(double time, const rhoban_geometry::Point& linear_position);

  /*!
   * \brief isOk object informations has been updated since less than 2 seconds
   * \return
   */
  bool isOk() const;
  /*!
   * \brief isTooOld object has not been update since 4 seconds
   * \return
   */
  bool isTooOld() const;

  Mobile();
  virtual ~Mobile();
};
}

}
