#pragma once

#include "physic/movement_sample.h"
#include "physic/movement.h"
#include <rhoban_utils/timing/time_stamp.h>

namespace rhoban_ssl
{
namespace data
{
//! TODO: remove hisotry_size ...
static const int history_size = 10;

class Mobile
{
  /*!
   * \brief last_update corresponds to last call to an update method... not related to game timing information.
   */
  rhoban_utils::TimeStamp last_update;

public:
  MovementSample movement_sample;
  Movement* movement;

  void update(double time, const rhoban_geometry::Point& linear_position, const rhoban_utils::Angle& angular_position);
  void update(double time, const rhoban_geometry::Point& linear_position, const ContinuousAngle& angular_position);
  void update(double time, const rhoban_geometry::Point& linear_position);

  /*!
   * \brief age return the time elapsed since last call to an update method
   * \return a duration in seconds
   */
  double age() const;

  /**
   * @brief isOk returns true if the mobile is active
   *
   * true if informations has been updated since less than 2 seconds
   *
   * @note before to be active robot needed to be PresentInVision and InsideTheField
   * @return a bool
   */
  bool isActive() const;

  /*!
   * \brief isTooOld object has not been update since 4 seconds
   * \return
   */
  bool isTooOld() const;

  /**
   * @brief changeFrame
   *
   * @todo Move in the update in vision
   * @param origin
   * @param v1
   * @param v2
   */
  void changeFrame(const rhoban_geometry::Point& origin, const Vector2d& v1, const Vector2d& v2);

  void setMovement(Movement* movement);

  const Movement& getMovement() const;
  /**
   * @brief setVisionData
   * @todo avoid useless copy of movement_sample
   * check Movement mechanisms
   * @param vision_data
   */
  void updateVisionData();

  /**
   * @brief isInsideTheField
   * @return
   */
  bool isInsideTheField();

  void initMovement();

  Mobile();
  virtual ~Mobile();
};
}  // namespace data

}  // namespace rhoban_ssl
