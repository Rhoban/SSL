#pragma once

#include <VisionClient.h>
#include <math/continuous_angle.h>
#include <math/gsl/gsl_matrix.h>
#include "vision_data.h"

namespace rhoban_ssl
{
namespace vision
{
enum PartOfTheField
{
  POSIVE_HALF_FIELD,
  NEGATIVE_HALF_FIELD,
  ALL_FIELD
};

bool objectCoordonateIsValid(double x, double y, PartOfTheField part_of_the_field_used);

void disableTheOrientation(GslMatrix* observation_model, size_t row_offset, size_t col_offset, size_t subvector_length);

void setupPredictPhaseParams(GslMatrix* physical_model, GslMatrix* process_noise_matrix);

void setupUpdatePhaseParams(GslMatrix* observation_model, const int camera_number, const bool is_odom_on,
                            GslMatrix* observation_noise_matrix);

class RobotPositionFilter
{
public:
  // static matrix physicalModel = ai::Config::physicalModelConfig;
  static std::pair<rhoban_geometry::Point, ContinuousAngle>
  kalmanFilter(int robot_id, const SSL_DetectionRobot& robot_frame, bool ally,
               const std::map<int, SSL_DetectionFrame>& camera_detections, bool& orientation_is_defined,
               vision::PartOfTheField part_of_the_field_used);

  static std::pair<rhoban_geometry::Point, ContinuousAngle>
  averageFilter(int robot_id, const SSL_DetectionRobot& robot_frame, bool ally,
                const std::map<int, SSL_DetectionFrame>& camera_detections, bool& orientation_is_defined,
                vision::PartOfTheField part_of_the_field_used);

  static std::pair<rhoban_geometry::Point, ContinuousAngle>
  exponentialDegressionFilter(int robot_id, const SSL_DetectionRobot& robot_frame, bool ally,
                              const std::map<int, SSL_DetectionFrame>& camera_detections, bool& orientation_is_defined);

  static std::pair<rhoban_geometry::Point, ContinuousAngle>
  noFilter(int robot_id, const SSL_DetectionRobot& robot_frame, bool ally,
           const std::map<int, SSL_DetectionFrame>& camera_detections, bool& orientation_is_defined);
};

};  // namespace vision
};  // namespace rhoban_ssl
