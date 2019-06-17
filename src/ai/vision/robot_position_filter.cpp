#include "robot_position_filter.h"
#include "data.h"

namespace rhoban_ssl
{
namespace vision
{
#define ERROR_FIELD 0.1
#define KALMAN_STATE_VECTOR_SIZE 6             // tuple (position, speed)
#define KALMAN_STATE_SUBVECTOR_SIZE 3          // size of subtuple position/speed -> (x, y, theta)
#define KALMAN_UPDATE_VECTOR_ELEMENTAR_SIZE 3  // position or speed (x, y, theta)

bool objectCoordonateIsValid(double x, double y, vision::PartOfTheField part_of_the_field_used)
{
  switch (part_of_the_field_used)
  {
    case PartOfTheField::POSIVE_HALF_FIELD:
    {
      return x > ERROR_FIELD;
    }
    break;
    case PartOfTheField::NEGATIVE_HALF_FIELD:
    {
      return x < ERROR_FIELD;
    }
    break;
    case PartOfTheField::ALL_FIELD:
    {
      return true;
    }
    break;
    default:;
  }
  return false;
}

void disableTheOrientation(GslMatrix* observation_model, size_t row_offset, size_t col_offset, size_t subvector_length)
{
  for (size_t k = 0; k < subvector_length; k++)
  {
    observation_model->setElement(row_offset + k, col_offset, 0.0);
  }
}

void setupPredictPhaseParams(GslMatrix* physical_model, GslMatrix* process_noise_matrix)
{
  process_noise_matrix->setZero();

  physical_model->setIdentity();
  for (int k = 0; k < KALMAN_STATE_SUBVECTOR_SIZE; k++)
  {
    physical_model->setElement((size_t)k, KALMAN_STATE_SUBVECTOR_SIZE + (size_t)k, ai::Config::period);
  }
}

// warning : this function is specific to the case where STATE_SUBVECTOR_SIZE == UPDATE_VECTOR_ELEMENTAR_SIZE. Behaviour
// in other cases can lead to incorrect setting up of parameters
void setupUpdatePhaseParams(GslMatrix* observation_model, const int camera_number, const bool odom_is_on,
                            GslMatrix* observation_noise_matrix)
{
  observation_noise_matrix->setZero();

  int camera_max = camera_number;
  if (odom_is_on)
  {
    camera_max--;
  }
  for (int camera_count = 0; camera_count < camera_max; camera_count++)
  {
    for (int k = 0; k < KALMAN_STATE_SUBVECTOR_SIZE; k++)
    {
      observation_model->setElement((size_t)(camera_count * KALMAN_UPDATE_VECTOR_ELEMENTAR_SIZE + k), (size_t)k, 1.0);
    }
  }
  observation_model->scaleMatrix(1.0 / ((double)camera_max));  // legal ?

  if (odom_is_on)
  {
    for (int k = 0; k < KALMAN_STATE_SUBVECTOR_SIZE; k++)
    {
      observation_model->setElement((size_t)(camera_max * KALMAN_UPDATE_VECTOR_ELEMENTAR_SIZE + k),
                                    (size_t)(KALMAN_STATE_SUBVECTOR_SIZE + k), 1.0);
    }
  }
}

std::pair<rhoban_geometry::Point, ContinuousAngle>
RobotPositionFilter::kalmanFilter(int robot_id, const SSL_DetectionRobot& robot_frame, bool ally,
                                  const std::map<int, SSL_DetectionFrame>& camera_detections,
                                  bool& orientation_is_defined, PartOfTheField part_of_the_field_used)
{
  rhoban_geometry::Point position(0.0, 0.0);
  ContinuousAngle angle(0.0);
  bool robot_has_orientation = false;  // check whether any camera relay the orientation of the robot
  bool odom_is_on = false;

  /* State matrices for Kalman */
  GslMatrix previous_state = rhoban_ssl::GslMatrix(KALMAN_STATE_VECTOR_SIZE, 1);
  GslMatrix predicted_state = rhoban_ssl::GslMatrix(KALMAN_STATE_VECTOR_SIZE, 1);
  GslMatrix new_state = rhoban_ssl::GslMatrix(KALMAN_STATE_VECTOR_SIZE, 1);

  GslMatrix previous_state_covariance = rhoban_ssl::GslMatrix(KALMAN_STATE_VECTOR_SIZE, KALMAN_STATE_VECTOR_SIZE);
  GslMatrix predicted_state_covariance = rhoban_ssl::GslMatrix(KALMAN_STATE_VECTOR_SIZE, KALMAN_STATE_VECTOR_SIZE);
  GslMatrix new_state_covariance = rhoban_ssl::GslMatrix(KALMAN_STATE_VECTOR_SIZE, KALMAN_STATE_VECTOR_SIZE);

  /* Predict Phase */
  /*** Setup of predict phase required matrices ***/

  GslMatrix physical_model = rhoban_ssl::GslMatrix(KALMAN_STATE_VECTOR_SIZE, KALMAN_STATE_VECTOR_SIZE);

  GslMatrix process_noise_matrix = rhoban_ssl::GslMatrix(KALMAN_STATE_VECTOR_SIZE, KALMAN_STATE_VECTOR_SIZE);

  setupPredictPhaseParams(&physical_model, &process_noise_matrix);

  GslMatrix null_vector = rhoban_ssl::GslMatrix(KALMAN_STATE_VECTOR_SIZE, 1);

  /*** Computing of predicted state ***/
  previous_state.multMatrixBLAS(&predicted_state, &physical_model, &null_vector);
  previous_state.multMatrixBLASKalman(&predicted_state_covariance, &physical_model, &physical_model,
                                      &process_noise_matrix, 1.0, 1.0, false, false, true);

  /* Update Phase */

  /*** Setup of predict phase required matrices ***/

  unsigned int camera_number = 0;
  for (const std::pair<int, SSL_DetectionFrame>& elem : camera_detections)
  {
    camera_number++;
  }
  if (odom_is_on)
  {  // an additional device of measurement
    camera_number++;
  }

  // for observation-related computing
  GslMatrix observation = rhoban_ssl::GslMatrix(KALMAN_UPDATE_VECTOR_ELEMENTAR_SIZE * camera_number, 1);

  GslMatrix observation_model =
      rhoban_ssl::GslMatrix(KALMAN_UPDATE_VECTOR_ELEMENTAR_SIZE * camera_number, KALMAN_STATE_VECTOR_SIZE);
  // Check whether orientation is on or not and set elems at zero when it is off ---> done during observation, for each
  // camera can or not deliver orientation

  GslMatrix observation_noise_matrix = rhoban_ssl::GslMatrix(KALMAN_UPDATE_VECTOR_ELEMENTAR_SIZE * camera_number,
                                                             KALMAN_UPDATE_VECTOR_ELEMENTAR_SIZE * camera_number);

  setupUpdatePhaseParams(&observation_model, camera_number, odom_is_on, &observation_noise_matrix);

  // for innovation computing
  GslMatrix innovation = rhoban_ssl::GslMatrix(KALMAN_UPDATE_VECTOR_ELEMENTAR_SIZE * camera_number, 1);

  GslMatrix innovation_cov_matrix = rhoban_ssl::GslMatrix(KALMAN_UPDATE_VECTOR_ELEMENTAR_SIZE * camera_number,
                                                          KALMAN_UPDATE_VECTOR_ELEMENTAR_SIZE * camera_number);

  GslMatrix inverse_innovation_cov_matrix = rhoban_ssl::GslMatrix(KALMAN_UPDATE_VECTOR_ELEMENTAR_SIZE * camera_number,
                                                                  KALMAN_UPDATE_VECTOR_ELEMENTAR_SIZE * camera_number);

  // for gain computing
  GslMatrix gain = rhoban_ssl::GslMatrix(KALMAN_STATE_VECTOR_SIZE, KALMAN_UPDATE_VECTOR_ELEMENTAR_SIZE * camera_number);

  GslMatrix null_matrix_update =
      rhoban_ssl::GslMatrix(KALMAN_STATE_VECTOR_SIZE, KALMAN_UPDATE_VECTOR_ELEMENTAR_SIZE * camera_number);

  /*** Computing of new state ***/
  for (const std::pair<int, SSL_DetectionFrame>& elem : camera_detections)  // parse cameras
  {
    int camera_id = elem.first;  // does the ID begin at 0 or 1 ? Following code assumes it is 0
    const SSL_DetectionFrame& detection = elem.second;

    const google::protobuf::RepeatedPtrField<SSL_DetectionRobot>* robots;
    if (ai::Config::we_are_blue)
    {
      robots = &detection.robots_blue();
    }
    else
    {
      robots = &detection.robots_yellow();
    }
    for (auto robot : *robots)
    {
      if (!objectCoordonateIsValid(robot.x() / 1000.0, robot.y() / 1000.0, part_of_the_field_used))
      {
        continue;
      }
      if (robot.has_robot_id() and (robot.robot_id() == static_cast<unsigned int>(robot_id)))
      {
        observation.setElement(camera_id * KALMAN_UPDATE_VECTOR_ELEMENTAR_SIZE, 0, robot.x() / 1000.0);
        observation.setElement(camera_id * KALMAN_UPDATE_VECTOR_ELEMENTAR_SIZE + 1, 0, robot.y() / 1000.0);
        robot_has_orientation = robot_has_orientation ||
                                robot.has_orientation();  // is there any camera who has the orientation of the robot ?

        if (robot.has_orientation())  // has THIS SPECIFIC camera the orientation of the robot ?
        {
          observation.setElement(camera_id * KALMAN_UPDATE_VECTOR_ELEMENTAR_SIZE + 2, 0, robot.orientation());
        }
        else
        {
          disableTheOrientation(&observation_model, camera_id * KALMAN_UPDATE_VECTOR_ELEMENTAR_SIZE,
                                KALMAN_STATE_VECTOR_SIZE, KALMAN_UPDATE_VECTOR_ELEMENTAR_SIZE);
        }
        break;
      }
    }
  }

  // here we need to parse odometry !!!!!!!!!!!!!!

  /****** Innovation computing ******/
  predicted_state.multMatrixBLAS(&innovation, &observation_model, &observation, -1.0, 1.0, false, false);
  predicted_state_covariance.multMatrixBLASKalman(&innovation_cov_matrix, &observation_model, &observation_model,
                                                  &observation_noise_matrix, 1.0, 1.0, false, false, true);

  /****** Gain computing ******/
  innovation_cov_matrix.inverseMatrix(&inverse_innovation_cov_matrix);
  observation_model.multMatrixBLASKalman(&gain, &predicted_state, &inverse_innovation_cov_matrix, &null_matrix_update,
                                         1.0, 0.0, false, true, false);

  /****** Final update computing ******/

  innovation.multMatrixBLAS(&new_state, &gain, &predicted_state);
  observation_model.multMatrixBLASKalman(&new_state_covariance, &gain, &predicted_state_covariance,
                                         &predicted_state_covariance, -1.0, 1.0, false, false, false);

  /* Return the results */
  position += rhoban_geometry::Point(new_state.getElement(0, 0), new_state.getElement(1, 0));

  if (!(robot_has_orientation))
  {
    orientation_is_defined = false;
    return { position, ContinuousAngle(0.0) };
  }
  else
  {
    orientation_is_defined = true;
    angle = new_state.getElement(KALMAN_UPDATE_VECTOR_ELEMENTAR_SIZE, 0);
    return { position, angle };
  }
}

std::pair<rhoban_geometry::Point, ContinuousAngle>
RobotPositionFilter::averageFilter(int robot_id, const SSL_DetectionRobot& robot_frame, bool ally,
                                   const std::map<int, SSL_DetectionFrame>& camera_detections,
                                   bool& orientation_is_defined, PartOfTheField part_of_the_field_used)
{
  int n_linear = 0;
  int n_angular = 0;
  rhoban_geometry::Point linear_average(0.0, 0.0);
  ContinuousAngle angular_average(0.0);
  double sina = 0.0;
  double cosa = 0.0;
  for (const std::pair<int, SSL_DetectionFrame>& elem : camera_detections)
  {
    // int camera_id = elem.first;
    const SSL_DetectionFrame& detection = elem.second;

    const google::protobuf::RepeatedPtrField<SSL_DetectionRobot>* robots;
    if (ai::Config::we_are_blue)
    {
      robots = &detection.robots_blue();
    }
    else
    {
      robots = &detection.robots_yellow();
    }
    for (auto robot : *robots)
    {
      if (!objectCoordonateIsValid(robot.x() / 1000.0, robot.y() / 1000.0, part_of_the_field_used))
      {
        continue;
      }
      if (robot.has_robot_id() and (robot.robot_id() == static_cast<unsigned int>(robot_id)))
      {
        linear_average += rhoban_geometry::Point(robot.x() / 1000.0, robot.y() / 1000.0);
        n_linear++;
        if (robot.has_orientation())
        {
          // angular_average += ContinuousAngle(
          //   robot.orientation()
          //   );
          sina += sin(robot.orientation());
          cosa += cos(robot.orientation());

          n_angular++;
        }
        break;
      }
    }
  }
  if (n_angular == 0)
  {
    orientation_is_defined = false;
    return { linear_average * (1.0 / n_linear), ContinuousAngle(0.0) };
  }
  else
  {
    orientation_is_defined = true;
    angular_average = atan2(sina, cosa);

    return { linear_average * (1.0 / n_linear), angular_average };
  }
}

std::pair<rhoban_geometry::Point, ContinuousAngle> RobotPositionFilter::exponentialDegressionFilter(
    int robot_id, const SSL_DetectionRobot& robot_frame, bool ally,
    const std::map<int, SSL_DetectionFrame>& camera_detections, bool& orientation_is_defined)
{
  double n_linear = 0;
  double n_angular = 0;
  rhoban_geometry::Point linear_average(0.0, 0.0);
  ContinuousAngle angular_average(0.0);
  for (const std::pair<int, SSL_DetectionFrame>& elem : camera_detections)
  {
    // int camera_id = elem.first;
    const SSL_DetectionFrame& detection = elem.second;

    const google::protobuf::RepeatedPtrField<SSL_DetectionRobot>* robots;
    const MovementSample& old_robot_movement = Data::get()->robots[Ally][robot_id].movement_sample;
    if (ai::Config::we_are_blue)
    {
      robots = &detection.robots_blue();
    }
    else
    {
      robots = &detection.robots_yellow();
    }
    for (auto robot : *robots)
    {
      if (robot.has_robot_id() and (robot.robot_id() == static_cast<unsigned int>(robot_id)))
      {
        double x = old_robot_movement.linearPosition().getX();
        double alpha = .5;
        double coef = (((x < 0 and robot.x() / 1000.0 < 0) or (x > 0 and robot.x() / 1000.0 > 0)) ?
                           1 - std::exp(-alpha * std::fabs(x)) :
                           std::exp(-alpha * std::fabs(x)));
        linear_average += (rhoban_geometry::Point(robot.x() / 1000.0, robot.y() / 1000.0) * coef);
        n_linear += coef;
        if (robot.has_orientation())
        {
          angular_average += (ContinuousAngle(robot.orientation()) * coef);
          n_angular += coef;
        }
        break;
      }
    }
  }
  if (n_angular == 0)
  {
    orientation_is_defined = false;
    return { linear_average * (1.0 / n_linear), ContinuousAngle(0.0) };
  }
  else
  {
    orientation_is_defined = true;
    return { linear_average * (1.0 / n_linear), angular_average * (1.0 / n_angular) };
  }
}

std::pair<rhoban_geometry::Point, ContinuousAngle>
RobotPositionFilter::noFilter(int robot_id, const SSL_DetectionRobot& robot_frame, bool ally,
                              const std::map<int, SSL_DetectionFrame>& camera_detections, bool& orientation_is_defined)
{
  orientation_is_defined = robot_frame.has_orientation();
  return { rhoban_geometry::Point(robot_frame.x() / 1000.0, robot_frame.y() / 1000.0),
           robot_frame.has_orientation() ? ContinuousAngle(robot_frame.orientation()) : ContinuousAngle(0.0) };
}

}  // namespace vision
}  // namespace rhoban_ssl
