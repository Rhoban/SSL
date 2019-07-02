#include "factory.h"

namespace rhoban_ssl
{
namespace vision
{
std::pair<rhoban_geometry::Point, ContinuousAngle>
Factory::filter(int robot_id, const SSL_DetectionRobot& robot_frame, bool ally,
                const std::map<int, SSL_DetectionFrame>& camera_detections, bool& orientation_is_defined,
                vision::PartOfTheField part_of_the_field_used)
{
  return RobotPositionFilter::averageFilter(robot_id, robot_frame, ally, camera_detections, orientation_is_defined,
                                            part_of_the_field_used);
  // return Robot_position_filter::exponential_degression_filter(
  //    robot_id, robotFrame, team_color, ally, camera_detections, orientation_is_defined, old_vision_data
  //);
  // return Robot_position_filter::no_filter(
  //    robot_id, robotFrame, team_color, ally, camera_detections, orientation_is_defined, old_vision_data
  //);
}

TimedPosition Factory::filter(RobotDetection** robots)
{
  TimedPosition t;
  double average_time = 0.0;
  rhoban_geometry::Point linear_average(0.0, 0.0);
  ContinuousAngle angular_average(0.0);
  double sina = 0.0;
  double cosa = 0.0;
  int n_angular = 0;
  int cmpt = 0;
  for (uint i = 0; i < ai::Config::NB_CAMERAS; ++i)
  {
    if (robots[i] != nullptr)
    {
      cmpt += 1;
      average_time += robots[i]->camera_->t_capture_;
      linear_average += rhoban_geometry::Point(robots[i]->x_ / 1000.0, robots[i]->y_ / 1000.0);
      if (robots[i]->has_orientation_)
      {
        sina += sin(robots[i]->orientation_);
        cosa += cos(robots[i]->orientation_);
        n_angular++;
      }
    }
  }
  if (cmpt > 0)
  {
    t.time_ = average_time / (double)cmpt;
    t.position_.linear = linear_average / (double)cmpt;
    if (n_angular == 0)
      t.orientation_is_defined_ = false;
    else
    {
      t.orientation_is_defined_ = true;
      angular_average = atan2(sina, cosa);
      t.position_.angular = angular_average;
    }
  }
  return t;
}


/***********************************/
/* START OF KALMAN FILTER CODE */
/***********************************/
double Kalman::kalmanTimePrefilter(CameraDetectionFrame last_camera_detections[ai::Config::NB_CAMERAS]) {
  double current_time = 0.0;
  int nb_cameras = 0;
  for(unsigned int k = 0; k < ai::Config::NB_CAMERAS; ++k) {
    if(last_camera_detections[k].frame_number_ != 0){
      current_time += last_camera_detections[k].t_capture_;
      nb_cameras++;
    }
  }
  if(nb_cameras > 0) {
    current_time = current_time / nb_cameras;
  }
  else {
    current_time = 0.0;
  }
  return current_time;
}

void Kalman::kalmanSpeedPrefilter(vision::RobotDetection* previous_detections[2][ai::Config::NB_OF_ROBOTS_BY_TEAM][ai::Config::NB_CAMERAS], vision::RobotDetection* new_detections[2][ai::Config::NB_OF_ROBOTS_BY_TEAM][ai::Config::NB_CAMERAS], double dt){
  for (int team = 0; team < 2; ++team){
    for (int r = 0; r < ai::Config::NB_OF_ROBOTS_BY_TEAM; ++r){
      for (uint c = 0; c < ai::Config::NB_CAMERAS; ++c){
        if(new_detections[team][r][c] != nullptr) {
          if((previous_detections[team][r][c] != nullptr) && (dt > 0.0)){
            new_detections[team][r][c]->v_x_ = (new_detections[team][r][c]->x_ - previous_detections[team][r][c]->x_) / dt;

            new_detections[team][r][c]->v_y_ = (new_detections[team][r][c]->y_ - new_detections[team][r][c]->y_) / dt;

            if(previous_detections[team][r][c]->has_orientation_ && new_detections[team][r][c]->has_orientation_){
              new_detections[team][r][c]->angular_speed_ = (new_detections[team][r][c]->orientation_ - previous_detections[team][r][c]->orientation_) / dt;
            }
            else {
              new_detections[team][r][c]->angular_speed_ = 0.0;
            }
          }
          else {
            new_detections[team][r][c]->v_x_ = 0.0;
            new_detections[team][r][c]->v_y_ = 0.0;
            new_detections[team][r][c]->angular_speed_ = 0.0;
          }
        }
      }
    }
  }
}


//This function set at 0 a subvector_length-sized submatrix from the observation_model
void Kalman::disableOrientation(GslMatrix* observation_model, size_t row_offset, size_t col_offset, size_t subvector_length)
{
  for (size_t i = 0; i < subvector_length; ++i)
  {
    for(size_t j=0; j<subvector_length; ++j) {
      observation_model->setElement(row_offset + i, col_offset + j, 0.0);
    }
  }
}

void Kalman::setupPredictPhaseParams(GslMatrix* physical_model, GslMatrix* process_noise_matrix, double dt)
{
  process_noise_matrix->setZero();

  physical_model->setIdentity();

  for (int i = 0; i < KALMAN_STATE_VECTOR_SIZE/KALMAN_STATE_SUBVECTOR_SIZE; ++i)
  {
    physical_model->setElement(2*i, 2*i+1, dt);
  }
}

void Kalman::setupUpdatePhaseParams(GslMatrix* observation_model, const int camera_number,
                            GslMatrix* observation_noise_matrix)
{
  observation_noise_matrix->setIdentity();
  observation_noise_matrix->scaleMatrix(0.1);

  for (int camera_count = 0; camera_count < camera_number; camera_count++)
  {
    for (int k = 0; k < KALMAN_OBSERVATION_VECTOR_ELEMENTAR_SIZE; k++)
    {
      observation_model->setElement((size_t)(camera_count * KALMAN_OBSERVATION_VECTOR_ELEMENTAR_SIZE + k), (size_t)k, 1.0);
    }
  }
}


//Rq : kalman does not include Odometry for now. To include odometry, setting the related parameters into a RobotDetection which will fit the array given in argument to kalman, and then increasing the related loops from NB_CAMERAS to NB_CAMERAS + 1 should do the job
TimedPosition Kalman::kalmanFilter(RobotDetection** robot_views, double cadence_time)
{
  TimedPosition t;
  rhoban_geometry::Point position(0.0, 0.0);
  ContinuousAngle angle(0.0);
  double cosa = 0.0;
  double sina = 0.0;
  double dt = cadence_time - previous_kalman_execution_time_;
  previous_kalman_execution_time_ = cadence_time;
  bool robot_has_orientation = false;  //will serve to check whether any camera relay the orientation of the robot

  /* State matrices for Kalman */
  GslMatrix previous_state = rhoban_ssl::GslMatrix(KALMAN_STATE_VECTOR_SIZE, 1);
  previous_state_.copyMatrix(&previous_state);
  GslMatrix predicted_state = rhoban_ssl::GslMatrix(KALMAN_STATE_VECTOR_SIZE, 1);
  GslMatrix new_state = rhoban_ssl::GslMatrix(KALMAN_STATE_VECTOR_SIZE, 1);

  GslMatrix previous_state_covariance = rhoban_ssl::GslMatrix(KALMAN_STATE_VECTOR_SIZE, KALMAN_STATE_VECTOR_SIZE);
  previous_state_covariance_.copyMatrix(&previous_state_covariance);
  GslMatrix predicted_state_covariance = rhoban_ssl::GslMatrix(KALMAN_STATE_VECTOR_SIZE, KALMAN_STATE_VECTOR_SIZE);
  GslMatrix new_state_covariance = rhoban_ssl::GslMatrix(KALMAN_STATE_VECTOR_SIZE, KALMAN_STATE_VECTOR_SIZE);

  /* Predict Phase */
  /*** Setup of predict phase required matrices ***/

  GslMatrix physical_model = rhoban_ssl::GslMatrix(KALMAN_STATE_VECTOR_SIZE, KALMAN_STATE_VECTOR_SIZE);

  GslMatrix process_noise_matrix = rhoban_ssl::GslMatrix(KALMAN_STATE_VECTOR_SIZE, KALMAN_STATE_VECTOR_SIZE);

  Kalman::setupPredictPhaseParams(&physical_model, &process_noise_matrix, dt);

  GslMatrix null_vector = rhoban_ssl::GslMatrix(KALMAN_STATE_VECTOR_SIZE, 1);

  /*** Computing of predicted state ***/

  //printf("Computing of predicted state...");

  previous_state.multMatrixBLAS(&predicted_state, &physical_model, &null_vector);
  previous_state_covariance.multMatrixBLASKalman(&predicted_state_covariance, &physical_model, &physical_model,
                                      &process_noise_matrix, 1.0, 1.0, false, false, true);

  /* Update Phase */

  /*** Setup of predict phase required matrices ***/

  unsigned int camera_number = 0;
  for(unsigned int i = 0; i< ai::Config::NB_CAMERAS; ++i) {
    if (robot_views[i] != nullptr)
    { 
      camera_number++;
    }
  }

  if(camera_number != 0){
    // for observation-related computing
    GslMatrix observation = rhoban_ssl::GslMatrix(KALMAN_OBSERVATION_VECTOR_ELEMENTAR_SIZE * camera_number, 1);

    GslMatrix observation_model =
        rhoban_ssl::GslMatrix(KALMAN_OBSERVATION_VECTOR_ELEMENTAR_SIZE * camera_number, KALMAN_STATE_VECTOR_SIZE);
    // It is necessary to check whether orientation is on or not for each camera view and set matrix' elems at zero when it is off ---> done during observation, for each camera can or not deliver orientation

    GslMatrix observation_noise_matrix = rhoban_ssl::GslMatrix(KALMAN_OBSERVATION_VECTOR_ELEMENTAR_SIZE * camera_number,
                                                              KALMAN_OBSERVATION_VECTOR_ELEMENTAR_SIZE * camera_number);
    Kalman::setupUpdatePhaseParams(&observation_model, camera_number, &observation_noise_matrix);

    // for innovation computing
    GslMatrix innovation = rhoban_ssl::GslMatrix(KALMAN_OBSERVATION_VECTOR_ELEMENTAR_SIZE * camera_number, 1);

    GslMatrix innovation_cov_matrix = rhoban_ssl::GslMatrix(KALMAN_OBSERVATION_VECTOR_ELEMENTAR_SIZE * camera_number,
                                                            KALMAN_OBSERVATION_VECTOR_ELEMENTAR_SIZE * camera_number);

    GslMatrix inverse_innovation_cov_matrix = rhoban_ssl::GslMatrix(KALMAN_OBSERVATION_VECTOR_ELEMENTAR_SIZE * camera_number,
                                                                    KALMAN_OBSERVATION_VECTOR_ELEMENTAR_SIZE * camera_number);

    // for gain computing
    GslMatrix gain = rhoban_ssl::GslMatrix(KALMAN_STATE_VECTOR_SIZE, KALMAN_OBSERVATION_VECTOR_ELEMENTAR_SIZE * camera_number);

    GslMatrix null_matrix_update =
        rhoban_ssl::GslMatrix(KALMAN_STATE_VECTOR_SIZE, KALMAN_OBSERVATION_VECTOR_ELEMENTAR_SIZE * camera_number);

    /*** Computing of new state ***/

    //printf("Computing of new state...");
    int counter = 0; //count active cameras, that is, robot_views whose first elem is not a nullptr
    for(unsigned int i = 0; i< ai::Config::NB_CAMERAS; ++i) {

      RobotDetection* robot_view = robot_views[i];
      if ((robot_view != nullptr) && (robot_view->has_id_)) //Discard every camera view but the most recent one (the last in the table)
      {
        observation.setElement(counter * KALMAN_OBSERVATION_VECTOR_ELEMENTAR_SIZE, 0, robot_view->x_ / 1000.0);
        observation.setElement(counter * KALMAN_OBSERVATION_VECTOR_ELEMENTAR_SIZE + 1, 0, robot_view->v_x_ / 1000.0);

        observation.setElement(counter * KALMAN_OBSERVATION_VECTOR_ELEMENTAR_SIZE + 2, 0, robot_view->y_ / 1000.0);
        observation.setElement(counter * KALMAN_OBSERVATION_VECTOR_ELEMENTAR_SIZE + 3, 0, robot_view->v_y_ / 1000.0);


        robot_has_orientation = robot_has_orientation ||
                                robot_view->has_orientation_ ; // is there view from any camera who has the orientation of the robot ?

        if (robot_view->has_orientation_) { // has THIS SPECIFIC camera view the orientation of the robot ?
          /*observation.setElement(counter * KALMAN_OBSERVATION_VECTOR_ELEMENTAR_SIZE + 4, 0, robot_view.orientation_);
          observation.setElement(counter * KALMAN_OBSERVATION_VECTOR_ELEMENTAR_SIZE + 5, 0, robot_view.angular_speed_); DEACTIVATED FOR NOW*/
          cosa += cos(robot_view->orientation_);
          sina += sin(robot_view->orientation_);
        }
        else
        {/*
          disableOrientation(&observation_model, counter * KALMAN_OBSERVATION_VECTOR_ELEMENTAR_SIZE + 4, 4, KALMAN_STATE_SUBVECTOR_SIZE); DEACTIVATED FOR NOW*/
        }
      
        ++counter;
      }
    }


    /****** Innovation computing ******/

    predicted_state.multMatrixBLAS(&innovation, &observation_model, &observation, -1.0, 1.0, false, false);
    predicted_state_covariance.multMatrixBLASKalman(&innovation_cov_matrix, &observation_model, &observation_model,
                                                    &observation_noise_matrix, 1.0, 1.0, false, false, true);

    /****** Gain computing ******/

    innovation_cov_matrix.inverseMatrix(&inverse_innovation_cov_matrix);
    observation_model.multMatrixBLASKalman(&gain, &predicted_state_covariance, &inverse_innovation_cov_matrix, &null_matrix_update,
                                          1.0, 0.0, false, true, false);

    /****** Final update computing ******/

    innovation.multMatrixBLAS(&new_state, &gain, &predicted_state);
    observation_model.multMatrixBLASKalman(&new_state_covariance, &gain, &predicted_state_covariance,
                                          &predicted_state_covariance, -1.0, 1.0, false, false, false);
  }
  else {

    predicted_state.copyMatrix(&new_state);
    predicted_state_covariance.copyMatrix(&new_state_covariance);
  }

  /* Return the results */
  position += rhoban_geometry::Point(new_state.getElement(0, 0), new_state.getElement(2, 0));
  if (!(robot_has_orientation))
  {

    t.orientation_is_defined_ = false;
    t.time_ = cadence_time;
    t.position_.linear = position;
    t.position_.angular = angle;
    previous_kalman_execution_time_ = cadence_time;
    new_state.copyMatrix(&previous_state_);
    new_state_covariance.copyMatrix(&previous_state_covariance_);

    return t;
  }
  else
  {

    //angle += ContinuousAngle(new_state.getElement(4, 0));
    angle += ContinuousAngle(atan2(sina, cosa));
    t.orientation_is_defined_ = false;
    t.time_ = cadence_time;    
    t.position_.linear = position;
    t.position_.angular = angle;
    previous_kalman_execution_time_ = cadence_time;
    new_state.copyMatrix(&previous_state_);
    new_state_covariance.copyMatrix(&previous_state_covariance_);

    return t;
  }
}

Kalman::Kalman() : previous_state_(KALMAN_STATE_VECTOR_SIZE, 1), previous_state_covariance_(KALMAN_STATE_VECTOR_SIZE, KALMAN_STATE_VECTOR_SIZE), previous_kalman_execution_time_(0.0)  {
  previous_state_covariance_.setIdentity();
}

Kalman::~Kalman(){}

TimedPosition::TimedPosition() : time_(0), orientation_is_defined_(false)
{
}

}  // namespace vision
}  // namespace rhoban_ssl
