#include "KalmanFilter.h"




KalmanFilter::KalmanFilter(){

}

KalmanFilter::KalmanFilter(MatrixXd _Fk, MatrixXd _Bk, MatrixXd _Uk,
                           MatrixXd _Pk,MatrixXd _Qk, MatrixXd _Xk, 
                           MatrixXd _Hk, MatrixXd _Rk, MatrixXd _Zk, 
                           MatrixXd _K, int _nmbSensors)
{
    physicModelFk      = _Fk;
    externCmdBk        = _Bk;
    cmdUk              = _Uk;
    predCovariancePk   = _Pk;
    externImpactQk     = _Qk;
    predictedXk        = _Xk; 
    sensorGainHk       = _Hk;
    sensorCovarianceRk = _Rk;
    measurementsZk     = _Zk;
    kalmanGainK        = _K;

    nmbSensors = _nmbSensors;
    lastUpdate = 0.0;
    dt         = 0.0;
    
}

void KalmanFilter::updatedt(){
    dt = getCurrentTime()-lastUpdate;
}
void KalmanFilter::predictPhase(){  
    predictedXk =  physicModelFk*filteredPos + externCmdBk*cmdUk;
    predCovariancePk = physicModelFk*filteredCov*(physicModelFk.transpose()) + externImpactQk;
}

void KalmanFilter::updatePhase(){
    filteredPos = predictedXk + kalmanGainK*(measurementsZk-sensorGainHk*predictedXk);

    kalmanGainK = predCovariancePk*(sensorGainHk.transpose())*((sensorGainHk*predCovariancePk*sensorGainHk.transpose()+sensorCovarianceRk).inverse());

    filteredCov = predCovariancePk - kalmanGainK*sensorGainHk*predCovariancePk;
    lastUpdate  = getCurrentTime();
}

void KalmanFilter::kalmanTick(int robot_id){
    updatedt(robot_id);
    predictPhase(robot_id);
    fusionSensors(robot_id);
    updatePhase(robot_id);
}
    

MatrixXd KalmanFilter::getActualState(){

}

double KalmanFilter::getLastTimeUpdate(){

}

void KalmanFilter::fusionSensors(int robot_id){

    std::pair< rhoban_geometry::Point, ContinuousAngle > vision = Vision::Factory::filter( //Gather video information
            robotFrame.robot_id(), robotFrame, team_color, ally, camera_detections,
            orientation_is_defined, 
            oldVisionData, part_of_the_field_used
            );
    
    double video_timestamp = robots[robot_id].lastUpdate;
    
    MatrixXd odom(3.1);
    odom(0, 0) = robots[robot_id].status.xpos;
    odom(1, 0) = robots[robot_id].status.ypos;
    odom(2, 0) = robots[robot_id].status.ang;
    odometryMean = odom;

    MatrixXd video(3.1);
    video(0, 0) = std::get<0>(vision).x;
    video(1, 0) = std::get<0>(vision).y;
    video(2, 0) = std::get<1>(vision);
    videoMean = video;

    MatrixXd odomGauss(3,3);//the idea is to modify the covariance with the fact that
    odomGauss(0, 0) = 300;  //the time elapsed between two video frames are variable 
    odomGauss(0, 1) = 0;    //if this value is too high, the covariance is huge.
    odomGauss(0, 2) = 0;    //With the Odometry, the concept is that we can reset it
    odomGauss(1, 0) = 0;    //every time a video frame is gathered (modulo the delay).
    odomGauss(1, 1) = 300;  //Thanks to that, it is possible to know if the Odometry
    odomGauss(1, 2) = 0;    //has been reseted a long time ago. If yes, the covariance will
    odomGauss(2, 0) = 0;    //be huge as well.
    odomGauss(2, 1) = 0;
    odomGauss(2, 2) = 300;
    odometryGauss = odomGauss;

    //MatrixXd videoGauss(3,3);
    videoGauss = odomGauss;

    matrixXd K(6,6);


}