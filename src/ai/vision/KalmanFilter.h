#include <iostream>
#include <Eigen/Dense>
#include "Ai.h"

using namespace Eigen;

class KalmanFilter{

    int nmbSensors;
    double lastUpdate;
    double dt;

    MatrixXd physicModelFk; //notations are the same as the following document : https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/
    MatrixXd externCmdBk;
    MatrixXd cmdUk;
    MatrixXd predCovariancePk;
    MatrixXd externImpactQk;
    MatrixXd predictedXk; 

    MatrixXd odometryMean;
    MatrixXd odometryGauss;
    MatrixXd videoMean;
    MatrixXd videoGauss;

    MatrixXd sensorGainHk;
    MatrixXd sensorCovarianceRk;
    MatrixXd measurementsZk;
    MatrixXd kalmanGainK;

    MatrixXd filteredPos;
    MatrixXd filteredCov;

    KalmanFilter();
    KalmanFilter(MatrixXd _Fk, MatrixXd _Bk, MatrixXd _Uk, MatrixXd _Pk, MatrixXd _Qk, MatrixXd _Xk, MatrixXd _Hk, MatrixXd _Rk, MatrixXd _Zk, MatrixXd _K, int _nmbSensors);

    void predictPhase();
    void updatePhase();
    void updatedt(int robot_id);
    void kalmanTick();
    void updateCovSensors();
    void fusionSensors(int robot_id);
    
    MatrixXd getActualState();
    double getLastTimeUpdate();

};