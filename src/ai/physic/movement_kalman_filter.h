#ifndef __MOVEMENT_KALMAN_FILTER_H__ 
#define __MOVEMENT_KALMAN_FILTER_H__ 

#include <iostream>
#include <Eigen/Dense>
#include <physic/Movement.h>


// namespace Eigen {
namespace RhobanSSL {

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
//typedef Eigen::Matrix<double, 6, 3> Matrix63d;
//typedef Eigen::Matrix<double, 6, 1> Vector6d;
double cont2signrad(ContinuousAngle angle);

class Movement_kalman_filter : public Movement {

    private:
        MovementSample samples[3]; //0=video 1=odom 2=kalman_result
        OrdersSample ordersSamples;
        bool odomOff;

        double lastUpdate;
        double dt;

        Matrix6d physicModelFk; //notations are the same as the following document : https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/
        Eigen::MatrixXd externCmdBk;
        Eigen::MatrixXd cmdUk;
        Matrix6d predCovariancePk;
        Matrix6d externImpactQk;
        Eigen::MatrixXd predictedXk; 

        Matrix6d odometryMean;
        Matrix6d odometryGauss;
        Matrix6d videoMean;
        Matrix6d videoGauss;

        Matrix6d sensorGainHk;
        Matrix6d sensorCovarianceRk;
        Eigen::MatrixXd measurementsZk;
        Matrix6d kalmanGainK;

        Eigen::MatrixXd filteredPos;
        Matrix6d filteredCov;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Movement_kalman_filter();
        //Movement_kalman_filter(Eigen::MatrixXd _Fk, Eigen::MatrixXd _Bk, Eigen::MatrixXd _Uk, Eigen::MatrixXd _Pk, Eigen::MatrixXd _Qk, Eigen::MatrixXd _Xk, Eigen::MatrixXd _Hk, Eigen::MatrixXd _Rk, Eigen::MatrixXd _Zk, Eigen::MatrixXd _K, int _nmbSensors);

        virtual Movement * clone() const;

        virtual double last_time() const;

        virtual void set_sample( const MovementSample & samples, unsigned int i=0 );
        virtual void set_orders_sample( const OrdersSample & samples);
        virtual const MovementSample & get_sample(unsigned int i=0) const;

        virtual rhoban_geometry::Point linear_position( double _time ) const;
        virtual ContinuousAngle angular_position( double _time ) const;

        virtual Vector2d linear_velocity( double _time ) const;
        virtual ContinuousAngle angular_velocity( double _time ) const;

        virtual Vector2d linear_acceleration( double _time ) const;
        virtual ContinuousAngle angular_acceleration( double _time ) const;

        virtual void print(std::ostream& stream) const;

        virtual ~Movement_kalman_filter();



        void kalman_tick(double _time);
        void predictPhase(double _time);
        void updatePhase(double _time);
        void fusionSensors(double _time);

};

// }
}

#endif