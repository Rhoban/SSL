#include "movement_kalman_filter.h"
#include <debug.h> 
#include <cmath>
#define POIDS 4  //weight of the robot in kg

namespace RhobanSSL{

Movement_kalman_filter::Movement_kalman_filter(){

    samples[0]    =  MovementSample(10);
    samples[1]    =  MovementSample(10);
    samples[2]    =  MovementSample(10);
    ordersSamples =  OrdersSample(10);
    odomOff       = true;

    dt = 0.0;
    lastUpdate = 0.0;
    externCmdBk.resize(6,3);
    cmdUk.resize(3,1);
    filteredPos.resize(6,1);

    physicModelFk << 1, 0, 0, dt, 0, 0,  //Maybe improve it with Romain physic model, that's just a first attempt
                     0, 1, 0, 0, dt, 0, 
                     0, 0, 1, 0, 0, dt, 
                     0, 0, 0, 1, 0, 0, 
                     0, 0, 0, 0, 1, 0, 
                     0, 0, 0, 0, 0, 1;


    externCmdBk   << (std::pow(dt,2)/(2*POIDS)), 0, 0,  //same here
                     0, (std::pow(dt,2)/(2*POIDS)), 0,
                     0, 0, (std::pow(dt,2)/(2*POIDS)),
                     dt, 0, 0,
                     0, dt, 0,
                     0, 0, dt;

    cmdUk = Eigen::MatrixXd::Zero(3,1); //Update with the order given to the robots
    predCovariancePk   = Eigen::MatrixXd::Identity(6,6); //Covariance supposed by the model
    externImpactQk = Eigen::MatrixXd::Zero(6,6); //Extern impact on the Covariance of the model
    predictedXk = Eigen::MatrixXd::Zero(6,6); //Result of the predictphase
    sensorGainHk = Eigen::MatrixXd::Identity(6,6); //Gain of each sensor (equal to 1 in our case i guess)
    sensorCovarianceRk = Eigen::MatrixXd::Zero(6,6); //Covariance (noise) around our sensors (Gaussian variance of all the sensors combined)
    measurementsZk = Eigen::MatrixXd::Zero(6,6); //Values read by the sensors (mean of all the values considered)
    kalmanGainK = Eigen::MatrixXd::Identity(6,6); //Kalman Gain (classic gain of a filter)
    filteredPos = Eigen::MatrixXd::Zero(6,1);
    filteredCov = Eigen::MatrixXd::Zero(6,6);

}

Movement * Movement_kalman_filter::clone() const{
    Movement_kalman_filter * res = new Movement_kalman_filter();
    *res = *this;
    return res;
}

double Movement_kalman_filter::last_time() const{
    return samples[2].time(0);
}

void Movement_kalman_filter::set_sample( const MovementSample & samples, unsigned int i){
    assert( samples.is_valid() );
    assert((i<2));

    if(this->samples[2].time() == 0.0){
        this->samples[2] = samples;
    }

    dt = samples.time() - this->samples[2].time();
    this->samples[i] = samples;
    if(this->samples[1].time() != 0.0){
        odomOff = false;
    }

    if((i != 2) && !(odomOff)){
        kalman_tick(samples.time());
    }
}

void Movement_kalman_filter::set_orders_sample( const OrdersSample & samples){
    this->ordersSamples = samples;
}


const MovementSample & Movement_kalman_filter::get_sample(unsigned int i) const{
    assert((i<=2));
    return this->samples[i];
}

rhoban_geometry::Point Movement_kalman_filter::linear_position( double _time ) const{
    if(samples[1].time() == 0.0){ //HACK in case there is no Odometry, just release video sample
        if( std::fabs( samples[0].time() - _time ) <= 0.000001 ){
            _time = samples[0].time();
        }
        double dt = _time - samples[0].time(0);

        return (
        samples[0].linear_position(0) + samples[0].linear_velocity(0) * dt// + samples.linear_acceleration(0) * dt*dt/2.0
        );
    }

    if(((samples[2].time()) - _time) < 0){
        if(samples[2].time() > samples[0].time()){//it means kalman is fresher than video
            return samples[2].linear_position();
        }
        else{
            return samples[0].linear_position();
        }
        
    }
    else{                                     //it means that the last values is close enough to the required time
        return samples[2].linear_position(); //Possible hack : also check video sample and if close enough return the video sample
    }
}

ContinuousAngle Movement_kalman_filter::angular_position( double _time ) const{
    if(samples[1].time() == 0.0){ //HACK in case there is no Odometry, just release video sample
        if( std::fabs( samples[0].time() - _time ) <= 0.000001 ){
            _time = samples[0].time();
        }
        double dt = _time - samples[0].time(0);

        return (
        samples[0].angular_position(0) + samples[0].angular_velocity(0) * dt// + samples.linear_acceleration(0) * dt*dt/2.0
        );
    }
    
    
    if(((samples[2].time()) - _time) < 0){
        if(samples[2].time() > samples[0].time()){//it means kalman is fresher than video
            return samples[2].angular_position();
        }
        else{
            return samples[0].angular_position();
        }
        
    }
    else{                                     //it means that the last values is close enough to the required time
        return samples[2].angular_position(); //Possible hack : also check video sample and if close enough return the video sample
    }
}

Vector2d Movement_kalman_filter::linear_velocity( double _time ) const{
    if(samples[1].time() == 0.0){ //HACK in case there is no Odometry, just release video sample
        if( std::fabs( samples[0].time() - _time ) <= 0.000001 ){
            _time = samples[0].time();
        }
        double dt = _time - samples[0].time(0);

        return (
        samples[0].linear_velocity(0) + samples[0].linear_acceleration(0) * dt// + samples.linear_acceleration(0) * dt*dt/2.0
        );
    }
    
    
    if(((samples[2].time()) - _time) < 0){
        if(samples[2].time() > samples[0].time()){//it means kalman is fresher than video
            return samples[2].linear_velocity();
        }
        else{
            return samples[0].linear_velocity();
        }
        
    }
    else{                                     //it means that the last values is close enough to the required time
        return samples[2].linear_velocity(); //Possible hack : also check video sample and if close enough return the video sample
    }
}

ContinuousAngle Movement_kalman_filter::angular_velocity( double _time ) const{
    if(samples[1].time() == 0.0){ //HACK in case there is no Odometry, just release video sample
        if( std::fabs( samples[0].time() - _time ) <= 0.000001 ){
            _time = samples[0].time();
        }
        double dt = _time - samples[0].time(0);

        return (
        samples[0].angular_velocity(0) + samples[0].angular_acceleration(0) * dt// + samples.linear_acceleration(0) * dt*dt/2.0
        );
    }
    
    
    
    if(((samples[2].time()) - _time) < 0){//it means that we have to recalculate it. We will calculate everything at the same time so one call will update all the others values
        if(samples[2].time() > samples[0].time()){//it means kalman is fresher than video
            return samples[2].angular_velocity();
        }
        else{
            return samples[0].angular_velocity();
        }
        
    }
    else{                                     //it means that the last values is close enough to the required time
        return samples[2].angular_velocity(); //Possible hack : also check video sample and if close enough return the video sample
    }
}

Vector2d Movement_kalman_filter::linear_acceleration( double _time ) const{
    if(samples[1].time() == 0.0){ //HACK in case there is no Odometry, just release video sample
        if( std::fabs( samples[0].time() - _time ) <= 0.000001 ){
            _time = samples[0].time();
        }
        double dt = _time - samples[0].time(0);

        return (
        samples[0].linear_acceleration(0) // + samples.linear_acceleration(0) * dt*dt/2.0
        );
    }


    if(((samples[2].time()) - _time) < 0){//it means that we have to recalculate it. We will calculate everything at the same time so one call will update all the others values
        if(samples[2].time() > samples[0].time()){//it means kalman is fresher than video
            return samples[2].linear_acceleration();
        }
        else{
            return samples[0].linear_acceleration();
        }
        
    }
    else{                                     //it means that the last values is close enough to the required time
        return samples[2].linear_acceleration(); //Possible hack : also check video sample and if close enough return the video sample
    }
}

ContinuousAngle Movement_kalman_filter::angular_acceleration( double _time ) const{
    if(samples[1].time() == 0.0){ //HACK in case there is no Odometry, just release video sample
        if( std::fabs( samples[0].time() - _time ) <= 0.000001 ){
            _time = samples[0].time();
        }
        double dt = _time - samples[0].time(0);

        return (
        samples[0].angular_acceleration(0) // + samples.linear_acceleration(0) * dt*dt/2.0
        );
    }
    
    if(((samples[2].time()) - _time) < 0){//it means that we have to recalculate it. We will calculate everything at the same time so one call will update all the others values
        if(samples[2].time() > samples[0].time()){//it means kalman is fresher than video
            return samples[2].angular_acceleration();
        }
        else{
            return samples[0].angular_acceleration();
        }
        
    }
    else{                                     //it means that the last values is close enough to the required time
        return samples[2].angular_acceleration(); //Possible hack : also check video sample and if close enough return the video sample
    }
}

void Movement_kalman_filter::print(std::ostream& stream) const{
    stream << samples[0] << samples[1];
}   

void Movement_kalman_filter::predictPhase(double _time){
    
    physicModelFk  << 1, 0, 0, dt, 0, 0,  //Maybe improve it with Romain physic model, that's just a first attempt
                     0, 1, 0, 0, dt, 0, 
                     0, 0, 1, 0, 0, dt, 
                     0, 0, 0, 1, 0, 0, 
                     0, 0, 0, 0, 1, 0, 
                     0, 0, 0, 0, 0, 1;

    externCmdBk   << (std::pow(dt,2)/(2*POIDS)), 0, 0,  //same here
                     0, (std::pow(dt,2)/(2*POIDS)), 0,
                     0, 0, (std::pow(dt,2)/(2*POIDS)),
                     dt, 0, 0,
                     0, dt, 0,
                     0, 0, dt;

    //DEBUG(physicModelFk*filteredPos);

    predictedXk      = physicModelFk*filteredPos + externCmdBk*cmdUk;
    predCovariancePk = physicModelFk*filteredCov*(physicModelFk.transpose()) + externImpactQk;

}

void Movement_kalman_filter::updatePhase(double _time){
    kalmanGainK = predCovariancePk*(sensorGainHk.transpose())*((sensorGainHk*predCovariancePk*sensorGainHk.transpose()+sensorCovarianceRk).inverse());
    filteredPos = predictedXk + kalmanGainK*(measurementsZk-sensorGainHk*predictedXk);
    filteredCov = predCovariancePk - kalmanGainK*sensorGainHk*predCovariancePk;
    //lastUpdate  = getCurrentTime();
}


void Movement_kalman_filter::fusionSensors(double _time){
    //Synchronize sensors data : if the last sensor of the other kind is too late (like video for example), the
    // idea is to extrapolate a new value as if the sensor gave a data at the good time. it is important to take
    //this information into consideration while setting the covariance matrix of the spoken sensor.
    Eigen::MatrixXd chosenOdom(6,1);
    Eigen::MatrixXd chosenVideo(6,1);
    Eigen::MatrixXd chosenOdomCov(6,6);
    Eigen::MatrixXd chosenVideoCov(6,6);

    if(samples[0].time() == _time){ //the time reference is set by a video sample
        chosenVideo << this->samples[0].linear_position().getX(),
                       this->samples[0].linear_position().getY(),
                       this->samples[0].angular_position().value(),
                       this->samples[0].linear_velocity()[0],
                       this->samples[0].linear_velocity()[1],
                       this->samples[0].angular_velocity().value();
    
        int i = 0;
        PositionSample extrapolatedOdom;
        while(samples[1].time(i) > _time){//Let's find the nearest odometry sample
            i++;
        }
        if(i > 0){//it means we have more recent odometry samples that we might use to gain precision
            double ratio = (samples[1].time(i-1)-_time)/(samples[1].time(i-1)-samples[1].time(i));
            extrapolatedOdom = PositionSample(_time,
                               (1-ratio)*samples[1].linear_position(i-1) + ratio*samples[1].linear_position(i),
                               (1-ratio)*samples[1].angular_position(i-1).value() + ratio*samples[1].angular_position(i).value());
        
            chosenOdom << extrapolatedOdom.linear_position.getX(),
                          extrapolatedOdom.linear_position.getY(),
                          extrapolatedOdom.angular_position.value(),
                          ((1-ratio)*samples[1].linear_velocity(i-1)[0] + ratio*samples[1].linear_velocity(i)[0]),
                          ((1-ratio)*samples[1].linear_velocity(i-1)[1] + ratio*samples[1].linear_velocity(i)[1]),
                          ((1-ratio)*samples[1].angular_velocity(i-1).value() + ratio*samples[1].angular_velocity(i).value());
        }
        else{     //we have to extrapolate a new value not given by the sensors
            int extra_dt = _time - samples[1].time();
            extrapolatedOdom = PositionSample(_time,
                               samples[1].linear_position() + extra_dt*samples[1].linear_velocity(),
                               samples[1].angular_position().value() + extra_dt*samples[1].angular_velocity().value());
        
            chosenOdom << extrapolatedOdom.linear_position.getX(),
                          extrapolatedOdom.linear_position.getY(),
                          extrapolatedOdom.angular_position.value(),
                          (samples[1].linear_velocity() + extra_dt*samples[1].linear_acceleration())[0],
                          (samples[1].linear_velocity() + extra_dt*samples[1].linear_acceleration())[1],
                          (samples[1].angular_velocity().value() + extra_dt*samples[1].angular_acceleration().value());
        }
        
    }
    else{//the time reference is set by the odometry sample
        chosenOdom << samples[1].linear_position().getX(),
                      samples[1].linear_position().getY(),
                      samples[1].angular_position().value(),
                      samples[1].linear_velocity()[0],
                      samples[1].linear_velocity()[1],
                      samples[1].angular_velocity().value();
        
        int i = 0;
        PositionSample extrapolatedVideo;
        while(samples[0].time(i) > _time){//Let's find the nearest odometry sample
            i++;
        }
        if(i > 0){//it means we have more recent odometry samples that we might use to gain precision
            double ratio = (samples[0].time(i-1)-_time)/(samples[0].time(i-1)-samples[0].time(i));
            extrapolatedVideo = PositionSample(_time,
                                (1-ratio)*samples[0].linear_position(i-1) + ratio*samples[0].linear_position(i),
                                (1-ratio)*samples[0].angular_position(i-1).value() + ratio*samples[0].angular_position(i).value());
        
            chosenVideo << extrapolatedVideo.linear_position.getX(),
                           extrapolatedVideo.linear_position.getY(),
                           extrapolatedVideo.angular_position.value(),
                           (1-ratio)*samples[0].linear_velocity(i-1)[0] + ratio*samples[0].linear_velocity(i)[0],
                           (1-ratio)*samples[0].linear_velocity(i-1)[1] + ratio*samples[0].linear_velocity(i)[1],
                           (1-ratio)*samples[0].angular_velocity(i-1).value() + ratio*samples[0].angular_velocity(i).value();
        }
        else{     //we have to extrapolate a new value not given by the sensors
            int extra_dt = _time - samples[0].time();
            extrapolatedVideo = PositionSample(_time,
                                samples[0].linear_position() + extra_dt*samples[0].linear_velocity(),
                                samples[0].angular_position().value() + extra_dt*samples[0].angular_velocity().value());
        
            chosenVideo << extrapolatedVideo.linear_position.getX(),
                           extrapolatedVideo.linear_position.getY(),
                           extrapolatedVideo.angular_position.value(),
                           (samples[0].linear_velocity() + extra_dt*samples[0].linear_acceleration())[0],
                           (samples[0].linear_velocity() + extra_dt*samples[0].linear_acceleration())[1],
                           (samples[0].angular_velocity().value() + extra_dt*samples[0].angular_acceleration().value());
        }
    }

    //TODO Calculate Covariance too = find a way to scale it on a timestamps
    chosenOdomCov  = Eigen::MatrixXd::Identity(6,6);
    chosenVideoCov = Eigen::MatrixXd::Identity(6,6);

    Eigen::MatrixXd gainFusion(6,6);
    gainFusion = chosenVideoCov * (chosenVideoCov + chosenOdomCov).transpose();
    measurementsZk = chosenVideo + gainFusion*(chosenOdom - chosenVideo);
    sensorCovarianceRk = chosenVideoCov - gainFusion * chosenOdomCov;

}

void Movement_kalman_filter::kalman_tick(double _time){
    filteredPos << this->samples[2].linear_position().getX(),
                   this->samples[2].linear_position().getY(),
                   this->samples[2].angular_position().value(),
                   this->samples[2].linear_velocity()[0],
                   this->samples[2].linear_velocity()[1],
                   this->samples[2].angular_position().value();

    cmdUk       << this->ordersSamples.linear_velocity()[0],
                   this->ordersSamples.linear_velocity()[1],
                   this->ordersSamples.angular_velocity();

    predictPhase(_time);
    fusionSensors(_time);
    updatePhase(_time);

    MovementSample tempo = this->samples[2];
    tempo.insert(PositionSample(_time, rhoban_geometry::Point(filteredPos(0,0), filteredPos(1,0)), ContinuousAngle(filteredPos(2,0))));
    this->set_sample(tempo, 2);
}

Movement_kalman_filter::~Movement_kalman_filter(){

    //delete samples;

}
    
}