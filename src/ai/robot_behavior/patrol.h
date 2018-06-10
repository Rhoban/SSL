#ifndef __ROBOT_BEHAVIOR__PATROL__H__
#define __ROBOT_BEHAVIOR__PATROL__H__

#include "robot_behavior.h"
#include "factory.h"

namespace RhobanSSL
{
namespace Robot_behavior {

class Patrol : public RobotBehavior  {
    private:
	ConsignFollower* follower;
    int zone;
    std::vector< rhoban_geometry::Point > traject;

    public:
    Patrol(Ai::AiData& ai_data);

    virtual void update(
        double time,
        const Ai::Robot & robot,
        const Ai::Ball & ball
    );

    static Patrol* two_way_trip( Ai::AiData& ai_data );
    static Patrol* tour_of_the_field( Ai::AiData& ai_data );

    void set_traject( const std::vector< rhoban_geometry::Point > & traject );

	virtual Control control() const;
    
    RhobanSSLAnnotation::Annotations get_annotations() const;

	virtual ~Patrol();

};

};
}; //Namespace Rhoban

#endif
