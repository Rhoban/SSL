#ifndef __ROBOT_BEHAVIOR__SEARCHSHOOTAREA__H__
#define __ROBOT_BEHAVIOR__SEARCHSHOOTAREA__H__

#include "robot_behavior.h"
#include "factory.h"

namespace RhobanSSL
{
namespace Robot_behavior {

class SearchShootArea : public RobotBehavior  {
    private:
      rhoban_geometry::Point p1;
      rhoban_geometry::Point p2;
      int random;

      ConsignFollower* follower;
      RhobanSSLAnnotation::Annotations annotations;
    public:
        SearchShootArea(Ai::AiData& ai_data);

        virtual void update(
            double time,
            const Ai::Robot & robot,
            const Ai::Ball & ball
        );

        void declare_area( rhoban_geometry::Point p1 ,
                                            rhoban_geometry::Point p2 );


	virtual Control control() const;
  virtual RhobanSSLAnnotation::Annotations get_annotations() const ;
	virtual ~SearchShootArea();
};

};
}; //Namespace Rhoban

#endif
