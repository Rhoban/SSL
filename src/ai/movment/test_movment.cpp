#include "debug.h"
#include "movment.h"

#include <iostream>
#include <Eigen/Dense>
#include <cmath>

bool eq( double u, double v, double err ){
    return std::abs(u-v) <= err;
}

double error_renormalisation(
    double u, const RenormalizedCurve & curve, 
    std::function<double (double)> & error_curve,
    double dt
){
    return (
        error_curve(
            curve.inverse_of_arc_length( curve.position_consign(u) )
        ) +
        error_curve(
            curve.inverse_of_arc_length( curve.position_consign(u+dt) )
        )
    )/dt;
};

void test_curves(){
    {
        double dt = 0.01;
        double dt_micro = dt/100;
        double erreur = dt/10;

        double length = 2.0;
        RenormalizedCurve curve(
            [&](double u){ return Eigen::Vector2d(length*u,0.0); }, 
            [](double t){ return 1.0; },
            dt_micro        
        );
        for( double u=0.0; u<=1.0; u = u+ dt ){
            assert( curve.original_curve(u) == Eigen::Vector2d(2*u,0));
        }    

        assert( 
            eq( length, curve.size(),  erreur)
        );

        assert( eq(length, curve.max_time(), erreur) );
        assert( eq(length, curve.time(length-erreur), erreur) );

        for( double d=0.0; d<=length; d = d + dt ){
            assert( eq(d, curve.time(d), erreur) );
        }    

        for( double t=0.0; t<2*length; t = t + dt ){
            assert( eq(t, curve.position_consign(t), erreur) );
        }    

        for( double u=0.0; u<1.0; u = u + dt ){
            assert( eq(length*u, curve.arc_length(u), erreur) );
        }
        
        for( double l=0.0; l<=length; l = l + dt ){
            assert( eq(l/length, curve.inverse_of_arc_length(l), erreur) );
        }    

        std::function<double(double)> error_curve = [&](double u){
            return 2*dt_micro;
        };
        for( double t=0.0; t<curve.max_time()-dt; t = t + dt ){
            assert(
                eq(
                    1.0,
                    ( curve(t+dt) - curve(t) ).norm()/dt, 
                    error_renormalisation(t, curve, error_curve, dt) + dt_micro/10.0
                )
            );
        }
    }

    {
        double dt = 0.01;
        double dt_micro = dt/100;

        RenormalizedCurve curve(
            [](double u){ return Eigen::Vector2d(u*u,2*u); }, 
            [](double t){ return t+.1; },
            dt_micro        
        );
        std::function<double(double)> error_curve = [&](double u){
            return 2*dt_micro*std::sqrt( std::pow( u+dt_micro/2.0, 2 ) + 1 );
        };
        for( double t=0.0; t<curve.max_time()-dt; t = t + dt ){
            assert(
                eq( 
                    t+.1, 
                    ( curve(t+dt) - curve(t) ).norm()/dt, 
                    error_renormalisation(t, curve, error_curve, dt) + dt_micro/10.0
                )
            );
        }
    }
}

void test_velocityconsign(){
    {
        double erreur = 0.0000001;
        double distance = 4.0;
        double max_acceleration = 3.0;
        double max_velocity = 2.0;
        VelocityConsign consign(distance, max_acceleration, max_velocity);
        assert( consign(-0.1) == 0.0 );
        assert( consign(0) == 0.0 );
        assert(
            eq(
                consign(consign.time_of_acceleration()/2.0),
                max_velocity/2.0, erreur
            )
        );
        assert(
            eq(
                consign(consign.time_of_acceleration()),
                max_velocity, erreur
            )
        );
        assert(
            eq( 
                consign( consign.time_of_deplacement() 
                - consign.time_of_acceleration()),
                max_velocity, erreur
            )
        );
        assert( 
            eq(
                consign( 
                    consign.time_of_deplacement() - 
                    consign.time_of_acceleration()/2.0
                ),
                max_velocity/2.0, erreur
            )
        );
        assert( consign(consign.time_of_deplacement()) == 0.0 );
        assert( consign(consign.time_of_deplacement()+.01) == 0.0 );
        double len = 0.0;
        double step = .00001;
        for( double t = 0.0; t<consign.time_of_deplacement(); t+= step ){
            len += consign(t) * step;
        }
        assert( eq( distance, len, .0001 ) );
    }
}

int main(){
//    test_curves();
    test_velocityconsign();
}
