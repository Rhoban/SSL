#include <tools/debug.h>
#include "curve.h"

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
            dt_micro, dt_micro/100
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
        assert( curve(curve.max_time()+dt) == curve( curve.max_time() ) );
        assert( curve(curve.max_time()+3*dt) == curve( curve.max_time() ) );
    }

    {
        double dt = 0.01;
        double dt_micro = dt/100;

        RenormalizedCurve curve(
            [](double u){ return Eigen::Vector2d(u*u,2*u); },
            [](double t){ return t+.1; },
            dt_micro, dt_micro/100
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
        assert( curve(curve.max_time()+dt) == curve( curve.max_time() ) );
        assert( curve(curve.max_time()+3*dt) == curve( curve.max_time() ) );
    }


}

void test_velocityconsign(){
    {
        double erreur = 0.0000001;
        double distance = 4.0;
        double max_acceleration = 3.0;
        double max_velocity = 2.0;
        ContinuousVelocityConsign consign(distance, max_velocity, max_acceleration);
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

void test_empty_curves(){
    {
        double dt = 0.01;
        double dt_micro = dt/100;

        Eigen::Vector2d position(3.0,7.0);
        RenormalizedCurve curve(
            [&](double u){ return position; },
            [](double t){ return 1.0; },
            dt_micro, dt_micro/100
        );

        assert( 0.0 == curve.size() );
        assert( 0.0 == curve.max_time() );
        assert( position == curve(0.0) );
        assert( position == curve(0.4) );
    }
}

struct Rotation {
    double orientation;

    double operator()(double u) const {
        return  90.0*u + orientation;
    };
};

struct Translation {
    Eigen::Vector2d position;

    Eigen::Vector2d operator()(double u) const {
        return  position + Eigen::Vector2d(u, u*u);
    };
};


struct fct_wrapper {
    std::function<double (double u)> rotation;

    fct_wrapper(
        const std::function<double (double u)> & rotation
    ):rotation(rotation){ };

    Eigen::Vector2d operator()(double t){
        return Eigen::Vector2d( rotation(t), 0.0 );
    };
};

void test_use_cases(){
    {
        Rotation rot;
        rot.orientation = 90.0;
        fct_wrapper fct(rot);
        Curve2d curve(fct, 0.0001);
        assert( curve.size() == 90 );
    }

    {
        double dt = 0.01;
        double dt_micro = dt/100;

        RenormalizedCurve curve(
            [](double u){ return Eigen::Vector2d(90.0*u,0.0); },
            [](double t){ return 10.0; },
            dt_micro, dt_micro/100
        );
        assert( eq( curve.max_time(), 9, 0.01 ) );
    }

    {
        double dt = 0.01;
        double dt_micro = dt/100;

        Curve2d crv(
            [](double u){ return Eigen::Vector2d(90.0*u,0.0); },
            dt_micro
        );
        assert( crv.size() == 90 );
        double max_acceleration = 300.0;
        double max_velocity = 90.0;
        ContinuousVelocityConsign consign(crv.size(), max_velocity, max_acceleration);

        RenormalizedCurve curve(
            crv,
            consign,
            dt_micro, dt_micro/100
        );
    }
    {
        double dt = 0.001;
        double dt_micro = 0.0001;

        Translation trans;
        Curve2d crv( trans, dt_micro );
        double max_acceleration = 20.0;
        double max_velocity = 1.0;
        ContinuousVelocityConsign consign(crv.size(), max_velocity, max_acceleration);

        RenormalizedCurve curve(
            crv,
            consign,
            dt_micro, dt_micro/100
        );
        for( double t = 0; t< curve.max_time()-dt; t+= dt ){
            Eigen::Vector2d xt = curve(t);
            Eigen::Vector2d xdt = curve(t+dt);
            assert( xdt[0] >= xt[0] );
            assert( xdt[1] >= xt[1] );
        }
    }
}

int main(){
    test_use_cases();
    test_empty_curves();
    test_curves();
    test_velocityconsign();
}
