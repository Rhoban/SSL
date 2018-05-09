#include "collision.h"

namespace RhobanSSL {

std::pair<bool, double> collision_time(
    double radius_1, const Movement & movement_1,
    double radius_2, const Movement & movement_2,
    double radius_error, double time
){
    std::pair<bool, double> result(false, 0.0); 

    rhoban_geometry::Point A = movement_1.linear_position( time );
    Vector2d V_A = movement_1.linear_velocity( time );
    rhoban_geometry::Point B  = movement_2.linear_position( time );
    Vector2d V_B = movement_2.linear_velocity( time );

    Vector2d V = V_B - V_A;
    Vector2d T = B - A;
    double full_radius = radius_error + radius_1 + radius_2;

    double a = T.norm_square();
    double b = 2 * scalar_product( V, T );
    double c = T.norm_square() - full_radius * full_radius;
    
    double discriminant = b*b - 4*a*c;
    if( discriminant >= 0 ){
        double square_discriminant = std::sqrt( discriminant );
        double t_p = (- b + square_discriminant)/2.0;
        double t_m = (- b - square_discriminant)/2.0;
        if( time < t_m ){
            result.first = true;
            result.second = t_m;
        }else if( time  <= t_p ){
            result.first = true;
            result.second = 0.0;
        }
    }
    return result;

}

}
