#include "collision.h"
#include <debug.h>
#include "constants.h"

namespace RhobanSSL {

std::pair<bool, double> collision_time(
    double radius_1, const Movement & movement_1,
    double radius_2, const Movement & movement_2,
    double radius_error, double time
){
    return collision_time(
        radius_1, movement_1.linear_position( time ), movement_1.linear_velocity( time ),
        radius_2, movement_2.linear_position( time ), movement_2.linear_velocity( time ),
        radius_error
    );
}

std::pair<bool, double> collision_time(
    double radius_A, const rhoban_geometry::Point & A, const Vector2d & V_A,
    double radius_B, const rhoban_geometry::Point & B, const Vector2d & V_B,
    double radius_error
){
    std::pair<bool, double> result(false, 0.0); 

    Vector2d V = V_B - V_A;
    Vector2d T = B - A;
    double full_radius = radius_error + radius_A + radius_B;

    double a = V.norm_square();
    double b = 2 * scalar_product( V, T );
    double c = T.norm_square() - full_radius * full_radius;

    if( a < EPSILON_VELOCITY ){
        if(c < EPSILON_DISTANCE){
            return std::pair<bool, double>(true, 0.0);
        }
    }else{
        double discriminant = b*b - 4*a*c;
        if( discriminant >= 0 ){
            double square_discriminant = std::sqrt( discriminant );
            double t_p = (- b + square_discriminant)/(2.0*a);
            double t_m = (- b - square_discriminant)/(2.0*a);
            if( 0 < t_m ){
                result.first = true;
                result.second = t_m;
            }else if( 0  <= t_p ){
                result.first = true;
                result.second = 0.0;
            }
        }
    }
    return result;

}

}
