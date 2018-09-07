#ifndef __ORDERSSAMPLE_H__
#define __ORDERSSAMPLE_H__

#include <math/vector2d.h>
#include <math/circular_vector.h>

namespace RhobanSSL {

struct SpeedTargetSample {

    double time;
    int16_t x_speed; // mm/s
    int16_t y_speed; // mm/s
    int16_t t_speed; // mrad/s

    SpeedTargetSample();
    SpeedTargetSample(
        double time,
        const int16_t & x_speed,
        const int16_t & y_speed,
        const int16_t & t_speed
    );
};

struct OrdersSample : public circular_vector<SpeedTargetSample> {

    OrdersSample(unsigned int);
    OrdersSample();

    double time( unsigned int i = 0 ) const;
    double dt( unsigned int i = 0 ) const;

    Vector2d linear_velocity( unsigned int i = 0 ) const;
    double angular_velocity( unsigned int i = 0 ) const;

    Vector2d linear_acceleration( unsigned int i = 0 ) const;
    double angular_acceleration( unsigned int i = 0 ) const;


    bool is_valid() const;
    void insert( const SpeedTargetSample & sample );

};

}//namespace

std::ostream& operator<<(
    std::ostream& stream, const RhobanSSL::SpeedTargetSample & speed
);

std::ostream& operator<<(
    std::ostream& stream, const RhobanSSL::OrdersSample & order
);


#endif

