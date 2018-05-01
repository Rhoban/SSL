#ifndef __CONTINUOUS_ANGLE__H__
#define __CONTINUOUS_ANGLE__H__

#include <rhoban_utils/angle.h>

class ContinuousAngle {
private:
    double angle_value;
public:
    ContinuousAngle();
    ContinuousAngle(double angle);
//    ContinuousAngle(const Angle& angle);  // Too dangerous, User set_to_nearest
    ContinuousAngle(const ContinuousAngle& angle);

    ContinuousAngle& operator=( double angle );
    ContinuousAngle& operator=( const ContinuousAngle & angle );
//    ContinuousAngle& operator=( const Angle & angle );

    rhoban_utils::Angle angle() const;
    double value() const;
    ContinuousAngle abs() const;

    ContinuousAngle operator-() const;
    ContinuousAngle operator+() const;

    ContinuousAngle operator-( double angle ) const;
    ContinuousAngle operator-( const ContinuousAngle & angle ) const;
    
    ContinuousAngle& operator-=( double angle );
    ContinuousAngle& operator-=( const ContinuousAngle & angle );

    ContinuousAngle operator+( double angle ) const;
    ContinuousAngle operator+( const ContinuousAngle & angle ) const;
    
    ContinuousAngle& operator+=( double angle );
    ContinuousAngle& operator+=( const ContinuousAngle & angle );
    
    ContinuousAngle operator/( double scalar ) const;
    ContinuousAngle& operator/=( double scalar );
    
    ContinuousAngle operator*( double scalar ) const;
    ContinuousAngle& operator*=( double scalar );

    double turn() const;
    int nb_turn() const;

    bool operator==( const ContinuousAngle& angle ) const;
    bool operator!=( const ContinuousAngle& angle ) const;
    bool operator<( const ContinuousAngle& angle ) const;
    bool operator<=( const ContinuousAngle& angle ) const;
    bool operator>( const ContinuousAngle& angle ) const;
    bool operator>=( const ContinuousAngle& angle ) const;

    void set_to_nearest( double angle );
    void set_to_nearest( const ContinuousAngle & angle );
    void set_to_nearest( const rhoban_utils::Angle & angle );
};

std::ostream& operator<<(std::ostream& out, const ContinuousAngle& a);

#endif
