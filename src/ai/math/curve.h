#ifndef __curve__H__
#define __curve__H__

#include <debug.h>
#include <Eigen/Dense>
#include <vector>

struct ContinuousVelocityConsign {
    double distance;
    double max_velocity;
    double max_acceleration; 

    ContinuousVelocityConsign(
        double distance, 
        double max_velocity,
        double max_acceleration 
    );

    double operator()(double t);
    double time_of_deplacement();
    double time_of_acceleration();
};


struct DifferentiableVelocityConsign {
    double distance;
    double max_velocity;
    double max_acceleration; 

    DifferentiableVelocityConsign(
        double distance, 
        double max_velocity,
        double max_acceleration 
    );

    double operator()(double t);
    double time_of_deplacement();
    double time_of_acceleration();
};

class Curve2d;

class Curve2d {
    public:
        std::function<Eigen::Vector2d (double u)> curve;
        
        double step_time;
        double curve_length;
        double time_max;

        void init();
    public:
        struct Length { 
            const Curve2d & _this;
            double v;
            double length;
            Eigen::Vector2d old;

            Length( const Curve2d & _this ):
                _this(_this),
                v(0),
                length(0),
                old( _this.curve( 0.0 ) )
            { }

            double operator() ( double u ){
                if( u <= 0 ) return 0.0;
                if( u > 1.0 ) return _this.curve_length;
                assert( v <= u ); 
                for( ; v <= u; v+=_this.step_time ){
                    Eigen::Vector2d current = _this.curve( v );
                    length += ( current - old ).norm();
                    old = current;
                }
                return length + ( _this.curve(u) - old ).norm();
            }
        };

        struct Inverse_of_length { 
            const Curve2d & _this;
            double u;
            double length;
            Eigen::Vector2d old;

            Inverse_of_length( const Curve2d & _this ):
                _this(_this),
                u(0),
                length(0),
                old( _this.curve( 0.0 ) )
            { }

            double operator() ( double l ){
                if( l <= 0 ) return 0.0;
                if( l >= _this.curve_length ) return 1.0;
                for( ; length < l; u+=_this.step_time ){
                    Eigen::Vector2d current = _this.curve( u );
                    length += ( current - old ).norm();
                    old = current;
                }
                return u;
            }
        };



    public:
        Curve2d(
            const std::function<Eigen::Vector2d (double u)> & curve,
            double step_time
        );
        Curve2d( const Curve2d& curve );

        Eigen::Vector2d operator()(double u) const;

        double arc_length( double u ) const;
        Length length_iterator() const {
            return Length(*this);
        }

        double inverse_of_arc_length( double l ) const;
        Inverse_of_length Inverse_of_length_iterator() const {
            return Inverse_of_length( *this );
        }

        double size() const;
        
};

class RenormalizedCurve;

class RenormalizedCurve : public Curve2d {
    public:
        std::function<double (double t)> velocity_consign;
        double time_max;

        double length_tolerance;

        void init();

    public:
        struct PositionConsign { 
            const RenormalizedCurve & _this;
            double pos;
            double u;

            PositionConsign( const RenormalizedCurve & _this ):
                _this(_this), pos(0.0), u(0.0)
            { }

            double operator() ( double t ){
                assert( u<=t );
                for(; u<t; u+=_this.step_time){
                    pos += _this.step_time * _this.velocity_consign(u);
                } 
                return pos;
            }
        };

        struct TimeCurve { 
            const RenormalizedCurve & _this;
            double res;
            double t;

            TimeCurve( const RenormalizedCurve & _this ):
                _this(_this), res(0.0), t(0.0)
            { }

            double operator() ( double length ){
                assert( 0 <= length );
                assert( length <= _this.curve_length );
                
                for( ; res < length-_this.length_tolerance; t+=_this.step_time ){
                    assert( _this.velocity_consign(t) >= 0.0 );
                    res += _this.step_time * _this.velocity_consign(t);
                } 
                return t;
            }
        };


        RenormalizedCurve(
            const std::function<Eigen::Vector2d (double u)> & curve,
            const std::function<double (double t)> & velocity_consign,
            double step_time, double length_tolerance
        );
        RenormalizedCurve(
            const Curve2d & curve,
            const std::function<double (double t)> & velocity_consign,
            double length_tolerance
        );

        void set_step_time( double dt );

        double max_time() const;
        Eigen::Vector2d original_curve( double u ) const;
        double time( double length ) const;
        TimeCurve time_iterator() const;
        double position_consign( double t ) const;


        struct CurveIterator {
            const RenormalizedCurve & _this;
            Inverse_of_length inverse_length_iterator;
            PositionConsign position_consign_iterator;
    
            CurveIterator( const RenormalizedCurve & _this ):
                _this(_this),
                inverse_length_iterator(_this.Inverse_of_length_iterator()),
                position_consign_iterator( PositionConsign(_this) )
            { }

            
            Eigen::Vector2d operator()(double t) {
                return _this.original_curve( inverse_length_iterator( position_consign_iterator(t) ) );
            }
        };

        CurveIterator curve_iterator() const {
            return CurveIterator(*this);
        }
        PositionConsign position_consign_iterator() const {
            return PositionConsign(*this);
        }
        Eigen::Vector2d operator()(double t) const;

        double error_position_consign() const;
        double get_step_time() const;
};

#endif
