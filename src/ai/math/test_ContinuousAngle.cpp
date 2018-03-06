#include "ContinuousAngle.h"
#include <assert.h>
#include <math.h>
#include <debug.h>
#include <sstream>

using namespace rhoban_utils;
    
void test_constructors(){
    {
        ContinuousAngle c;
        assert( c.value() == 0.0 );
    }

    {
        ContinuousAngle c(2.1);
        assert( c.value() == 2.1 );
    }

    {
        ContinuousAngle c(2.1);
        ContinuousAngle d(c);
        assert( c.value() == 2.1 );
        assert( d.value() == 2.1 );
    }


    {
        ContinuousAngle c(4.5);
        assert( c.value() == 4.5 );
    }

    {
        ContinuousAngle c(-4.5);
        assert( c.value() == -4.5 );
    }

    {
        ContinuousAngle c(-4.5);
        assert( c.value() == -4.5 );
    }
/*
    {
        ContinuousAngle c( Angle(321) );
        assert( c.angle() == Angle(321) );
    }
*/
    {
        ContinuousAngle c = -4.5;
        assert( c.value() == -4.5 );
    }

    {
        ContinuousAngle c = 4.5;
        assert( c.value() == 4.5 );
    }
/*
    {
        ContinuousAngle c = Angle(321);
        assert( c.angle() == Angle(321) );
    }
*/
}

void test_angle(){
    {
        ContinuousAngle c(2*M_PI);
        assert( std::abs( c.angle().getSignedValue() - 0 ) < 0.00001 );
    }
    {
        ContinuousAngle c(4*M_PI);
        assert( std::abs( c.angle().getSignedValue() - 0 ) < 0.00001 );
    }
    {
        ContinuousAngle c(-4*M_PI);
        assert( std::abs( c.angle().getSignedValue() - 0 ) < 0.00001 );
    }
}

void test_abs(){
    {
        ContinuousAngle c(1.4);
        ContinuousAngle d = c.abs();
        assert( std::abs( d.value() - 1.4 ) < 0.0001 );
    }
    {
        ContinuousAngle c(-1.4);
        ContinuousAngle d = c.abs();
        assert( std::abs( d.value() - 1.4 ) < 0.0001 );
    }
}

void test_operator_plus(){
    {
        ContinuousAngle c(1.4);
        c+=1;
        assert( std::abs( c.value() - 2.4 ) < 0.0001 );
    }
    {
        ContinuousAngle c(1.4);
        ContinuousAngle d = c + 1;
        assert( std::abs( c.value() - 1.4 ) < 0.0001 );
        assert( std::abs( d.value() - 2.4 ) < 0.0001 );
    }
    {
        ContinuousAngle c(1.4);
        ContinuousAngle d = +c;
        assert( std::abs( c.value() - 1.4 ) < 0.0001 );
        assert( std::abs( d.value() - 1.4 ) < 0.0001 );
    }
}

void test_operator_minus(){
    {
        ContinuousAngle c(1.4);
        c-=1;
        assert( std::abs( c.value() - 0.4 ) < 0.0001 );
    }
    {
        ContinuousAngle c(1.4);
        ContinuousAngle d = c - 1;
        assert( std::abs( c.value() - 1.4 ) < 0.0001 );
        assert( std::abs( d.value() - 0.4 ) < 0.0001 );
    }
    {
        ContinuousAngle c(1.4);
        ContinuousAngle d = -c;
        assert( std::abs( c.value() - 1.4 ) < 0.0001 );
        assert( std::abs( d.value() + 1.4 ) < 0.0001 );
    }
}

void test_operator_prod(){
    {
        ContinuousAngle c(1.4);
        c*=2;
        assert( std::abs( c.value() - 2.8 ) < 0.0001 );
    }
    {
        ContinuousAngle c(1.4);
        ContinuousAngle d = c * 2;
        assert( std::abs( c.value() - 1.4 ) < 0.0001 );
        assert( std::abs( d.value() - 2.8 ) < 0.0001 );
    }
}

void test_operator_div(){
    {
        ContinuousAngle c(2.8);
        c/=2;
        assert( std::abs( c.value() - 1.4 ) < 0.0001 );
    }
    {
        ContinuousAngle c(2.8);
        ContinuousAngle d = c / 2;
        assert( std::abs( c.value() - 2.8 ) < 0.0001 );
        assert( std::abs( d.value() - 1.4 ) < 0.0001 );
    }
}

void test_operator_equal(){
    {
        ContinuousAngle c(2.8);
        ContinuousAngle d(2.8);
        ContinuousAngle e(4.8);

        assert( c == c );
        assert( not(c != c) );
        assert( not(c < c) );
        assert( c <= c );
        assert( not(c > c) );
        assert( c >= c );
    }
}


void test_turn(){
    {
        ContinuousAngle c(2.8);
        assert( std::abs( c.turn() - 0.445633840657 ) < 0.0001 );
    }
}

void test_nb_turn(){
    {
        ContinuousAngle c(2.8);
        assert( c.nb_turn() == 0 );
        c += (2*M_PI);
        assert( c.nb_turn() == 1 );
        c += (2*M_PI);
        assert( c.nb_turn() == 2 );
    }
    {
        ContinuousAngle c(2.8);
        assert( c.nb_turn() == 0 );
        c -= (2*M_PI);
        assert( c.nb_turn() == 0 );
        c -= (2*M_PI);
        assert( c.nb_turn() == -1 );
        c -= (2*M_PI);
        assert( c.nb_turn() == -2 );
    }
}

void test_stream(){
    {
        ContinuousAngle c(2.8);

        std::ostringstream s1;
        
        s1 << c;
        assert( "2.8" == s1.str() ); 
        s1.str("");
        s1.clear();
        
        c += (2*M_PI);
        s1 << c;
        assert( "1*2pi+2.8" == s1.str() ); 
        s1.str("");
        s1.clear();
        
        c += (2*M_PI);
        s1 << c;
        assert( "2*2pi+2.8" == s1.str() ); 
        s1.str("");
        s1.clear();
    } 
    {
        ContinuousAngle c(2.8);

        std::ostringstream s1;
        
        s1 << c;
        assert( "2.8" == s1.str() ); 
        s1.str("");
        s1.clear();
        
        c -= (2*M_PI);
        s1 << c;
        assert( "-3.48319" == s1.str() ); 
        s1.str("");
        s1.clear();
        
        c -= (2*M_PI);
        s1 << c;

        assert( "-1*2pi-3.48319" == s1.str() ); 
        s1.str("");
        s1.clear();

        c -= (2*M_PI);
        s1 << c;
        assert( "-2*2pi-3.48319" == s1.str() ); 
        s1.str("");
        s1.clear();
    } 
}

void test_set_to_nearest(){
    {
        double amplitude = 4*2*M_PI;
        ContinuousAngle c(-amplitude);
        ContinuousAngle d(c);
        double step = .1;
        for( int i=0; i<2*amplitude/step; i++ ){
            d = c;

            double angle = std::fmod(
                c.value() + step, 2*M_PI
            );
            c.set_to_nearest( angle );
            
            assert(
                std::abs( c.value() - d.value() ) < 2*step
            );
        } 
    }
    {
        double amplitude = 4*2*M_PI;
        ContinuousAngle c(amplitude);
        ContinuousAngle d(c);
        double step = .1;
        for( int i=0; i<2*amplitude/step; i++ ){
            d = c;
            double angle = std::fmod(
                c.value() - step, 2*M_PI
            );
            c.set_to_nearest( angle );
            assert(
                std::abs( c.value() - d.value() ) < 2*step
            );
            
        }
    } 
    {
        double amplitude = 4*2*M_PI;
        ContinuousAngle c(-amplitude);
        ContinuousAngle d(c);
        double step = .1;
        for( int i=0; i<2*amplitude/step; i++ ){
            d = c;
            
            Angle angle( rad2deg( c.value() + step ) );
            c.set_to_nearest( angle );
           
            assert(
                std::abs( 
                    Angle( rad2deg(c.value()) ).getSignedValue()
                    -
                    angle.getSignedValue()
                ) < 0.000001
            );
            
            assert(
                std::abs( c.value() - d.value() ) < 2*step
            );
        }
    }
    {
        double amplitude = 4*2*M_PI;
        ContinuousAngle c(amplitude);
        ContinuousAngle d(c);
        double step = .1;
        for( int i=0; i<2*amplitude/step; i++ ){
            d = c;
            Angle angle ( rad2deg( c.value() - step ) );
            c.set_to_nearest( angle );

            assert(
                std::abs( 
                    Angle( rad2deg(c.value()) ).getSignedValue()
                    -
                    angle.getSignedValue()
                ) < 0.000001
            );

            assert(
                std::abs( c.value() - d.value() ) < 2*step
            );
            
        }
    } 
/*
    {
        double amplitude = 4*2*M_PI;
        ContinuousAngle c(-amplitude);
        ContinuousAngle d(c);
        double step = .1;
        for( int i=0; i<2*amplitude/step; i++ ){
            d = c;

            double angle = std::fmod(
                c.value() + step, 2*M_PI
            );
            c = Angle( rad2deg(angle) );
            
            assert(
                std::abs( c.value() - d.value() ) < 2*step
            );
        } 
    }
    {
        double amplitude = 4*2*M_PI;
        ContinuousAngle c(amplitude);
        ContinuousAngle d(c);
        double step = .1;
        for( int i=0; i<2*amplitude/step; i++ ){
            d = c;
            double angle = std::fmod(
                c.value() - step, 2*M_PI
            );
            c = Angle( rad2deg(angle) );
            assert(
                std::abs( c.value() - d.value() ) < 2*step
            );
            
        }
    } 
*/
}

int main(){

    test_constructors();
    test_angle();
    test_operator_plus();
    test_operator_minus();
    test_operator_prod();
    test_operator_div();
    test_operator_equal();
    
    test_abs();

    test_turn();
    test_nb_turn();

    test_set_to_nearest();

    test_stream();
    return 0;
}
