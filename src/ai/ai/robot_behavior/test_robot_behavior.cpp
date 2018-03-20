#include <gtest/gtest.h>

#include "robot_behavior.h"

TEST(test_robot_behavior, constructors){
    {
        RhobanSSL::Control p;
        EXPECT_TRUE( ! p.kick );
        EXPECT_TRUE( p.active );
        EXPECT_TRUE( ! p.ignore );
//        EXPECT_TRUE( p == PidControl() );
    }

    {
        bool cases[8][3] = {
            {false, false, false},
            {false, false, true},
            {false, true, false},
            {false, true, true},
            {true, false, false},
            {true, false, true},
            {true, true, false},
            {true, true, true}
        };
        for( int i = 0; i<8; i++ ){
            RhobanSSL::Control p(
                cases[i][0], cases[i][1], cases[i][2]
            );
            EXPECT_TRUE( p.kick == cases[i][0] );
            EXPECT_TRUE( p.active == cases[i][1] );
            EXPECT_TRUE( p.ignore == cases[i][2] );
        }
    }
}

TEST(test_robot_behavior, make_desactivated){
    {
        RhobanSSL::Control c = RhobanSSL::Control::make_desactivated(); 
        EXPECT_TRUE( c.kick == false );
        EXPECT_TRUE( c.active == false );
        EXPECT_TRUE( c.ignore == false );
    }
}

TEST(test_robot_behavior, make_ignored){
    {
        RhobanSSL::Control c = RhobanSSL::Control::make_ignored(); 
        EXPECT_TRUE( c.kick == false );
        EXPECT_TRUE( c.active == false );
        EXPECT_TRUE( c.ignore == true );
    }
}

TEST(test_robot_behavior, make_null){
    {
        RhobanSSL::Control c = RhobanSSL::Control::make_null(); 
        EXPECT_TRUE( c.kick == false );
        EXPECT_TRUE( c.active == true );
        EXPECT_TRUE( c.ignore == false );
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
