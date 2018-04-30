#include <gtest/gtest.h>

#include <debug.h>
#include "matrix2d.h"

TEST(test_matrix2d, constructor){
    {
        Matrix2d m;
        EXPECT_EQ( m(0,0), 0.0 );
        EXPECT_EQ( m(0,1), 0.0 );
        EXPECT_EQ( m(1,0), 0.0 );
        EXPECT_EQ( m(1,1), 0.0 );
    }
    {
        Matrix2d m(1.0, 2.0, 3.0, 4.0);
        EXPECT_EQ( m(0,0), 1.0 );
        EXPECT_EQ( m(0,1), 2.0 );
        EXPECT_EQ( m(1,0), 3.0 );
        EXPECT_EQ( m(1,1), 4.0 );
    }
}

TEST(test_matrix2d, determinant){
    {
        Matrix2d m(1,2,3,4);
        EXPECT_EQ( m.det(), 1.0*4.0 - 2.0*3.0 );
    }
}

TEST(test_matrix2d, inverse){
    {
        // m = 1 2
        //     3 4
        Matrix2d m(1.0,2.0,3.0,4.0);
        // det = 1*4-2*3 = -2
        // m =  4/-2 -2/-2
        //     -3/-2  1/-2
        Matrix2d i = m.inverse();

        EXPECT_TRUE( std::fabs( i(0,0) - (-2.0) )<0.00001 );
        EXPECT_TRUE( std::fabs( i(0,1) - (1.0) )<0.00001 );
        EXPECT_TRUE( std::fabs( i(1,0) - (3.0/2.0) )<0.00001 );
        EXPECT_TRUE( std::fabs( i(1,1) - (-1.0/2.0) )<0.00001 );

        Matrix2d id1 = m * i;
        EXPECT_TRUE( std::fabs( id1(0,0) - 1.0 )<0.00001 );
        EXPECT_TRUE( std::fabs( id1(0,1) - 0.0 )<0.00001 );
        EXPECT_TRUE( std::fabs( id1(1,0) - 0.0 )<0.00001 );
        EXPECT_TRUE( std::fabs( id1(1,1) - 1.0 )<0.00001 );
        
        Matrix2d id2 = i * m;
        EXPECT_TRUE( std::fabs( id2(0,0) - 1.0 )<0.00001 );
        EXPECT_TRUE( std::fabs( id2(0,1) - 0.0 )<0.00001 );
        EXPECT_TRUE( std::fabs( id2(1,0) - 0.0 )<0.00001 );
        EXPECT_TRUE( std::fabs( id2(1,1) - 1.0 )<0.00001 );
    }
}

TEST(test_matrix2d, vector2d){
    {
        // m = 1 2
        //     3 4
        Matrix2d m(1,2,3,4);
        
        Vector2d v1(4,5);
        Vector2d v2 = m*v1;

        EXPECT_TRUE( std::fabs( v2[0] - (4.0*1.0 + 5.0*2.0) )<0.00001 );
        EXPECT_TRUE( std::fabs( v2[1] - (4.0*3.0 + 5.0*4.0) )<0.00001 );
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
