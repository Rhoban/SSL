#include <gtest/gtest.h>

#include <debug.h>
#include "vector2d.h"

TEST(test_vector2d, vectorial_product){
    {
        Vector2d v1(0.0, 0.0);
        Vector2d v2(0.0, 0.0);
        EXPECT_EQ( vectorial_product(v1, v2), 0.0 );
    }
    {
        Vector2d v1(1.0, 2.0);
        Vector2d v2(0.0, 0.0);
        EXPECT_EQ( vectorial_product(v1, v2), 0.0 );
    }
    {
        Vector2d v1(0.0, 0.0);
        Vector2d v2(3.0, 4.0);
        EXPECT_EQ( vectorial_product(v1, v2), 0.0 );
    }
    {
        Vector2d v1(2.0, 3.0);
        Vector2d v2(4.0, 5.0);
        EXPECT_EQ(
            vectorial_product(v1, v2),
            2.0*5.0-4.0*3.0 
        );
    }
}

TEST(test_vector2d, scalar_product){
    {
        Vector2d v1(0.0, 0.0);
        Vector2d v2(0.0, 0.0);
        EXPECT_EQ( scalar_product(v1, v2), 0.0 );
    }
    {
        Vector2d v1(1.0, 2.0);
        Vector2d v2(0.0, 0.0);
        EXPECT_EQ( scalar_product(v1, v2), 0.0 );
    }
    {
        Vector2d v1(0.0, 0.0);
        Vector2d v2(3.0, 4.0);
        EXPECT_EQ( scalar_product(v1, v2), 0.0 );
    }
    {
        Vector2d v1(2.0, 3.0);
        Vector2d v2(4.0, 5.0);
        EXPECT_EQ(
            scalar_product(v1, v2),
            2.0*4.0+3.0*5.0 
        );
    }
}


TEST(test_vector2d, norm){
    {
        Vector2d v(0.0, 0.0);
        EXPECT_EQ( norm(v), 0.0 );
    }
    {
        Vector2d v(1.0, 0.0);
        EXPECT_EQ( norm(v), 1.0 );
    }
    {
        Vector2d v(0.0, 1.0);
        EXPECT_EQ( norm(v), 1.0 );
    }
    {
        Vector2d v(3.0, 4.0);
        EXPECT_EQ( norm(v), 5.0 );
    }
    {
        Vector2d v(4.0, 3.0);
        EXPECT_EQ( norm(v), 5.0 );
    }
    {
        Vector2d v(0.0, 0.0);
        EXPECT_TRUE( std::fabs(v.norm()) < 0.0001 );
    }
    {
        Vector2d v(2.0, 0.0);
        EXPECT_TRUE( std::fabs(v.norm() - 2.0) < 0.0001 );
    }
    {
        Vector2d v(0.0, 2.0);
        EXPECT_TRUE( std::fabs(v.norm() - 2.0) < 0.0001 );
    }
    {
        Vector2d v(1.0, 1.0);
        EXPECT_TRUE( std::fabs(v.norm() - std::sqrt(2.0)) < 0.0001 );
        EXPECT_TRUE( std::fabs(v.norm() - norm(v)) < 0.0001 );
        EXPECT_TRUE( std::fabs(v.norm() - norm_2(v)) < 0.0001 );
    }
}

TEST(test_vector2d, normalized){
    {
        Vector2d v(1.0, 0.0);
        EXPECT_EQ( normalized(v), Vector2d(1.0,0.0) );
    }
    {
        Vector2d v(2.0, 0.0);
        EXPECT_EQ( normalized(v), Vector2d(1.0,0.0) );
    }
    {
        Vector2d v(0.0, 1.0);
        EXPECT_EQ( normalized(v), Vector2d(0.0,1.0) );
    }
    {
        Vector2d v(0.0, 2.0);
        EXPECT_EQ( normalized(v), Vector2d(0.0,1.0) );
    }
    {
        Vector2d v(3.0, 4.0);
        EXPECT_EQ( normalized(v), Vector2d(3.0/5.0,4.0/5.0) );
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
