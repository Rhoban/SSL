#include <gtest/gtest.h>

#include <debug.h>
#include "vector.h"

TEST(test_vector, vectorial_product){
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

TEST(test_vector, norm){
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
}

TEST(test_vector, normalized){
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
