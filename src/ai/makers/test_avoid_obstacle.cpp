#include <gtest/gtest.h>
#include "avoid_obstacle.h"

TEST(test_avoid_obstacle, obstacle_constructor){
    {
        Obstacle obstacle( 2.0, rhoban_geometry::Point(3.0, 4.0) );
        EXPECT_EQ( obstacle.radius, 2.0 );
        EXPECT_EQ( obstacle.linear_position, rhoban_geometry::Point(3.0, 4.0) );
    }
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
