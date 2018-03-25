#include <gtest/gtest.h>

#include <debug.h>
#include "tangents.h"
#include "vector.h"

TEST(test_tangents, center_of_cone_incircle){
    {
        rhoban_geometry::Point cone_vertex(0.0, 0.0);
        rhoban_geometry::Point cone_base_A(0.0, -2.0);
        rhoban_geometry::Point cone_base_B(-2.0, 0.0);
        double circle_radius = 1.0;
        rhoban_geometry::Point circle_center = center_of_cone_incircle(
            cone_vertex, cone_base_A, cone_base_B, circle_radius
        );
        EXPECT_EQ( circle_center, rhoban_geometry::Point(-1.0, -1.0) );
    }
    {
        rhoban_geometry::Point cone_vertex(0.0, 0.0);
        rhoban_geometry::Point cone_base_A(-2.0, 0.0);
        rhoban_geometry::Point cone_base_B(0.0, -2.0);
        double circle_radius = 1.0;
        rhoban_geometry::Point circle_center = center_of_cone_incircle(
            cone_vertex, cone_base_A, cone_base_B, circle_radius
        );
        EXPECT_EQ( circle_center, rhoban_geometry::Point(-1.0, -1.0) );
    }
    {
        rhoban_geometry::Point cone_vertex(2.0, 3.0);
        rhoban_geometry::Point cone_base_A(0.0, 3.0);
        rhoban_geometry::Point cone_base_B(2.0, 1.0);
        double circle_radius = 1.0;
        rhoban_geometry::Point circle_center = center_of_cone_incircle(
            cone_vertex, cone_base_A, cone_base_B, circle_radius
        );
        EXPECT_EQ( circle_center, rhoban_geometry::Point(1.0, 2.0) );
    }
    {
        rhoban_geometry::Point cone_vertex(2.0, 3.0);
        rhoban_geometry::Point cone_base_A(2.0, 1.0);
        rhoban_geometry::Point cone_base_B(0.0, 3.0);
        double circle_radius = 1.0;
        rhoban_geometry::Point circle_center = center_of_cone_incircle(
            cone_vertex, cone_base_A, cone_base_B, circle_radius
        );
        EXPECT_EQ( circle_center, rhoban_geometry::Point(1.0, 2.0) );
    }
    {
        double d = std::sqrt(3)/2.0;
        rhoban_geometry::Point cone_vertex(1.0/2.0, d);
        rhoban_geometry::Point cone_base_A(0.0, 0.0);
        rhoban_geometry::Point cone_base_B(1.0, 0.0);
        double circle_radius = d/3.0;
        rhoban_geometry::Point circle_center = center_of_cone_incircle(
            cone_vertex, cone_base_A, cone_base_B, circle_radius
        );
        EXPECT_TRUE( norm( circle_center - rhoban_geometry::Point(1.0/2.0, d/3.0) ) < 0.00001 );
    }
    {
        double d = std::sqrt(3)/2.0;
        rhoban_geometry::Point cone_vertex(1.0/2.0, d);
        rhoban_geometry::Point cone_base_B(0.0, 0.0);
        rhoban_geometry::Point cone_base_A(1.0, 0.0);
        double circle_radius = d/3.0;
        rhoban_geometry::Point circle_center = center_of_cone_incircle(
            cone_vertex, cone_base_A, cone_base_B, circle_radius
        );
        EXPECT_TRUE( norm( circle_center - rhoban_geometry::Point(1.0/2.0, d/3.0) ) < 0.00001 );
    }
    {
        double d = std::sqrt(3)/2.0;
        rhoban_geometry::Point cone_vertex(2 + 1.0/2.0, 3 + d);
        rhoban_geometry::Point cone_base_A(2 + 0.0, 3 + 0.0);
        rhoban_geometry::Point cone_base_B(2 + 1.0, 3 + 0.0);
        double circle_radius = d/3.0;
        rhoban_geometry::Point circle_center = center_of_cone_incircle(
            cone_vertex, cone_base_A, cone_base_B, circle_radius
        );
        EXPECT_TRUE( norm( circle_center - rhoban_geometry::Point(2 + 1.0/2.0, 3 + d/3.0) ) < 0.00001 );
    }
    {
        double d = std::sqrt(3)/2.0;
        rhoban_geometry::Point cone_vertex(2 + 1.0/2.0, 3 + d);
        rhoban_geometry::Point cone_base_B(2 + 0.0, 3 + 0.0);
        rhoban_geometry::Point cone_base_A(2 + 1.0, 3 + 0.0);
        double circle_radius = d/3.0;
        rhoban_geometry::Point circle_center = center_of_cone_incircle(
            cone_vertex, cone_base_A, cone_base_B, circle_radius
        );
        EXPECT_TRUE( norm( circle_center - rhoban_geometry::Point(2 + 1.0/2.0, 3 + d/3.0) ) < 0.00001 );
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
