#include <gtest/gtest.h>

#include "eigen_convertion.h"

TEST(test_eigen_convertion, eigen2point){
    {
        Eigen::Vector2d v(2.0, 3.0);
        rhoban_geometry::Point p = eigen2point(v);
        EXPECT_EQ(p, rhoban_geometry::Point(2.0, 3.0) );
    }
}

TEST(test_eigen_convertion, eigen2vector){
    {
        Eigen::Vector2d v(2.0, 3.0);
        Vector2d p = eigen2vector(v);
        EXPECT_EQ(p, Vector2d(2.0, 3.0) );
    }
}

TEST(test_eigen_convertion, point2eigen){
    {
        rhoban_geometry::Point p(2.0, 3.0);
        Eigen::Vector2d v = point2eigen(p);
        EXPECT_EQ(v, Eigen::Vector2d(2.0, 3.0) );
    }
}

TEST(test_eigen_convertion, vector2eigen){
    {
        Vector2d v1(2.0, 3.0);
        Eigen::Vector2d v2 = point2eigen(v1);
        EXPECT_EQ(v2, Eigen::Vector2d(2.0, 3.0) );
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
