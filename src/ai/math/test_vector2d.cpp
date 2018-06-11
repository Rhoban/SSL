/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

    SSL is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SSL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with SSL.  If not, see <http://www.gnu.org/licenses/>.
*/

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

TEST(test_vector2d, norm_square){
    {
        Vector2d v(0.0, 0.0);
        EXPECT_EQ( v.norm_square(), 0.0 );
    }
    {
        Vector2d v(1.0, 0.0);
        EXPECT_EQ( v.norm_square(), 1.0 );
    }
    {
        Vector2d v(0.0, 1.0);
        EXPECT_EQ( v.norm_square(), 1.0 );
    }
    {
        Vector2d v(3.0, 4.0);
        EXPECT_EQ( v.norm_square(), 25.0 );
    }
    {
        Vector2d v(4.0, 3.0);
        EXPECT_EQ( v.norm_square(), 25.0 );
    }
    {
        Vector2d v(0.0, 0.0);
        EXPECT_TRUE( std::fabs(v.norm_square()) < 0.0001 );
    }
    {
        Vector2d v(2.0, 0.0);
        EXPECT_TRUE( std::fabs(v.norm_square() - 4.0) < 0.0001 );
    }
    {
        Vector2d v(0.0, 2.0);
        EXPECT_TRUE( std::fabs(v.norm_square() - 4.0) < 0.0001 );
    }
    {
        Vector2d v(1.0, 1.0);
        EXPECT_TRUE( std::fabs(v.norm_square() - 2.0) < 0.0001 );
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

TEST(test_vector2d, vector2point){
    {
        Vector2d v(0.0, 0.0);
        rhoban_geometry::Point p = vector2point(v);
        EXPECT_EQ( p.getX(), 0.0 );
        EXPECT_EQ( p.getY(), 0.0 );
    }
    {
        Vector2d v(12.0, 42.0);
        rhoban_geometry::Point p = vector2point(v);
        EXPECT_EQ( p.getX(), v[0] );
        EXPECT_EQ( p.getY(), v[1] );
    }
}

TEST(test_vector2d, point2vector){
    {
        rhoban_geometry::Point p(0.0, 0.0);
        Vector2d v = point2vector(p);
        EXPECT_EQ( v[0], 0.0 );
        EXPECT_EQ( v[1], 0.0 );
    }
    {
        rhoban_geometry::Point p(12.0, 42.0);
        Vector2d v = point2vector(p);
        EXPECT_EQ( p.getX(), v[0] );
        EXPECT_EQ( p.getY(), v[1] );
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
