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

#include "collection.h"

TEST(test_collection, list2vector){
    {
        std::list<int> l;
        std::vector<int> v = list2vector(l);
        EXPECT_EQ( v.size(), 0);
    }
    {
        std::list<int> l;
        std::vector<int> v = list2vector(l,4);
        EXPECT_EQ( v.size(), 4);
    }
    {
        std::list<int> l = {0, 1, 2};
        std::vector<int> v = list2vector(l);
        EXPECT_EQ( v.size(), 3);
        int i = 0;
        for( int a : l ){
            EXPECT_EQ( a, v[i] );
            i++;    
        }
    }
    {
        std::list<int> l = {0, 1, 2};
        std::vector<int> v = list2vector(l,5);
        EXPECT_EQ( v.size(), 5);
        int i = 0;
        for( int a : l ){
            EXPECT_EQ( a, v[i] );
            i++;    
        }
    }
}


TEST(test_collection, vector2set){
    {
        std::vector<int> v;
        std::set<int> s = vector2set(v);
        EXPECT_EQ( s.size(), 0);
    }
    {
        std::vector<int> v = {2,0,1,2,4,4,4};
        std::set<int> s = vector2set(v);
        EXPECT_EQ(
            s, (
                std::set<int>( {0,1,2,4} )
            )  
        );
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
