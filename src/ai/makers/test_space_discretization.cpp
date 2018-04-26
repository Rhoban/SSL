#include <gtest/gtest.h>
#include "space_discretization.h"

TEST(test_space_discretization, getX){
    {
    	EXPECT_EQ(Space_discretization(1,2,0.2,0.3).getX(),1);
    	EXPECT_EQ(Space_discretization(10.0,2.0,0.2).getX(),50);
    	EXPECT_EQ(Space_discretization(10.0,2.0).getX(),50);
    }
}

TEST(test_space_discretization, getY){
    {
    	EXPECT_EQ(Space_discretization(1,2,0.2,0.3).getY(),2);
    	EXPECT_EQ(Space_discretization(10.0,2.0,0.2).getY(),10);
    	EXPECT_EQ(Space_discretization(10.0,2.0).getY(),10);
    }
}

TEST(test_space_discretization, getEX){
    {
        EXPECT_EQ(Space_discretization(1,2,0.2,0.3).getEX(),0.2);
        EXPECT_EQ(Space_discretization(10.0,2.0,0.2).getEX(),0.2);
        EXPECT_EQ(Space_discretization(10.0,2.0).getEX(),0.2);
    }
}

TEST(test_space_discretization, getEY){
    {
        EXPECT_EQ(Space_discretization(1,2,0.2,0.3).getEY(),0.3);
        EXPECT_EQ(Space_discretization(10.0,2.0,0.2).getEY(),0.2);
        EXPECT_EQ(Space_discretization(10.0,2.0).getEY(),0.2);
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}