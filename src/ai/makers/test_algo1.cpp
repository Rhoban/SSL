#include <gtest/gtest.h>
#include "algo1.h"

TEST(test_algo1, constructors){
    {
	Algo1 algo1( 3.0 );
        EXPECT_EQ( algo1.discretisation_size, 3.0 );
    }
}

TEST(test_algo1, linear_position){
    {
	Algo1 algo1( 3.0 );
        EXPECT_EQ(
		algo1.linear_position(0.0),  
		rhoban_geometry::Point( 0.0, 0.0 )
	);
        EXPECT_EQ(
		algo1.linear_position(0.5),  
		rhoban_geometry::Point( 0.0, 0.0 )
	);
        EXPECT_EQ(
		algo1.linear_position(1.0),  
		rhoban_geometry::Point( 0.0, 0.0 )
	);
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
