#include <gtest/gtest.h>

#include <debug.h>
#include "circular_vector.h"
#include <sstream>

TEST(test_circular_vector, Constructors){
    {
        circular_vector<double> vec(0);
        EXPECT_TRUE( vec.size() == 0 );
    }
    {
        circular_vector<double> vec;
        EXPECT_TRUE( vec.size() == 0 );
    }
    {
        circular_vector<double> vec(3);
        EXPECT_TRUE( vec.size() == 3 );
        for( int i = 0 ; i<vec.size(); i++ ){
            EXPECT_TRUE( vec[i] == 0 );
        }
    }
}

TEST(test_circular_vector, insert){
    {
        circular_vector<double> vec(3);
        vec.insert(3);
        EXPECT_TRUE( vec[0] == 3 );
        EXPECT_TRUE( vec[1] == 0 );
        EXPECT_TRUE( vec[2] == 0 );
        
        vec.insert(7);
        EXPECT_TRUE( vec[0] == 7 );
        EXPECT_TRUE( vec[1] == 3 );
        EXPECT_TRUE( vec[2] == 0 );

        vec.insert(13);
        EXPECT_TRUE( vec[0] == 13 );
        EXPECT_TRUE( vec[1] == 7 );
        EXPECT_TRUE( vec[2] == 3 );
        
        vec.insert(-2);
        EXPECT_TRUE( vec[0] == -2 );
        EXPECT_TRUE( vec[1] == 13 );
        EXPECT_TRUE( vec[2] == 7 );
    }
}


TEST(test_circular_vector, size){
    {
        circular_vector<double> vec(42);
        EXPECT_TRUE( vec.size() == 42 );
    }
    {
        circular_vector<double> vec(0);
        EXPECT_TRUE( vec.size() == 0 );
    }
    {
        circular_vector<double> vec;
        EXPECT_TRUE( vec.size() == 0 );
    }
}

TEST(test_circular_vector, resize){
    {
        circular_vector<double> vec(5);

        vec.insert(1);
        vec.insert(2);
        vec.insert(3);
        vec.insert(4);
        vec.insert(5);
        vec.insert(6);
        vec.insert(7);

        EXPECT_TRUE( vec[0] == 7 );
        EXPECT_TRUE( vec[1] == 6 );
        EXPECT_TRUE( vec[2] == 5 );
        EXPECT_TRUE( vec[3] == 4 );
        EXPECT_TRUE( vec[4] == 3 );

        EXPECT_TRUE( vec.size() == 5 );

        vec.resize(10);

        EXPECT_TRUE( vec.size() == 10 );

        EXPECT_TRUE( vec[0] == 7 );
        EXPECT_TRUE( vec[1] == 6 );
        EXPECT_TRUE( vec[2] == 5 );
        EXPECT_TRUE( vec[3] == 4 );
        EXPECT_TRUE( vec[4] == 3 );

        vec.insert(8);
        vec.insert(9);
        vec.insert(10);

        EXPECT_TRUE( vec[0] == 10 );
        EXPECT_TRUE( vec[1] == 9 );
        EXPECT_TRUE( vec[2] == 8 );
        EXPECT_TRUE( vec[3] == 7 );
        EXPECT_TRUE( vec[4] == 6 );
        EXPECT_TRUE( vec[5] == 5 );
        EXPECT_TRUE( vec[6] == 4 );
        EXPECT_TRUE( vec[7] == 3 );

        vec.resize(4);

        EXPECT_TRUE( vec.size() == 4 );

        EXPECT_TRUE( vec[0] == 10 );
        EXPECT_TRUE( vec[1] == 9 );
        EXPECT_TRUE( vec[2] == 8 );
        EXPECT_TRUE( vec[3] == 7 );
    }
}


TEST(test_circular_vector, getters){
    {
        circular_vector<double> vec(4);

        vec.insert(1);
        vec.insert(2);
        vec.insert(3);
        vec.insert(4);
        
        EXPECT_TRUE( vec[0] == 4 );
        EXPECT_TRUE( vec[1] == 3 );
        EXPECT_TRUE( vec[2] == 2 );
        EXPECT_TRUE( vec[3] == 1 );

        vec[0] = 14;
        vec[1] = 13;
        vec[2] = 12;
        vec[3] = 11;

        EXPECT_TRUE( vec[0] == 14 );
        EXPECT_TRUE( vec[1] == 13 );
        EXPECT_TRUE( vec[2] == 12 );
        EXPECT_TRUE( vec[3] == 11 );
    }
}

TEST(test_circular_vector, stream){
    {
        circular_vector<int> vec(4);

        vec.insert(11);
        vec.insert(12);
        vec.insert(13);
        vec.insert(14);
        
        std::ostringstream s1;
        s1 << vec;

        EXPECT_TRUE( "(14, 13, 12, 11)" == s1.str() ); 

        vec[1] = 42;
        
        s1.str("");
        s1.clear();
        s1 << vec;
        EXPECT_TRUE( "(14, 42, 12, 11)" == s1.str() ); 
    }
}

TEST(test_circular_vector, copy){
    {
        circular_vector<int> vec(4);

        vec.insert(1);
        vec.insert(2);
        vec.insert(3);
        vec.insert(4);

        circular_vector<int> vec1(vec);

        EXPECT_TRUE( vec1.size() == 4 );
        EXPECT_TRUE( vec1[0] == 4 );
        EXPECT_TRUE( vec1[1] == 3 );
        EXPECT_TRUE( vec1[2] == 2 );
        EXPECT_TRUE( vec1[3] == 1 );
    }
    {
        circular_vector<int> vec(4);

        vec.insert(1);
        vec.insert(2);
        vec.insert(3);
        vec.insert(4);

        circular_vector<int> vec1;
        vec1 = vec;

        EXPECT_TRUE( vec1.size() == 4 );
        EXPECT_TRUE( vec1[0] == 4 );
        EXPECT_TRUE( vec1[1] == 3 );
        EXPECT_TRUE( vec1[2] == 2 );
        EXPECT_TRUE( vec1[3] == 1 );
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
