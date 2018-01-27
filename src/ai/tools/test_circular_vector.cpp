#include <tools/debug.h>
#include "circular_vector.h"
#include <sstream>

void test_constructors(){
    {
        circular_vector<double> vec(0);
        assert( vec.size() == 0 );
    }
    {
        circular_vector<double> vec;
        assert( vec.size() == 0 );
    }
    {
        circular_vector<double> vec(3);
        assert( vec.size() == 3 );
        for( int i = 0 ; i<vec.size(); i++ ){
            assert( vec[i] == 0 );
        }
    }
}

void test_insert(){
    {
        circular_vector<double> vec(3);
        vec.insert(3);
        assert( vec[0] == 3 );
        assert( vec[1] == 0 );
        assert( vec[2] == 0 );
        
        vec.insert(7);
        assert( vec[0] == 7 );
        assert( vec[1] == 3 );
        assert( vec[2] == 0 );

        vec.insert(13);
        assert( vec[0] == 13 );
        assert( vec[1] == 7 );
        assert( vec[2] == 3 );
        
        vec.insert(-2);
        assert( vec[0] == -2 );
        assert( vec[1] == 13 );
        assert( vec[2] == 7 );
    }
}


void test_size(){
    {
        circular_vector<double> vec(42);
        assert( vec.size() == 42 );
    }
    {
        circular_vector<double> vec(0);
        assert( vec.size() == 0 );
    }
    {
        circular_vector<double> vec;
        assert( vec.size() == 0 );
    }
}

void test_resize(){
    {
        circular_vector<double> vec(5);

        vec.insert(1);
        vec.insert(2);
        vec.insert(3);
        vec.insert(4);
        vec.insert(5);
        vec.insert(6);
        vec.insert(7);

        assert( vec[0] == 7 );
        assert( vec[1] == 6 );
        assert( vec[2] == 5 );
        assert( vec[3] == 4 );
        assert( vec[4] == 3 );

        assert( vec.size() == 5 );

        vec.resize(10);

        assert( vec.size() == 10 );

        assert( vec[0] == 7 );
        assert( vec[1] == 6 );
        assert( vec[2] == 5 );
        assert( vec[3] == 4 );
        assert( vec[4] == 3 );

        vec.insert(8);
        vec.insert(9);
        vec.insert(10);

        assert( vec[0] == 10 );
        assert( vec[1] == 9 );
        assert( vec[2] == 8 );
        assert( vec[3] == 7 );
        assert( vec[4] == 6 );
        assert( vec[5] == 5 );
        assert( vec[6] == 4 );
        assert( vec[7] == 3 );

        vec.resize(4);

        assert( vec.size() == 4 );

        assert( vec[0] == 10 );
        assert( vec[1] == 9 );
        assert( vec[2] == 8 );
        assert( vec[3] == 7 );
    }
}


void test_getting(){
    {
        circular_vector<double> vec(4);

        vec.insert(1);
        vec.insert(2);
        vec.insert(3);
        vec.insert(4);
        
        assert( vec[0] == 4 );
        assert( vec[1] == 3 );
        assert( vec[2] == 2 );
        assert( vec[3] == 1 );

        vec[0] = 14;
        vec[1] = 13;
        vec[2] = 12;
        vec[3] = 11;

        assert( vec[0] == 14 );
        assert( vec[1] == 13 );
        assert( vec[2] == 12 );
        assert( vec[3] == 11 );
    }
}

void test_stream(){
    {
        circular_vector<int> vec(4);

        vec.insert(11);
        vec.insert(12);
        vec.insert(13);
        vec.insert(14);
        
        std::ostringstream s1;
        s1 << vec;

        assert( "(14, 13, 12, 11)" == s1.str() ); 

        vec[1] = 42;
        
        s1.str("");
        s1.clear();
        s1 << vec;
        assert( "(14, 42, 12, 11)" == s1.str() ); 
    }
}

void test_copy(){
    {
        circular_vector<int> vec(4);

        vec.insert(1);
        vec.insert(2);
        vec.insert(3);
        vec.insert(4);

        circular_vector<int> vec1(vec);

        assert( vec1.size() == 4 );
        assert( vec1[0] == 4 );
        assert( vec1[1] == 3 );
        assert( vec1[2] == 2 );
        assert( vec1[3] == 1 );
    }
    {
        circular_vector<int> vec(4);

        vec.insert(1);
        vec.insert(2);
        vec.insert(3);
        vec.insert(4);

        circular_vector<int> vec1;
        vec1 = vec;

        assert( vec1.size() == 4 );
        assert( vec1[0] == 4 );
        assert( vec1[1] == 3 );
        assert( vec1[2] == 2 );
        assert( vec1[3] == 1 );
    }
}

int main(){
    test_constructors();
    test_insert();
    test_size();
    test_resize();
    test_getting();
    test_stream();
    test_copy();
}
