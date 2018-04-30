#ifndef __CIRCULAR_VECTOR_H__
#define __CIRCULAR_VECTOR_H__

#include <vector>
#include <algorithm>
#include <assert.h>

template <typename T>
class circular_vector {
    private:
        std::vector<T> vector;
        unsigned int index;
    public:
        circular_vector():
            vector(0), index(0)
        { }
        circular_vector( unsigned int size):
            vector(size), index(0)
        { }
        circular_vector( const circular_vector<T> & cv ):
            vector(cv.vector), index(cv.index)
        { }

        unsigned int size() const { return vector.size(); }
        void resize( unsigned int size ){
            std::rotate(
                vector.begin(), vector.begin()+index,
                vector.end()
            );
            index = 0;
            vector.resize(size);
        }

        void insert( const T& element){
            if( index == 0 ){
                index = vector.size();
            }
            index -= 1;
            vector[ index ] = element;
        }

        const T& operator[]( unsigned int i) const {
            assert( i < vector.size() );
            if( index+i < vector.size() ){
                return vector[index+i];
            }else{
                return vector[index+i-vector.size()];
            }
        }
        T& operator[]( unsigned int i){
            assert( i < vector.size() );
            if( index+i < vector.size() ){
                return vector[index+i];
            }else{
                return vector[index+i-vector.size()];
            }
        }
    template <typename U>
    friend std::ostream& operator<< (
        std::ostream& stream, const circular_vector<U> & vec
    );
};

template <typename T>
std::ostream& operator<<(
    std::ostream& stream, const circular_vector<T> & vec
){
    assert( vec.size() >= 1 );
    stream << "(";
    for( unsigned int i=0; i< vec.size()-1; i++ ){
        stream << vec[i] << ", ";  
    }
    stream << vec[vec.size()-1];
    stream << ")";
    return stream;
}
#endif
