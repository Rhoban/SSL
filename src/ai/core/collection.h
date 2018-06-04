#ifndef __CORE__COLLECTION__H__
#define __CORE__COLLECTION__H__

#include <list>
#include <vector>
#include <map>
#include <set>

template <typename DATA>
std::vector<DATA> list2vector(
    const std::list<DATA> & list,
    size_t vector_size
){
    assert( vector_size >= list.size() );
    std::vector<DATA> vec(vector_size);
    unsigned int i = 0;
    for( const DATA & data : list ){
        vec[i] = data;
        i++;
    }
    return vec;
}

template <typename DATA>
std::vector<DATA> list2vector(
    const std::list<DATA> & list
){
    return list2vector( list, list.size() );
}

template <typename DATA>
std::set<DATA> vector2set(
    const std::vector<DATA> & vec
){
    std::set<DATA> res;
    for( const DATA & data : vec ){
        res.insert( data );
    }
    return res;
}

template <typename KEY, typename VALUE>
std::vector< std::pair<KEY, VALUE> >
map2vector(
    const std::map<KEY,VALUE> & map
){
    std::vector< std::pair<KEY, VALUE> > result;
    int i=0;
    for( const std::pair<KEY,VALUE> & assoc : map ){
        result[i] = assoc;
        i++;
    }
    return result;
}

template <typename KEY, typename VALUE>
std::set< std::pair<KEY, VALUE> >
map2set(
    const std::map<KEY,VALUE> & map
){
    std::set< std::pair<KEY, VALUE> > result;
    for( const std::pair<KEY,VALUE> & assoc : map ){
        result.insert( assoc );
    }
    return result;
}

#endif
