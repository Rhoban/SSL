#ifndef __PRINT_COLLECTION__H__
#define __PRINT_COLLECTION__H__

#include <iostream>
#include <set>
#include <utility>
#include <list>
#include <vector>
#include <map>

namespace std {

template <typename T>
std::ostream & operator<<(
    std::ostream & out, const std::set<T> & set
){
    out << "{";
    for( const std::string & elem : set ){
        out << elem << ", ";
    }
    out << "}";
    return out;
}

template <typename T>
std::ostream & operator<<(
    std::ostream & out, const std::list<T> & list
){
    out << "<";
    for( const T & elem : list ){
        out << elem << ", ";
    }
    out << ">";
    return out;
}

template <typename K, typename V>
std::ostream & operator<<(
    std::ostream & out, const std::map<K,V> & map
){
    out << "{";
    for( const std::pair<K,V> & elem : map ){
        out << elem.first << " : " << elem.second << ", ";
    }
    out << "}";
    return out;
}

template <typename F, typename S>
std::ostream & operator<<(
    std::ostream & out, const std::pair<F,S> & pair
){
    out << "(" << pair.first << ", " << pair.second << ")";
    return out;
}

template <typename V>
std::ostream & operator<<(
    std::ostream & out, const std::vector<V> & vector
){
    out << "[";
    for( unsigned int i=0; i<vector.size(); i++ ){
        out << vector.at(i) << ", ";
    }
    out << "]";
    return out;
}

}

#endif
