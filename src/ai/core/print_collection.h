#ifndef __PRINT_COLLECTION__H__
#define __PRINT_COLLECTION__H__

#include <iostream>
#include <set>
#include <list>
#include <map>

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
    out << "[";
    for( const std::string & elem : list ){
        out << elem << ", ";
    }
    out << "]";
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

#endif
