#include "print_collection.h"
#include <sstream>
#include <iostream>
#include <debug.h>
#include <assert.h>

void clear( std::ostringstream & o ){
    o.str("");
    o.clear();
}


void test_print_list(){
    std::ostringstream s;
    std::list<std::string> l;
   
    s << l;
    assert( s.str() == "[]" );
    clear(s);
 
    l.push_back( "voiture" );
    s << l;
    assert( s.str() == "[voiture, ]" );
    clear(s);

    l.push_back( "maison" );
    s << l;
    assert( s.str() == "[voiture, maison, ]" );
    clear(s);
}

void test_print_set(){
    std::ostringstream s;
    std::set<std::string> set;
   
    s << set;
    assert( s.str() == "{}" );
    clear(s);
 
    set.insert( "voiture" );
    s << set;
    assert( s.str() == "{voiture, }" );
    clear(s);

    set.insert( "maison" );
    s << set;
    assert( 
        ( s.str() == "{voiture, maison, }" )
        or
        ( s.str() == "{maison, voiture, }" )
    );
    clear(s);
}

void test_print_map(){
    std::ostringstream s;
    std::map<std::string, int> map;
   
    s << map;
    assert( s.str() == "{}" );
    clear(s);
 
    map["voiture"] = 1;
    s << map;
    assert( s.str() == "{voiture : 1, }" );
    clear(s);

    map["maison"] = 2;
    s << map;
    assert( 
        ( s.str() == "{voiture : 1, maison : 2, }" )
        or
        ( s.str() == "{maison : 2, voiture : 1, }" )
    );
    clear(s);
}

int main(){
    test_print_list();
    test_print_set();

    return 0;
}
