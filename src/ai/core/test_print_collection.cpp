#include <gtest/gtest.h>

#include "print_collection.h"
#include <sstream>
#include <iostream>
#include <debug.h>

void clear( std::ostringstream & o ){
    o.str("");
    o.clear();
}


TEST(test_print_collection, print_list){
    std::ostringstream s;
    std::list<std::string> l;
   
    s << l;
    EXPECT_TRUE( s.str() == "[]" );
    clear(s);
 
    l.push_back( "voiture" );
    s << l;
    EXPECT_TRUE( s.str() == "[voiture, ]" );
    clear(s);

    l.push_back( "maison" );
    s << l;
    EXPECT_TRUE( s.str() == "[voiture, maison, ]" );
    clear(s);
}

TEST(test_print_collection, print_set){
    std::ostringstream s;
    std::set<std::string> set;
   
    s << set;
    EXPECT_TRUE( s.str() == "{}" );
    clear(s);
 
    set.insert( "voiture" );
    s << set;
    EXPECT_TRUE( s.str() == "{voiture, }" );
    clear(s);

    set.insert( "maison" );
    s << set;
    EXPECT_TRUE( 
        ( s.str() == "{voiture, maison, }" )
        or
        ( s.str() == "{maison, voiture, }" )
    );
    clear(s);
}

TEST(test_print_collection, print_map){
    std::ostringstream s;
    std::map<std::string, int> map;
   
    s << map;
    EXPECT_TRUE( s.str() == "{}" );
    clear(s);
 
    map["voiture"] = 1;
    s << map;
    EXPECT_TRUE( s.str() == "{voiture : 1, }" );
    clear(s);

    map["maison"] = 2;
    s << map;
    EXPECT_TRUE( 
        ( s.str() == "{voiture : 1, maison : 2, }" )
        or
        ( s.str() == "{maison : 2, voiture : 1, }" )
    );
    clear(s);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
