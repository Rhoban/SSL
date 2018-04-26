#include <gtest/gtest.h>
#include "Astar.h"

TEST(test_Astar, constructor){
  {
    vector< pair<int,int> > v;
    for(int i = 1; i < 7; i++){
      v.push_back(make_pair(3,i));
    }
    Grid Grid(6,7,1.0,2.0,make_pair(2,5),make_pair(5,2),v);
    EXPECT_EQ(Grid.X,6);
    EXPECT_EQ(Grid.Y,7);
    EXPECT_EQ(Grid.edgeX,1.0);
    EXPECT_EQ(Grid.edgeY,2.0);
    EXPECT_EQ(Grid.start,make_pair(2,5));
    EXPECT_EQ(Grid.goal,make_pair(5,2));
    int j = 1;
    for(map<pair<int,int>,int>::iterator it = (Grid.obstacles).begin(); it != (Grid.obstacles).end(); it++){
      EXPECT_EQ(it->first,make_pair(3,j));
      j++;
    }
  }
}

TEST(test_Astar, heuristic_cost_estimate){
  {
    vector< pair<int,int> > v;
    for(int i = 1; i < 7; i++){
      v.push_back(make_pair(3,i));
    }
    Grid Grid(6,7,1.0,1.0,make_pair(2,5),make_pair(5,2),v);
    EXPECT_EQ(Grid.heuristic_cost_estimate(make_pair(2,5),make_pair(5,2)),6);
    EXPECT_EQ(Grid.heuristic_cost_estimate(make_pair(5,2),make_pair(2,5)),6);
    EXPECT_EQ(Grid.heuristic_cost_estimate(make_pair(2,5),make_pair(2,5)),0);
  }
}

TEST(test_Astar, findN){
  {
    vector< pair<int,int> > v;
    for(int i = 1; i < 7; i++){
      v.push_back(make_pair(3,i));
    }
    Grid Grid(6,7,1.0,1.0,make_pair(2,5),make_pair(5,2),v);
    vector< pair<int,int> > p = Grid.findN(make_pair(2,5));
    EXPECT_EQ(p.size(),8);
    vector< pair<int,int> >::iterator jt, jf, jk;
    int a, b;
    jt = p.begin();
    jf = p.end();
    for(int k = -1; k<= 1; k++){
      for(int l = -1; l<= 1; l++){
	a = 2+k;
	b = 5+l;
	if(a!=2 && b!=5){
	  jk = find(jt,jf,make_pair(a,b));
	  EXPECT_NE(jk,jf);
	}
      }
    }
    jk = find(jt,jf,make_pair(4,5));
    EXPECT_EQ(jk,jf);
  }
}

TEST(test_Astar, dist_between){
  {
    vector< pair<int,int> > v;
    for(int i = 1; i < 7; i++){
      v.push_back(make_pair(3,i));
    }
    Grid Grid(6,7,1.0,1.0,make_pair(2,5),make_pair(5,2),v);
    EXPECT_EQ(Grid.dist_between(make_pair(2,3),make_pair(2,4)),10);
    EXPECT_EQ(Grid.dist_between(make_pair(2,3),make_pair(3,5)),14);
  }
}

TEST(test_Astar, A){
  {
    vector< pair<int,int> > v;
    v.push_back(make_pair(4,2));
    v.push_back(make_pair(4,3));
    v.push_back(make_pair(4,5));
    Grid Grid(6,7,1.0,2.0,make_pair(2,5),make_pair(5,2),v);
    vector< pair<int,int> > a = Grid.A();
    vector< pair<int,int> > g;
    g.push_back(make_pair(2,5));
    g.push_back(make_pair(3,5));
    g.push_back(make_pair(4,4));
    g.push_back(make_pair(5,3));
    g.push_back(make_pair(5,2));
    EXPECT_EQ(a.size(),5);
    vector< pair<int,int> >::iterator je, jt, jf, jk, jg;
    je = a.begin();
    jt = a.end();
    for(jf = g.begin(); jf!= g.end(); ++jf){
      jk = find(je,jt,*jf);
      EXPECT_NE(jk,jt);
    }
    EXPECT_EQ(find(je,jt,make_pair(1,1)),jt);
  }
}


TEST(test_Astar, Astar){
  {
    vector< pair<int,int> > v;
    v.push_back(make_pair(4,2));
    v.push_back(make_pair(4,3));
    v.push_back(make_pair(4,5));
    Grid Grid(6,7,1.0,2.0,make_pair(2,5),make_pair(5,2),v);
    vector< pair<double,double> > a = Grid.Astar();
    vector< pair<double,double> > g;
    g.push_back(make_pair(1.5,9.0));
    g.push_back(make_pair(2.5,9.0));
    g.push_back(make_pair(3.5,7.0));
    g.push_back(make_pair(4.5,5.0));
    g.push_back(make_pair(4.5,3.0));
    EXPECT_EQ(a.size(),5);
    vector< pair<double,double> >::iterator je, jt, jf, jk;
    je = a.begin();
    jt = a.end();
    for(jf = g.begin(); jf!= g.end(); ++jf){
      jk = find(je,jt,*jf);
      EXPECT_NE(jk,jt);
    }
    EXPECT_EQ(find(je,jt,make_pair(0.5,0.5)),jt);
  }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

