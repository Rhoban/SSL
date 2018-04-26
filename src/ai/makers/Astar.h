#ifndef __ASTAR_H__
#define __ASTAR_H__

#include <iostream>
#include <cstring>
#include <stdlib.h>
#include <utility>
#include <map>
#include <vector>
#include <string>
#include <set>
#include <cmath>
#include <algorithm>

using namespace std;

class Grid{
public:
  int X;
  int Y;
  double edgeX;
  double edgeY;
  pair<int,int> start;
  pair<int,int> goal;
  map<pair<int,int>,int> obstacles;
  
  Grid(int X, int Y, double eX, double eY, pair<int,int> s, pair<int,int> g, vector< pair<int,int> > o);
  
  int heuristic_cost_estimate(pair<int,int> p1, pair<int,int> p2);
  
  vector< pair<int,int> > findN(pair<int,int> p);
  
  int dist_between(pair<int,int> p1, pair<int,int> p2);
  
  vector< pair<int,int> > reconstruct_path(map< pair<int,int>, pair<int,int> > m, pair<int,int> p);
  
  vector< pair<int,int> > A();

  vector< pair<double,double> > Astar();
};

#endif /* __ASTAR_H__ */
