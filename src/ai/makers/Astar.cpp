#include "Astar.h"

Grid::Grid(int X, int Y, double eX, double eY, pair<int,int> s, pair<int,int> g, vector< pair<int,int> > o):X(X),Y(Y),edgeX(eX),edgeY(eY),start(s),goal(g){
  vector< pair<int,int> >::const_iterator it;
  for(it = o.begin(); it != o.end(); ++it){
    obstacles[(*it)] = 1;
  }
}

int Grid::heuristic_cost_estimate(pair<int,int> p1, pair<int,int> p2){
  return abs(p1.first-p2.first)+abs(p1.second-p2.second);
}

vector< pair<int,int> > Grid::findN(pair<int,int> p){
  vector< pair<int,int> > v;
  if(p.first+1 <= X){
    v.push_back(make_pair(p.first+1,p.second));
  }
  if(p.second+1 <= Y){
    v.push_back(make_pair(p.first,p.second+1));
  }
  if(p.first-1 >= 1){
    v.push_back(make_pair(p.first-1,p.second));
  }
  if(p.second-1 >= 1){
    v.push_back(make_pair(p.first,p.second-1));
  }
  if(p.first+1 <= X && p.second+1 <= Y){
    v.push_back(make_pair(p.first+1,p.second+1));
  }
  if(p.first+1 <= X && p.second-1 >= 1){
    v.push_back(make_pair(p.first+1,p.second-1));
  }
  if(p.first-1 >= 1 && p.second+1 <= Y){
    v.push_back(make_pair(p.first-1,p.second+1));
  }
  if(p.first-1 >= 1 && p.second-1 >= 1){
    v.push_back(make_pair(p.first-1,p.second-1));
  }
  return v;
}

int Grid::dist_between(pair<int,int> p1, pair<int,int> p2){
  if(abs(p1.first-p2.first)+abs(p1.second-p2.second)==1){
    return 10;
  }
  else{
    return 14;
  }
}

vector< pair<int,int> > Grid::reconstruct_path(map< pair<int,int>, pair<int,int> > m, pair<int,int> p){
  vector< pair<int,int> > vp;
  vp.push_back(p);
  while(m.find(p) != m.end()){
    p = m[p];
    vp.push_back(p);
  }
  return vp;
}

vector< pair<int,int> > Grid::A(){
  set< pair<int,int> > closedSet;
  set< pair<int,int> > openSet;
  openSet.insert(start);
  map< pair<int,int>, pair<int,int> > cameFrom;
  map< pair<int,int>, int> gScore;
  for(int i = 1; i <= X; i++){
    for(int j = 1; j <= Y; j++){
      gScore[make_pair(i,j)] = INFINITY;
    }
  }
  gScore[start] = 0;
  map<pair<int,int>,int> fScore;
  for(int i = 1; i <= X; i++){
    for(int j = 1; j <= Y; j++){
      fScore[make_pair(i,j)] = INFINITY;
    }
  }
  fScore[start] = heuristic_cost_estimate(start,goal);
  pair<int,int> current;
  int min;
  vector< pair<int,int> > neighbors;
  int tentative_gScore;
  while(!openSet.empty()){
    min = INFINITY;
    for(set< pair<int,int> >::const_iterator is = openSet.begin(); is != openSet.end(); ++is){
      if(fScore[(*is)] < min){
	min = fScore[(*is)];
	current = (*is);
      }
    }
    if(current == goal){
      return reconstruct_path(cameFrom,current);
    }
    openSet.erase(current);
    closedSet.insert(current);
    neighbors = findN(current);
    for(vector< pair<int,int> >::const_iterator in = neighbors.begin(); in != neighbors.end(); ++in){
      if(closedSet.find(*in)!=closedSet.end() || obstacles.find(*in)!=obstacles.end()){
	continue;
      }
      if(openSet.find(*in)==openSet.end()){
	openSet.insert(*in);
      }
      tentative_gScore = gScore[current] + dist_between(current,*in);
      if(tentative_gScore < gScore[*in]){
	cameFrom[*in] = current;
	gScore[*in] = tentative_gScore;
	fScore[*in] = gScore[*in] + heuristic_cost_estimate(*in,goal);
      }
    }
  }
  vector< pair<int,int> > b;
  return b;
}

vector< pair<double,double> > Grid::Astar(){
  vector< pair<int,int> > p = A();
  vector< pair<double,double> > d;
  for(vector< pair<int,int> >::iterator it = p.begin(); it!=p.end(); ++it){
    d.push_back(make_pair(((it->first)-1)*edgeX+edgeX/2,((it->second)-1)*edgeY+edgeY/2));
  }
  return d;
}

