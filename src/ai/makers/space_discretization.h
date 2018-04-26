#ifndef __SPACE_DISCRETIZATION_H__
#define __SPACE_DISCRETIZATION_H__

#include <cstdio>
#include <iostream>
#include <cstring>
#include <stdlib.h>
#include <cmath>
#include <list>



using namespace std;

class Space_discretization{
	int X;
	int Y;
	double edgeX;
	double edgeY;
public:
	// constructor 1
	Space_discretization(int X, int Y, double eX, double eY):X(X), Y(Y), edgeX(eX), edgeY(eY){}

	// constructor 2
	Space_discretization(double W, double H, double edge){
		X = (int)(W/edge);
		Y = (int)(H/edge);
		edgeX = W/X;
		edgeY = H/Y;
	}

	// constructor 3
	// We suppose that H is always <= W
	Space_discretization(double W, double H){
		double e = H/W;
		X = (int)(W/e);
		Y = (int)(H/e);
		edgeX = W/X;
		edgeY = H/Y;
	}

	int getX();
	int getY();
	double getEX();
	double getEY();
	void print_space_discretization();

};

#endif /* __SPACE_DISCRETIZATION_H__*/