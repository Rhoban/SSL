#include "space_discretization.h"

int Space_discretization::getX(){
	return X;
}

int Space_discretization::getY(){
	return Y;
}

double Space_discretization::getEX(){
	return edgeX;
}

double Space_discretization::getEY(){
	return edgeY;
}

void Space_discretization::print_space_discretization(){
	cout << "X : " << Space_discretization::getX() << endl;
	cout << "Y : " << Space_discretization::getY() << endl;
	cout << "edgeX : " << Space_discretization::getEX() << endl;
	cout << "edgeY : " << Space_discretization::getEY() << endl;
}