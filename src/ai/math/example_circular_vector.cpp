#include "circular_vector.h"

int main()
{
  CircularVector<int> v(3);  // create  a circular vector for three elements: 0 0 0
  v.insert(11);              // v is now 11 0 0
  v.insert(15);              // v is now 15 11 0
  v.insert(20);              // v is now 20 15 11
  v.insert(10);              // v is now v[0]==10   v[1]==20 and v[2]==15
  return 0;
}
