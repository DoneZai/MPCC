#include <types.h>
#include <iostream>

using namespace std;
struct State {
  double X;
  double Y;
};


void arrayToState(double *xk)
{
  for (int i = 0; i <= 2; i++) {
    cout << xk[i] << endl;
  }
  cout << "hahaha" << endl;
};


int main()
{
  double xtraj[6];
  for (int ii = 0; ii < 6; ii++) {
    xtraj[ii]=ii;
  }
  for (int i = 0; i <= 1; i++) {
    arrayToState(&xtraj[i * 3]);
  }
  State x1;
  x1.X=0;
  x1.Y=0;
  State x2;
  x2.X=123;
  x2.Y=456;
  x1=x2;
  cout << x1.X <<" "<< x1.Y << endl;
  return 0;
};
