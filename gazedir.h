#ifndef GAZEDIR_H_
#define GAZEDIR_H_


#include <math.h>
#include <iostream>


float absDistance(float x1, float x2);
float horDegree(float alpha, float theta, float d1, float d2);
float betaAngle(float lDegree, float rDegree);
void EOR(float beta);

#endif 