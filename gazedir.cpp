#include "gazedir.h"



float absDistance(float x1, float x2) {
	return abs(x1 - x2);
}

float horDegree(float alpha, float theta, float d1, float d2){
	//return (theta - acos((2/(d1/d2+1))*sin(theta)*sin(alpha/2)+cos(alpha/2+theta)));
	return (theta - acos( ((d2*cos(theta - alpha/2)) + d1*sin(theta + alpha/2)) / (d1+d2)) );
}

float betaAngle(float lDegree, float rDegree) {
	return (lDegree + rDegree)/2;
}

void EOR(float beta) {
	float index = (beta - 88.7) * 100;
	if(index <= 1.2){
		std::cout << "--> --> --> --> --> -->"<< std::endl;
	} else if(index >7.5){
		std::cout << "<-- <-- <-- <-- <-- <--"<< std::endl;
	}
	
}