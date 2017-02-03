#include <stdio.h>
#include <stdlib.h>
//#include <windows.h>
#include <deque>

#include "opencv/highgui.h"
#include "opencv2/opencv.hpp"
#include <opencv2/video/tracking.hpp>

#include "../stasm/stasm_lib.h"
#include "../stasm/stasm_landmarks.h"

#include "../VisualStudio/gazedir.h"
#include "../VisualStudio/drowsydet.h"

/*
Global variables
*/
static const char* path = "./img.jpg";
cv::VideoCapture cap(0);
int foundface;
float landmarks[2 * stasm_NLANDMARKS];
float landmarks0[2 * stasm_NLANDMARKS];

//Optical flow
cv::Mat prev_gray;
std::vector<cv::Point2f> features_prev, features_next;
std::vector<uchar> status;
std::vector<float> err;

//gaze degree
float beta;

//drowsy detection
std::deque<float> eyeOpenRate (60, 0.9);
float openRate, sumOpenRate = 54;
int closeNum = 0;

//length
int lEyeLen = 0, rEyeLen = 0;
int faceLen = 0, faceWid = 0;

//new Mat
cv::Mat cropped;
cv::Mat dst, detected_edges; 


/*
function declared
*/
void compute();
void featuresMotion();
void wrinkles();
void cropImg(cv::Mat_<unsigned char> frame);
void cannyEdge(cv::Mat_<unsigned char> frame);
void gaborFilter(cv::Mat_<unsigned char> cropped);

/*
function defined in main.cpp
*/

void compute() {
	lEyeLen = int(landmarks[30*2] - landmarks[34*2]);
	rEyeLen = int(landmarks[44*2] - landmarks[40*2]);
	faceLen = int(landmarks[6*2+1] - landmarks[14*2+1]);
	faceWid = int(landmarks[11*2] - landmarks[1*2]);
	
}

void featuresMotion(){
	float eyeMiddleL = (landmarks[30*2+1] + landmarks[34*2+1])/2;
	float eyeMiddleL0 = (landmarks0[30*2+1] + landmarks0[34*2+1])/2;
	float eyeMiddleR = (landmarks[40*2+1] + landmarks[44*2+1])/2;
	float eyeMiddleR0 = (landmarks0[40*2+1] + landmarks0[44*2+1])/2;

	float innerBrow = ((landmarks[21*2+1] - eyeMiddleL + landmarks[22*2+1] - eyeMiddleR) - (landmarks0[21*2+1] - eyeMiddleL0 + landmarks0[22*2+1] - eyeMiddleR0)) / (landmarks0[21*2+1] - eyeMiddleL0 + landmarks0[22*2+1] - eyeMiddleR0);
	float outerBrow = ((landmarks[18*2+1] - eyeMiddleL + landmarks[25*2+1] - eyeMiddleR) - (landmarks0[18*2+1] - eyeMiddleL0 + landmarks0[25*2+1] - eyeMiddleR0)) / (landmarks0[18*2+1] - eyeMiddleL0 + landmarks0[25*2+1] - eyeMiddleR0);
	float browsDis = (landmarks[22*2] - landmarks[21*2]) - (landmarks0[22*2+1] - landmarks0[21*2+1]);
	float eyeHeight = openRate;
	float eyeTopLip = ((landmarks[32*2+1] - eyeMiddleL + landmarks[42*2+1] - eyeMiddleR) - (landmarks0[32*2+1] - eyeMiddleL0 + landmarks0[42*2+1] - eyeMiddleR0)) / (landmarks0[32*2+1] - eyeMiddleL0 + landmarks0[43*2+1] - eyeMiddleR0);
	float eyeBottomLip = ((landmarks[36*2+1] - eyeMiddleL + landmarks[46*2+1] - eyeMiddleR) - (landmarks0[36*2+1] - eyeMiddleL0 + landmarks0[46*2+1] - eyeMiddleR0)) / (landmarks0[36*2+1] - eyeMiddleL0 + landmarks0[46*2+1] - eyeMiddleR0);

	std::cout << "Upper face features: "<<innerBrow<< ", " <<outerBrow<< ", " <<browsDis<< ", " <<eyeHeight<< ", "<<eyeTopLip<< ", "<<eyeBottomLip << std::endl;

	float lipMiddle = landmarks[59*2+1] + landmarks[65*2+1];
	float lipMiddle0 = landmarks0[59*2+1] + landmarks0[65*2+1];

	float lipH = (landmarks[62*2+1] + landmarks[74*2+1] - lipMiddle - (landmarks0[62*2+1] + landmarks0[74*2+1] - lipMiddle0))/(landmarks0[62*2+1] + landmarks0[74*2+1] - lipMiddle0);
	float lipW = ((landmarks[65*2] - landmarks[59*2]) - (landmarks0[65*2] - landmarks0[59*2]))/(landmarks0[65*2] - landmarks0[59*2]);
	float lipCornerL = ((landmarks[59*2] - lipMiddle) - (landmarks0[59*2] - lipMiddle0)) / (landmarks0[59*2] - lipMiddle0);
	float lipCornerR = (landmarks[65*2] - lipMiddle) - (landmarks0[65*2] - lipMiddle0) / (landmarks0[65*2] - lipMiddle0);

	std::cout << "Lower face features: "<<lipH<< ", " <<lipW<< ", " <<lipCornerL<< ", " <<lipCornerR <<std::endl;

	std::copy(std::begin(landmarks),std::end(landmarks),std::begin(landmarks0));
}

void drowsyDetect1() {
	openRate = (landmarks[36*2+1] - landmarks[32*2+1]) / (lEyeLen) + (landmarks[46*2+1] - landmarks[42*2+1]) / (rEyeLen);
	std::cout << "Open Rate: " << openRate << std::endl;

	eyeOpenRate.push_front(openRate);
		
	sumOpenRate += eyeOpenRate.front();
	sumOpenRate -= eyeOpenRate.back();

	eyeOpenRate.pop_back();

	if(sumOpenRate / 60 < 0.79) {
		drowsy();
		std::cout << "================Drowsy Driving Detected====================" << std::endl;
	}
}

void drowsyDetect2() {
	openRate = (landmarks[36*2+1] - landmarks[32*2+1]) / (lEyeLen) + (landmarks[46*2+1] - landmarks[42*2+1]) / (rEyeLen);
	std::cout << "Open Rate: " << openRate << std::endl;

	if(openRate < 0.70) {
		closeNum++;
	} else {
		closeNum = 0;
	}

	if(closeNum == 60) {
		closeNum--;
		drowsy();
		std::cout << "================Drowsy Driving Detected====================" << std::endl;
	}
}

void cropImg(cv::Mat_<unsigned char> frame) {
	int adjust = lEyeLen;
	cv::Rect myROI((int)landmarks[0] , (int)landmarks[14*2+1] , faceWid + adjust, faceLen + adjust);
	// Copy the data into new matrix
	frame(myROI).copyTo(cropped);
	imshow( "cropped", cropped );
}

void cannyEdge(cv::Mat_<unsigned char> img){
	dst.create(img.size(), img.type());
	cv::namedWindow( "Edge Map", CV_WINDOW_AUTOSIZE );
	/// Reduce noise with a kernel 2x2
	cv::blur( img, detected_edges, cv::Size(2,2) );

	/// Canny detector
	cv::Canny( detected_edges, detected_edges, 25, 25*3, 3 );
	//cv::Canny( frame, detected_edges, 22, 22*3, 3 );

	/// Using Canny's output as a mask, we display our result
	dst = cv::Scalar::all(0);

	img.copyTo( dst, detected_edges);
    imshow( "Edge Map", dst );
}


void wrinkles(){
	int yEnd = (int)((landmarks[30*2+1] + landmarks[40*2+1])/2 - landmarks[14*2+1]);
	int count = 0;
	//
	for(int x = int(landmarks[21*2] - landmarks[0]); x < landmarks[22*2]- landmarks[0]; x++) {
		for(int y = int(landmarks[21*2+1] - landmarks[14*2+1]); y < yEnd; y++) {
			if((int)dst.at<uchar>(x,y) > 0)
				count++;
		}
	}
	std::cout<< "nasal wrinkles: " << count << std::endl;
}

void gaborFilter(cv::Mat_<unsigned char> cropped){
	int kernel_size = 3;
    double sig = 5, theta = 100, th = 0, lm = 8, gm = 0.02, ps = 0;
	cv::Mat kernel;
    cv::Mat src, dest;
	//src.convertTo(cropped,CV_32F, 1.0/255, 0);
    kernel = cv::getGaborKernel(cv::Size(kernel_size,kernel_size), sig, theta, lm, gm, ps);
	cv::filter2D(cropped, dest, CV_32F, kernel);
	//std::cerr << dest(cv::Rect(30,30,10,10)) << std::endl; // peek into the data
	imshow("k",kernel);
	//cv::Mat viz;
	//dest.convertTo(viz,CV_8U,1.0/255.0);     // move to proper[0..255] range to show it

	//imshow("d",viz);
}