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
cv::Mat_<unsigned char> img, frame;

//Optical flow
cv::Mat_<unsigned char> prev_frame;
std::vector<cv::Point2f> features_prev, features_next;
//std::vector<float> features_next, features_prev;
std::vector<uchar> status;
std::vector<float> err;

//gaze degree
float beta;

//drowsy detection
std::deque<float> eyeOpenRate (60, (float)0.9);
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
int init();
void compute();
void featuresMotion();
void wrinkles();
void cropImg(cv::Mat_<unsigned char> frame);
void cannyEdge(cv::Mat_<unsigned char> frame);
void gaborFilter(cv::Mat_<unsigned char> cropped);

void force_points_into_image(std::vector<cv::Point2f> &pointV, int ncols, int nrows);

/*
function defined in main.cpp
*/

int init(){
	if(!cap.isOpened()){
		std::cout<<"camera is not opened";
		return 0;
	}

	cap >> img; // get a new frame from camera
	cv::cvtColor(img, frame, CV_BGR2GRAY);

	if(!frame.data){
		std::cout<<"========== No image loaded !!!!!!! ===============";
		return 0;
	}

	cap >> img; // get a new frame from camera
	cv::cvtColor(img, frame, CV_BGR2GRAY);

	if(!stasm_search_single(&foundface, landmarks, (const char*)frame.data, frame.cols, frame.rows, path, "../data")){
		std::cout<<"stasm_search_single failed";
		return 0;
	}

	if (!foundface)
		std::cout<<"=============== No face found !!!!!!! ==============="<<std::endl;

	stasm_force_points_into_image(landmarks,frame.cols, frame.rows);
	//std::vector<cv::Point> features_prev(std::begin(landmarks), std::end(landmarks));
	for(int i = 0; i < stasm_NLANDMARKS * 2; i += 2){
		features_prev.push_back(cv::Point2f(landmarks[i], landmarks[i+1]));
		
	}

	//features_prev.assign(landmarks, landmarks + stasm_NLANDMARKS * 2);

	return 1;
}

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

void gaborFilter(cv::Mat_<unsigned char> frame){
	//int kernel_size = 3; double sig = 5, theta = 100, th = 0, lm = 8, gm = 0.02, ps = 0;
	imshow("frame",frame);
    cv::Mat in, src, dest;
	int kernel_size = 31;
	double sig = 1, theta = 0, lm = 1.0, gm = 0.02, ps = 0;

	frame.copyTo(in);
	in.convertTo(src,CV_32F);
	//cropped.convertTo(src,CV_32F);
    cv::Mat kernel = cv::getGaborKernel(cv::Size(kernel_size,kernel_size), sig, theta, lm, gm, ps);
	cv::filter2D(src, dest, CV_32F, kernel);
	std::cerr << dest(cv::Rect(30,30,10,10)) << std::endl; // peek into the data
	imshow("k",kernel);
	cv::Mat viz;
	dest.convertTo(viz,CV_8U,1.0/255.0);     // move to proper[0..255] range to show it

	imshow("d",viz);
}

void force_points_into_image(std::vector<cv::Point2f> &pointV, int ncols, int nrows){
	for (int i = 0; i < stasm_NLANDMARKS; i++)
    {
		pointV[i].x = MIN(MAX(pointV[i].x, 0.f), float(ncols-1));//MIN(MAX(x, min), max), force x to a value between min and max;
		pointV[i].y = MIN(MAX(pointV[i].y, 0.f), float(nrows-1));
    }
}