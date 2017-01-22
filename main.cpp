#include <stdio.h>
#include <stdlib.h>
//#include <windows.h>
#include <deque>

#include "opencv/highgui.h"
#include "opencv2/opencv.hpp"
#include "../stasm/stasm_lib.h"
#include "../stasm/stasm_landmarks.h"

#include "../vc10/gazedir.h"
#include "../vc10/drowsydet.h"

/*
Global variables
*/
static const char* path = "./img.jpg";
cv::VideoCapture cap(0);
int foundface;
float landmarks[2 * stasm_NLANDMARKS];

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
function declared in main.cpp
*/
void compute();
void cropImg(cv::Mat_<unsigned char> frame);
void cannyEdge(cv::Mat_<unsigned char> frame);


/*
function defined in main.cpp
*/
void compute() {
	lEyeLen = landmarks[30*2] - landmarks[34*2];
	rEyeLen = landmarks[44*2] - landmarks[40*2];
	faceLen = landmarks[6*2+1] - landmarks[14*2+1];
	faceWid = landmarks[11*2] - landmarks[1*2];
}

void cropImg(cv::Mat_<unsigned char> frame) {
	int adjust = lEyeLen/1.7;
	cv::Rect myROI(landmarks[0] , landmarks[14*2+1] , faceWid + adjust, faceLen + adjust);
	// Copy the data into new matrix
	frame(myROI).copyTo(cropped);
	imshow( "cropped", cropped );
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
		std::cout << "====================================" << std::endl;
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
		std::cout << "====================================" << std::endl;
	}
}

void cannyEdge(cv::Mat_<unsigned char> img){
	dst.create(img.size(), img.type());
	cv::namedWindow( "Edge Map", CV_WINDOW_AUTOSIZE );
	/// Reduce noise with a kernel 2x2
	cv::blur( img, detected_edges, cv::Size(2,2) );

	/// Canny detector
	cv::Canny( detected_edges, detected_edges, 20, 20*3, 3 );
	//cv::Canny( frame, detected_edges, 22, 22*3, 3 );

	/// Using Canny's output as a mask, we display our result
	dst = cv::Scalar::all(0);

	img.copyTo( dst, detected_edges);
    imshow( "Edge Map", dst );
}

int main(int argc, char** argv){

	cv::Mat_<unsigned char> img, frame;

	if(!cap.isOpened()){

		std::cout<<"camera is not opened";
		return -1;
	}

	std::cout<<"Reading Habits Monitor is working...";
		while(true){

        cap >> img; // get a new frame from camera
		cv::cvtColor(img, frame, CV_BGR2GRAY);

		if(!frame.data){
			std::cout<<"========== No image loaded !!!!!!! ===============";
			return -1;
		}

		if(!stasm_search_single(&foundface, landmarks, (const char*)frame.data, frame.cols, frame.rows, path, "../data")){
			std::cout<<"stasm_search_single failed";
			return -1;
		}

        if (!foundface)
			std::cout<<"=============== No face found !!!!!!! ==============="<<std::endl;

		stasm_force_points_into_image(landmarks,frame.cols, frame.rows);
		//for(int i = 0; i < stasm_NLANDMARKS; i++)
		//	frame(cvRound(landmarks[i*2+1]), cvRound(landmarks[i*2])) = 255;

		compute();

		/*=============================================== Gaze Direction ==============================================================*/
		beta = betaAngle(
			horDegree(
				90, 
				90, 
				absDistance(landmarks[30*2], landmarks[38*2]), 
				absDistance(landmarks[38*2], landmarks[34*2])
			),
			horDegree(
				90, 
				90, 
				absDistance(landmarks[44*2], landmarks[39*2]), 
				absDistance(landmarks[39*2], landmarks[40*2])
				)
			);

		EOR(beta);

		std::cout << beta << std::endl;
		/*=============================================== Growsy Detection ==============================================================*/

		drowsyDetect2();

		/*=============================================== Canny Edge Detection ==============================================================*/
		if (foundface){
			cropImg(frame);
			cannyEdge(cropped);
		}
		

		//std::cout <<" data : " << (int)frame.at<uchar>(12,12) << std::endl;


		/*=============================================== show video ==============================================================*/
		//cv::flip(frame, output, 1);//mirror output
		//cv::imshow("camera", frame);
        if(cv::waitKey(1) >= 0) break;
	}

	return 0;
}