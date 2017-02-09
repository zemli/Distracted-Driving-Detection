#include "../VisualStudio/main.h"


int main(int argc, char** argv){

	cv::Mat_<unsigned char> img, frame;

	if(!cap.isOpened()){

		std::cout<<"camera is not opened";
		return -1;
	}

	std::cout<<"Reading Habits Monitor is working...";
	try{

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
		
		compute();

		featuresMotion();

		//cv::calcOpticalFlowPyrLK(

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

		//std::cout << beta << std::endl;
		/*=============================================== Growsy Detection ==============================================================*/

		drowsyDetect2();

		/*=============================================== Canny Edge Detection ==========================================================*/
		if (foundface){
			cropImg(frame);
			cannyEdge(cropped);
			wrinkles();
		/*=============================================== Gabor Filter ==============================================================*/
			
			gaborFilter(cropped);
		}
		

		//std::cout <<" data : " << (int)frame.at<uchar>(12,12) << std::endl;

		


		/*=============================================== show video ==============================================================*/
		for(int i = 0; i < stasm_NLANDMARKS; i++)
			frame(cvRound(landmarks[i*2+1]), cvRound(landmarks[i*2])) = 255;
		//cv::flip(frame, output, 1);//mirror output
		cv::imshow("camera", frame);
		if(cv::waitKey(1) >= 0) break;
	}
	}
	catch( cv::Exception & e){
		std::cerr << e.msg << std::endl;
		std::cin.get();// pause the console
	}

	return 0;
}