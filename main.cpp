#include "../VisualStudio/main.h"



void preprocess();

void preprocess(){
	compute();

		featuresMotion();

		

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

		

}


int main(int argc, char** argv){

	if(!init()){
		std::cout << "error in initialization" << std::endl;
		std::cin.get();// pause the console
		return -1;
	}
	
	try{

		while(true){
			prev_frame = frame.clone();
			cap >> img; // get a new frame from camera
			cv::cvtColor(img, frame, CV_BGR2GRAY);

			cv::calcOpticalFlowPyrLK(prev_frame, frame, features_prev, features_next, status, err);
			//std::copy(features_next.begin(), features_next.end(), landmarks);

			/*=============================================== show video ==============================================================*/
			force_points_into_image(features_next, frame.cols, frame.rows);
			for(int i = 0; i < stasm_NLANDMARKS; i++)
				frame(features_next[i]) = 255;

			//cv::flip(frame, output, 1);//mirror output
			cv::imshow("camera", frame);
			if(cv::waitKey(1) >= 0) break;
		}//while
	}//try
	catch( cv::Exception & e){
		std::cerr << e.msg << std::endl;
		std::cin.get();// pause the console
	}

	return 0;
}