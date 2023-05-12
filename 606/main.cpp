#include"../Detector/ArmorDetector/ArmorDetector.hpp"
#include"../Detector/ApexDetector/ApexDetector.hpp"
#include"../PoseSolver/PoseSolver.hpp"
//#include"../Camera/mv_video_capture.hpp"
#include"../MVCamera/MVCamera.hpp"
#include"Serial/Serial.hpp"
#include"utils/fps.hpp"
//#include"../Predictor/KalmanPredictor.h"
#include"../KalmanFilter/KalmanFilter.h"
using namespace std;
using namespace cv;
using namespace apex_detector;
using namespace armor_detector;

int main()
{
	double last_pitch_;
	double last_yaw_;
	double delta_pitch, delta_yaw;
	double pitch_raw_angle_;
	double yaw_raw_angle_;
	double yaw_ecd_angle_;
	double gimbal_global_angle_;
	int missed_cnt_ = 0;
	int detected_cnt_ = 0;
	int sentry_miss_cnt_ = 0;
	double bullet_speed_;
	int detect_mode = 0;
    
	Serial serial =Serial("/home/wujyqaq/Desktop/auto_aim/Configs/serial/serial.xml");
	PoseSolver poseSolver=PoseSolver("/home/wujyqaq/Desktop/auto_aim/Configs/pose_solver/camera_params.xml",1);

	poseSolver.setObjPoints(smallArmor,135,55);
	poseSolver.setObjPoints(bigArmor,230,55);
	mindvision::MVCamera* mv_capture_ = new mindvision::MVCamera(mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_1024, mindvision::EXPOSURE_10000));
	
	cv::Mat src_img_;
	armor_detector::ArmorDetector armor_detector;
	
	apex_detector::ApexDetector apex_detector;
	kalmanFilter kalman_filter;

	float yaw;
	float pitch;
	int dist;

	std::vector<apex_detector::ArmorObject> objects;

    // 初始化网络模型
    const string network_path = "/home/wujyqaq/Desktop/auto_aim/model/opt-0517-001.xml";
    apex_detector.initModel(network_path);

    fps::FPS      global_fps_;
    while (true)  
    {
		//std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
        global_fps_.getTick();
		if (mv_capture_->isCameraOnline()) 
		{
      	src_img_ = mv_capture_->image();
		}
        switch(detect_mode){
			case 0:
			if(apex_detector.detect(src_img_, objects) && (apex_detector.state=LOST)){
				
				for (auto armor_object : objects){
					apex_detector.display(src_img_, armor_object); // 识别结果可视化
					poseSolver.getImgpPoints(armor_object.pts);
			        poseSolver.solvePose(apex_detector.getArmorType());
					
					kalman_filter.initState(poseSolver.getCameraPose());
					apex_detector.state=SHOOT;
				}
			}

			if(apex_detector.detect(src_img_, objects) && (apex_detector.state=SHOOT)){
				
				for (auto armor_object : objects){
					apex_detector.display(src_img_, armor_object); // 识别结果可视化
					poseSolver.getImgpPoints(armor_object.pts);
			        poseSolver.solvePose(apex_detector.getArmorType());
					
					cv::Point3f predict_coord = kalman_filter.predict(poseSolver.getCameraPose(),global_fps_.lastTime());
					poseSolver.show_predict(src_img_, poseSolver.camera_to_pixel(predict_coord));

				}
			}
			if(apex_detector.detect(src_img_, objects) == false){
				apex_detector.state=LOST;
			}
			
			break;
        
		    case 1:
			armor_detector.runArmorDetector(src_img_, BLUE);
			poseSolver.getImgpPoints(armor_detector.returnFinalArmorRect());
			poseSolver.solvePose(armor_detector.getArmorType());
			break;
		}


		
		// std::cout<<endl<<"yaw角"<<poseSolver.getYawAngle()<<endl;

		// std::cout<<endl<<"pitch角"<<poseSolver.getPitchAngle()<<endl;

		// std::cout<<endl<<"距离"<<poseSolver.getDistance()<<endl;

	
		// serial.sendData(apex_detector.isFindTarget(),poseSolver.getYawAngle(),poseSolver.getPitchAngle(),poseSolver.getDistance());
		
		
		
		imshow("output", src_img_);
          
        cv::waitKey(1);    //延时30  

		mv_capture_->releaseBuff();

        global_fps_.calculateFPSGlobal();
    }  
	
	return 0;
}

// #include <opencv2/core/core.hpp>
// #include <opencv2/opencv.hpp>
 
// #include <iostream>
 
// using namespace cv;
// using namespace std;
 
// const int winWidth = 800;
// const int winHeight = 600;
 
// Point mousePosition = Point(winWidth>>1, winHeight>>1);
 
// //mouse call back
// void mouseEvent(int event, int x, int y, int flags, void *param)
// {
// 	if(event==EVENT_MOUSEMOVE)
// 	{
// 		mousePosition=Point(x,y);
// 	}
// }
 
// int main()
// {
// 	//1.kalman filter setup   
// 	const int stateNum=4;  
// 	const int measureNum=2;  
 
// 	KalmanFilter KF(stateNum, measureNum, 0);
// 	Mat state (stateNum, 1, CV_32FC1); //state(x,y,detaX,detaY)
// 	Mat processNoise(stateNum, 1, CV_32F);
// 	Mat measurement = Mat::zeros(measureNum, 1, CV_32F);	//measurement(x,y)
 
 
// 		randn( state, Scalar::all(0), Scalar::all(0.1) ); //随机生成一个矩阵，期望是0，标准差为0.1;
// 		KF.transitionMatrix = (Mat_<float>(4, 4) << 
// 			1,0,1,0, 
// 			0,1,0,1, 
// 			0,0,1,0, 
// 			0,0,0,1 );//元素导入矩阵，按行;
 
// 		//setIdentity: 缩放的单位对角矩阵;
// 		//!< measurement matrix (H) 观测模型
// 		setIdentity(KF.measurementMatrix);
 
// 		//!< process noise covariance matrix (Q)
// 		// wk 是过程噪声，并假定其符合均值为零，协方差矩阵为Qk(Q)的多元正态分布;
// 		setIdentity(KF.processNoiseCov, Scalar::all(1e-5));
		
// 		//!< measurement noise covariance matrix (R)
// 		//vk 是观测噪声，其均值为零，协方差矩阵为Rk,且服从正态分布;
// 		setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
		
// 		//!< priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)*/  A代表F: transitionMatrix
// 		//预测估计协方差矩阵;
// 		setIdentity(KF.errorCovPost, Scalar::all(1));
		
 
// 		//!< corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
// 		//initialize post state of kalman filter at random 
// 		randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));
// 		Mat showImg(winWidth, winHeight,CV_8UC3);
 
// 		for(;;)
// 		{
// 			setMouseCallback("Kalman", mouseEvent);
// 			showImg.setTo(0);
 
// 			Point statePt = Point( (int)KF.statePost.at<float>(0), (int)KF.statePost.at<float>(1));
 
// 			//2.kalman prediction   
// 			Mat prediction = KF.predict();
// 			Point predictPt = Point( (int)prediction.at<float>(0), (int)prediction.at<float>(1));
 
// 			//3.update measurement
// 			measurement.at<float>(0)= (float)mousePosition.x;
// 			measurement.at<float>(1) = (float)mousePosition.y;
 
// 			//4.update
// 			KF.correct(measurement);
 
// 			//randn( processNoise, Scalar(0), Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
//             //state = KF.transitionMatrix*state + processNoise;
// 			//draw
// 			circle(showImg, predictPt, 5, CV_RGB(255,0,0),1);//former point
// 			//circle(showImg, predictPt, 5, CV_RGB(0,255,0),1);//predict point
// 			circle(showImg, mousePosition, 5, CV_RGB(0,0,255),1);//ture point
			
 
// // 			CvFont font;//字体
// // 			cvInitFont(&font, CV_FONT_HERSHEY_SCRIPT_COMPLEX, 0.5f, 0.5f, 0, 1, 8);
// 			putText(showImg, "Red: Former Point", cv::Point(10,30), FONT_HERSHEY_SIMPLEX, 1 ,Scalar :: all(255));
// 			putText(showImg, "Green: Predict Point", cv::Point(10,60), FONT_HERSHEY_SIMPLEX, 1 ,Scalar :: all(255));
// 			putText(showImg, "Blue: Ture Point", cv::Point(10,90), FONT_HERSHEY_SIMPLEX, 1 ,Scalar :: all(255));
 
// 			imshow( "Kalman", showImg );
// 			int key = waitKey(3);
// 			if (key == 27)
// 			{
// 				break;
// 			}
// 		}
// }

