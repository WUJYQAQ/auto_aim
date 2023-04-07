#include"../ArmorDetector/ArmorDetector.hpp"
#include"../PoseSolver/PoseSolver.hpp"
//#include"../Camera/mv_video_capture.hpp"
#include"../MVCamera/MVCamera.hpp"
#include"Serial/Serial.hpp"
#include"utils/fps.hpp"
#include"../KalmanPredictor/KalmanPredictor.h"
using namespace std;
using namespace cv;

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
    
	Serial serial =Serial("/home/wujyqaq/桌面/ ArmorDetector/Configs/serial/serial.xml");
	PoseSolver poseSolver=PoseSolver("Configs/pose_solver/camera_params.xml",1);

	poseSolver.setObjPoints(smallArmor,135,55);
	poseSolver.setObjPoints(bigArmor,230,127);
	mindvision::MVCamera* mv_capture_ = new mindvision::MVCamera(mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_1024, mindvision::EXPOSURE_2500));
	cv::Mat src_img_;
	ArmorDetector armor_detector;
	

    fps::FPS       global_fps_;
    while (true)  
    {
        global_fps_.getTick();
		if (mv_capture_->isCameraOnline()) 
		{
      	src_img_ = mv_capture_->image();
		}

		armor_detector.runArmorDetector(src_img_, RED);
		
		poseSolver.getImgpPoints(armor_detector.returnFinalArmorRect());
		
		poseSolver.solvePose(armor_detector.getArmorType());
		
		
		
		std::cout<<endl<<"yaw角"<<poseSolver.getYawAngle()<<endl;

		std::cout<<endl<<"pitch角"<<poseSolver.getPitchAngle()<<endl;

		std::cout<<endl<<"距离"<<poseSolver.getDistance()<<endl;

	
		//serial.sendData(armor_detector.findArmor(),poseSolver.getYawAngle(),poseSolver.getPitchAngle(),poseSolver.getDistance());
		

		
		imshow("output", src_img_);
          
        cv::waitKey(1);    //延时30  

		mv_capture_->releaseBuff();

        //global_fps_.calculateFPSGlobal();
    }  
	
	return 0;
}
