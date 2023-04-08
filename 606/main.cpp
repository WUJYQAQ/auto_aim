#include"../Detector/ArmorDetector/ArmorDetector.hpp"
#include"../Detector/ApexDetector/ApexDetector.hpp"
#include"../PoseSolver/PoseSolver.hpp"
//#include"../Camera/mv_video_capture.hpp"
#include"../MVCamera/MVCamera.hpp"
#include"Serial/Serial.hpp"
#include"utils/fps.hpp"
#include"../KalmanPredictor/KalmanPredictor.h"
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
    
	Serial serial =Serial("/home/matrix/Desktop/auto_aim/Configs/serial/serial.xml");
	PoseSolver poseSolver=PoseSolver("/home/matrix/Desktop/auto_aim/Configs/pose_solver/camera_params.xml",1);

	poseSolver.setObjPoints(smallArmor,135,55);
	poseSolver.setObjPoints(bigArmor,230,55);
	mindvision::MVCamera* mv_capture_ = new mindvision::MVCamera(mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_1024, mindvision::EXPOSURE_2500));
	
	cv::Mat src_img_;
	armor_detector::ArmorDetector armor_detector;
	apex_detector::ApexDetector apex_detector;


	std::vector<apex_detector::ArmorObject> objects;

    // 初始化网络模型
    const string network_path = "/home/matrix/Desktop/auto_aim/model/opt-0517-001.xml";
    apex_detector.initModel(network_path);

    fps::FPS      global_fps_;
    while (true)  
    {
        //global_fps_.getTick();
		if (mv_capture_->isCameraOnline()) 
		{
      	src_img_ = mv_capture_->image();
		}

        switch(detect_mode){
			case 0:
			if(apex_detector.detect(src_img_, objects)){
				for (auto armor_object : objects){
					apex_detector.display(src_img_, armor_object); // 识别结果可视化
					poseSolver.getImgpPoints(armor_object.pts);
			        poseSolver.solvePose(apex_detector.getArmorType());
				}
			}
			break;
        
		    case 1:
			armor_detector.runArmorDetector(src_img_, RED);
			poseSolver.getImgpPoints(armor_detector.returnFinalArmorRect());
			poseSolver.solvePose(armor_detector.getArmorType());
			break;
		}


		
		std::cout<<endl<<"yaw角"<<poseSolver.getYawAngle()<<endl;

		std::cout<<endl<<"pitch角"<<poseSolver.getPitchAngle()<<endl;

		std::cout<<endl<<"距离"<<poseSolver.getDistance()<<endl;

	
		serial.sendData(0,poseSolver.getYawAngle(),poseSolver.getPitchAngle(),poseSolver.getDistance());
		

		
		imshow("output", src_img_);
          
        cv::waitKey(1);    //延时30  

		mv_capture_->releaseBuff();

        //global_fps_.calculateFPSGlobal();
    }  
	
	return 0;
}


