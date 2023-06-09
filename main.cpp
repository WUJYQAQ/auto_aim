#include"../Detector/ArmorDetector/ArmorDetector.hpp"
#include"../Detector/ApexDetector/ApexDetector.hpp"
#include"../PoseSolver/PoseSolver.hpp"
//#include"../Camera/mv_video_capture.hpp"
#include"../MVCamera/MVCamera.hpp"
#include"Serial/Serial.hpp"
#include"utils/fps.hpp"
#include"../Predictor/PredictorKalman.hpp"
using namespace std;
using namespace cv;
using namespace apex_detector;
using namespace armor_detector;

int main()
{
	
	int detect_mode = 0;
    
	Serial serial =Serial("/home/wujyqaq/Desktop/auto_aim/Configs/serial/serial.xml");
	PoseSolver poseSolver=PoseSolver("/home/wujyqaq/Desktop/auto_aim/Configs/pose_solver/camera_params.xml",1);
	mindvision::MVCamera* mv_capture_ = new mindvision::MVCamera(mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_1024, mindvision::EXPOSURE_4000));

	cv::Mat src_img_;
	armor_detector::ArmorDetector armor_detector;
	apex_detector::ApexDetector apex_detector;

	std::vector<cv::Point2f> image_points;
	
	std::vector<apex_detector::ArmorObject> objects;

	ArmorObject optimal_target;
	//ReceiveData receive_data;
    // 初始化网络模型
    const string network_path = "/home/wujyqaq/Desktop/auto_aim/model/opt-0517-001.xml";
    apex_detector.initModel(network_path);

    fps::FPS      global_fps_;
    while (true)  
    {
		//serial.getReceiveData(receive_data);
        global_fps_.getTick();
		if (mv_capture_->isCameraOnline()) 
		{
      	src_img_ = mv_capture_->image();
		}

        switch(detect_mode)
		{
			case 0:
			if(apex_detector.detect(src_img_, objects))
			{
				
				apex_detector.getOptimalTarget(objects,optimal_target,apex_detector::BLUE);
						
				if(!optimal_target.pts.empty())
				{	
					apex_detector.display(src_img_, optimal_target); // 识别结果可视化
					poseSolver.getImgpPoints(optimal_target.pts);	       		
					
					serial.sendData(1,poseSolver.solvePose(apex_detector.getArmorType(),16));
				}
					
				
			}
			break;
        
		    case 1:
			if(armor_detector.runArmorDetector(src_img_, armor_detector::BLUE))
			{
				std::cout << "==================DEBUG=======================" << std::endl;
				if(armor_detector.getOptimalTarget(image_points))
				{
					poseSolver.getImgpPoints(image_points);
					std::cout << "==================DEBUG=======================" << std::endl;
					serial.sendData(1,poseSolver.solvePose(armor_detector.getArmorType(),16));
				}
				

			}
			
			break;
		}


	
		//serial.sendData(0,poseSolver.getYawAngle(),poseSolver.getPitchAngle(),poseSolver.getDistance());
		

		
		imshow("output", src_img_);
          
        cv::waitKey(1);    //延时30  

		mv_capture_->releaseBuff();

        global_fps_.calculateFPSGlobal();
    }  
	
	return 0;
}