#include "../Detector/OpenVINO2022/openvino_detector.hpp"
#include "../PoseSolver/PoseSolver.hpp"
#include "../MVCamera/MVCamera.hpp"
#include "Serial/Serial.hpp"
#include "utils/fps.hpp"
#include "../KalmanFilter/KalmanFilter.h"
using namespace cv;
using namespace rm_auto_aim;
int main()
{
	int detect_mode = 0;
    
	// 初始化网络模型
    const string network_path = "/home/hezhexi2002/WorkSpace/auto_aim/model/opt-1208-001.onnx";
	Serial serial =Serial("/home/hezhexi2002/WorkSpace/auto_aim/Configs/serial/serial.xml");
	PoseSolver poseSolver=PoseSolver("/home/hezhexi2002/WorkSpace/auto_aim/Configs/pose_solver/camera_params.xml",1);

	poseSolver.setObjPoints(smallArmor,135,55);
	poseSolver.setObjPoints(bigArmor,230,55);
	mindvision::MVCamera* mv_capture_ = new mindvision::MVCamera(mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_1024, mindvision::EXPOSURE_10000));
	
	cv::Mat src_img_, infer_img;
	cv::Point3f last_coord;
	rm_auto_aim::OpenVINODetector openvino_detector(network_path, "AUTO");
	kalmanFilter kalman_filter;

	// std::vector<apex_detector::ArmorObject> objects;
	std::vector<rm_auto_aim::ArmorObject> objects;
    
 
    openvino_detector.init();
    
	openvino_detector.set_callback(
    [& objects](const std::vector<rm_auto_aim::ArmorObject> & objs, int64_t timestamp,
    const cv::Mat & src_img) {
    objects.assign(objs.begin(), objs.end());
    });


    ArmorState state = ArmorState::LOST;

    
    fps::FPS      global_fps_;
    while (true)  
    {
		
	global_fps_.getTick();
	if (mv_capture_->isCameraOnline()) 
	{
	src_img_ = mv_capture_->image();
	
	}
		
	cv::cvtColor(src_img_, infer_img, cv::COLOR_BGR2RGB);

	auto res = openvino_detector.push_input(infer_img, 0);
	if (res.get()) {
		if (state == LOST) {
			for (auto armor_object : objects) {
				poseSolver.getImgpPoints(armor_object.pts);
				poseSolver.solvePose(openvino_detector.getArmorType(armor_object));
				kalman_filter.initState(poseSolver.getCameraPose());
				state = SHOOT;
			}
		} else if (state == SHOOT) {
			for (auto armor_object : objects) {
				openvino_detector.display(src_img_, armor_object); // Visualize detection results
				poseSolver.getImgpPoints(armor_object.pts);
				poseSolver.solvePose(openvino_detector.getArmorType(armor_object));
				kalman_filter.initState(last_coord, true);
				kalman_filter.predictAsync(last_coord, kalman_filter, [&](cv::Point3f predict_coord) {
        			poseSolver.show_predict(src_img_, poseSolver.camera_to_pixel(predict_coord));
        			last_coord = predict_coord;
    			});
			}
		}
	} else {
		state = LOST;
	}


			
	
		
		
		imshow("output", src_img_);
          
        cv::waitKey(1);    //延时30  

		mv_capture_->releaseBuff();

        global_fps_.calculateFPSGlobal();
    }  
	
	return 0;
}

