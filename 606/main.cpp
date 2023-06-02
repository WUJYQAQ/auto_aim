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
	
	int detect_mode = 1;
    
	Serial serial =Serial("/home/wujyqaq/Desktop/auto_aim/Configs/serial/serial.xml");
	PoseSolver poseSolver=PoseSolver("/home/wujyqaq/Desktop/auto_aim/Configs/pose_solver/camera_params.xml",1);

	poseSolver.setObjPoints(smallArmor,135,55);
	poseSolver.setObjPoints(bigArmor,230,55);
	mindvision::MVCamera* mv_capture_ = new mindvision::MVCamera(mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_1024, mindvision::EXPOSURE_5000));
	
	cv::Mat src_img_;
	armor_detector::ArmorDetector armor_detector;
	apex_detector::ApexDetector apex_detector;

	std::vector<cv::Point2f> image_points;
	std::vector<apex_detector::ArmorObject> objects;

    // 初始化网络模型
    const string network_path = "/home/wujyqaq/Desktop/auto_aim/model/opt-0517-001.xml";
    apex_detector.initModel(network_path);

    fps::FPS      global_fps_;
    while (true)  
    {
        global_fps_.getTick();
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
			if(armor_detector.runArmorDetector(src_img_, BLUE))
			{
				std::cout << "==================DEBUG=======================" << std::endl;
				if(armor_detector.getOptimalTarget(image_points))
				{
					poseSolver.getImgpPoints(image_points);
					std::cout << "==================DEBUG=======================" << std::endl;
					poseSolver.solvePose(armor_detector.getArmorType());
				}
				

<<<<<<< HEAD
			}
			
			break;
=======
    auto res = openvino_detector.push_input(infer_img, 0);
	if (res.get()) {
		if (state == LOST) {
			openvino_detector.getOptimalTarget(objects, optimal_target, ArmorColor::BLUE);
			poseSolver.getImgpPoints(optimal_target.pts);
			poseSolver.solvePose(openvino_detector.getArmorType(optimal_target));
			kalman_filter.initState(poseSolver.getCameraPose(), optimal_target.delta_centr);
			state = SHOOT;
		} else if (state == SHOOT) {
			openvino_detector.getOptimalTarget(objects, optimal_target, ArmorColor::BLUE);
			openvino_detector.display(src_img_, optimal_target);
			poseSolver.getImgpPoints(optimal_target.pts);
			poseSolver.solvePose(openvino_detector.getArmorType(optimal_target));
			cv::Point3f predict_coord = kalman_filter.predict(poseSolver.getCameraPose(), std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch().count());
			poseSolver.show_predict(src_img_, poseSolver.camera_to_pixel(predict_coord));
			last_coord = predict_coord;
>>>>>>> 9e882074549752b26dcbf28df87804cf7b599ca6
		}


	
		serial.sendData(0,poseSolver.getYawAngle(),poseSolver.getPitchAngle(),poseSolver.getDistance());
		

		
		imshow("output", src_img_);
          
        cv::waitKey(1);    //延时30  

		mv_capture_->releaseBuff();

        global_fps_.calculateFPSGlobal();
    }  
	
	return 0;
}