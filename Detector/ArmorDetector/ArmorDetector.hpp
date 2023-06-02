#ifndef ARMOR_DETECTOR_HPP
#define ARMOR_DETECTOR_HPP
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include<iostream>

namespace armor_detector
{

//
enum EnemyColor
{
	ALL = 0,
	BLUE = 1,
	RED = 2
};

struct ImageConfigs
{

};

//
struct LightConfigs
{
	//
	int perimeter_max = 1000;
	int perimeter_min = 16;
	//
	float maxLightArea = 0;
	float minLightArea = 0;
	//
	int light_draw = 0;
	int light_edit = 0;

	///width
	float maxAspectRatio = 30;
	float minAspectRatio = 1;

    float maxAngle = 45;
	float minAngle= 45;
};

//
struct ArmorConfigs
{
	//
	int armor_draw = 1;
	int armor_edit = 0;
	// 
	int light_height_ratio_min = 5;
	int light_height_ratio_max = 20;
	// 
	int light_width_ratio_min = 5;
	int light_width_ratio_max = 30;
	// 
	int light_y_different = 10;
	// 
	int light_height_different = 10;
	// 
	int armor_angle_different = 80;
	// 
	int small_armor_aspect_min = 11;
	int armor_type_th = 33;
	int big_armor_aspect_max = 42;
};


//struct LightData
//{
//	float width=0;
//	float height=0;
//	float area=0;
//
//	
//	cv::RotatedRect lightRect;
//};

struct ArmorData
{
	float width = 0.f;
	float height = 0.f;
	float area = 0.f;
	float tan_angle = 0.f;
	float aspectRatio = 0.f;
	
	float center_distance = 0.f;
	
	float left_light_width = 0.f;
	float right_light_width = 0.f;
	float left_light_height = 0.f;
	float right_light_height = 0.f;
	float light_height_aspect = 0.f;
	float light_width_aspect = 0.f;
	
	cv::Point2f armor_center;
	cv::Point2f armor_lt;
	cv::Point2f armor_rb;
	cv::Point2f p0;
	cv::Point2f p1;
	cv::Point2f p2;
	cv::Point2f p3;
	cv::Point2f armor_points[4];

	float delta_center;

	int distinguish = 0;      //  0  1 
	//int distance_center = 0;      // 
	cv::RotatedRect armorRect;
	cv::RotatedRect leftLight;
	cv::RotatedRect rightLight;
};

class ArmorDetector
{
public:
	
	/**
	* @brief 
	* @param 
	*/
	ArmorDetector();
	
	/**
	* @brief 
	* @param
	*/
	~ArmorDetector();
	
	/**
	* @brief 
	*/
	void release();
	
	/**
	* @brief 识别总入口
	* @param  原图像 敌方颜色
	*/
	bool runArmorDetector(const cv::Mat& srcImg, const EnemyColor enemy_color);
	
	/**
	* @brief	图像预处理
	* @param	原图像 敌方颜色
	* @return	Mat
	*/
	cv::Mat imgPretreat(const cv::Mat & srcImg,const EnemyColor enemy_color);
	
	/**
	* @brief 	
	* @param 
	* @return
	*/
	bool isFindLights();

	/**
	* @brief 
	* @param
	* @return
	*/
	void findArmor();

	/**
	* @brief 
	* @param 
	* @return
	*/
	bool matchLights(const int i, const int j);
	
	/**
	* @brief 
	* @param 
	* @return
	*/
	void chooseTarget();

	/**
	* @brief 
	* @param 
	* @return
	*/
	float getDistance(const cv::Point a, const cv::Point b);

	void getAnglediff();

	int getArmorType();

	bool getOptimalTarget(std::vector<cv::Point2f> &image_points);

	std::vector<cv::Point2f> returnFinalArmorRect();
private:
	cv::Mat image;
	cv::Mat _grayImg;
	cv::Mat split_image;
	cv::Mat gray_image;
	cv::Mat _binImg;

	cv::Point2f last_armor_center;

	LightConfigs light_config;
	ArmorConfigs armor_config;

	std::vector<cv::Mat> channels;
	std::vector<cv::Point2f> image_points;

	ArmorData armor_data;
	std::vector<ArmorData> armors;
	
	std::vector<cv::RotatedRect> light;
	std::vector<cv::RotatedRect> armors_;
	std::vector<ArmorData> armor;


	
};

}
#endif