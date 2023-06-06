#ifndef POSE_HPP
#define POSE_HPP
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include<iostream>


enum ArmorType
{
	smallArmor = 0,
	bigArmor = 1
};

struct PnP_Results
{
	float yaw_angle;
	float pitch_angle;
	int distance;
	PnP_Results() 
	{
    yaw_angle   = 0.f;
    pitch_angle = 0.f;
    distance       = 0;
  	}
};

struct PnPConfig
{
	int smallArmorHeight = 60;
	int smallArmorWidth = 140;

	int bigArmorHeight = 60;
	int bigArmorWidth = 245;
};



class PoseSolver
{
public:

	PoseSolver()=default;
	explicit PoseSolver(const char* filePath, int camId);

	~PoseSolver(); 


	void setCameraParams(const cv::Mat& camMatrix, const cv::Mat& distCoeffs);

	int readFile(const char* filePath, int camId);

	void setObjPoints(ArmorType type, double width, double height);

	void getImgpPoints(std::vector<cv::Point2f> image_points);

	Eigen::Vector3f solvePose(int armorType , float bs);

	Eigen::Vector2f solvePose(int armorType,Eigen::Quaternionf q);

	void runPoseSolver();

	cv::Point3f getCameraPose();

    void showPredict(cv::Mat image2show, cv::Mat predict_coord);

	cv::Mat camera2pixel(cv::Point3f camera_coord);

	cv::Point3f camera2earth(Eigen::Quaternionf q, cv::Point3f camera_coord);
	
	cv::Point3f earth2camera(Eigen::Quaternionf q, cv::Point3f earth_coord);

	cv::Point3f camera2earth(Eigen::Quaternionf q, cv::Mat tvec);

	cv::Mat earth2pixel(Eigen::Quaternionf q1, cv::Point3f earth_coord);

	float getFlyTime(float angle , cv::Point3f camera_coord);

	float compensate(cv::Point3f camera_coord);

	void setBulletSpeed(float bs , cv::Point3f coord);

private:

	cv::Point3f camera2gun_offest=cv::Point3f(0.f,0.f,0.f);
	cv::Point3f camera2imu_offest=cv::Point3f(11.55,122.6,70.1);
	
	cv::Point3f camera_coord;

	cv::Mat instantMatrix;	//Camera Matrix
	cv::Mat distortionCoeffs;	//Distortion Coeffs of Camera

	std::vector<cv::Point3f> bigObjPoints;
	std::vector<cv::Point3f> smallObjPoints;

	std::vector<cv::Point2f> imagePoints;

	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);

	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
	
	ArmorType armorType;
	float bs_coeff = 0.90;
	float bullet_speed = 16.0;
	float k = 0.0402;	//阻力係數
	float g = 0.975;
};


#endif 