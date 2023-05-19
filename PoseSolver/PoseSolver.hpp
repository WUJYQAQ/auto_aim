#ifndef POSE_HPP
#define POSE_HPP
#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>
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

	void solvePose(int armorType);

	float getYawAngle();

	float getPitchAngle();

	int getDistance();

	std::tuple<double,double,double> getPose();

	void runPoseSolver();

	cv::Point3f getCameraPose();

    void show_predict(cv::Mat image2show, cv::Mat predict_coord);

	cv::Mat camera_to_pixel(cv::Point3f camera_coord);
	
private:
	cv::Point3f camera_coord;

	PnP_Results pnp_results;

	cv::Mat instantMatrix;	//Camera Matrix
	cv::Mat distortionCoeffs;	//Distortion Coeffs of Camera

	std::vector<cv::Point3f> bigObjPoints;
	std::vector<cv::Point3f> smallObjPoints;

	std::vector<cv::Point2f> imagePoints;

	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);

	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
	
	double pitch, yaw, distance;
	
	ArmorType armorType;


};


#endif 