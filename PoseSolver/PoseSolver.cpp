﻿#include "PoseSolver.hpp"

using namespace cv;
using namespace std;

PoseSolver::PoseSolver(const char* filePath, int camId) 
{
	FileStorage fsRead;
	fsRead.open(filePath, FileStorage::READ);
	if (!fsRead.isOpened())
	{
		cout << "Failed to open xml" << endl;
	}

	//fsRead["Y_DISTANCE_BETWEEN_GUN_AND_CAM"] >> GUN_CAM_DISTANCE_Y;

	Mat cameraMatrix;
	Mat distortionCoeffs;
	switch (camId)
	{
	case 1:
		fsRead["CAMERA_MATRIX_1"] >> cameraMatrix;
		fsRead["DISTORTION_COEFF_1"] >> distortionCoeffs;
		break;
	default:
		cout << "WRONG CAMID GIVEN!" << endl;
		break;
	}
	setCameraParams(cameraMatrix, distortionCoeffs);
	fsRead.release();
}

PoseSolver::~PoseSolver(void)
{

}

void PoseSolver::setCameraParams(const Mat& camMatrix, const Mat& distCoeffs)
{
	instantMatrix = camMatrix;
	distortionCoeffs = distCoeffs;
}

int PoseSolver::readFile(const char* filePath, int camId)
{
	FileStorage fsRead;
	fsRead.open(filePath, FileStorage::READ);
	if (!fsRead.isOpened())
	{
		cout << "Failed to open xml" << endl;
		return -1;
	}

	//fsRead["Y_DISTANCE_BETWEEN_GUN_AND_CAM"] >> GUN_CAM_DISTANCE_Y;

	Mat cameraMatrix;
	Mat distortionCoeffs;
	switch (camId)
	{
	case 1:
		fsRead["CAMERA_MATRIX_1"] >> cameraMatrix;
		fsRead["DISTORTION_COEFF_1"] >> distortionCoeffs;
		break;
	default:
		cout << "WRONG CAMID GIVEN!" << endl;
		break;
	}
	setCameraParams(cameraMatrix, distortionCoeffs);
	fsRead.release();
	return 0;
}

void PoseSolver::setObjPoints(ArmorType type, double width, double height)
{
	double centerX = width / 2.0;
	double centerY = height / 2.0;
	switch (type)
	{
	case smallArmor:
	    smallObjPoints.push_back(Point3f(-centerX, centerY, 0));   //tl top left左上
		smallObjPoints.push_back(Point3f(-centerX, -centerY, 0));  //bl below left左下
		smallObjPoints.push_back(Point3f(centerX, -centerY, 0));   //br below right右下
		smallObjPoints.push_back(Point3f(centerX, centerY, 0));	//tr top right右上
		break;

	case bigArmor:
	    bigObjPoints.push_back(Point3f(-centerX, centerY, 0));   //tl top left左上
		bigObjPoints.push_back(Point3f(-centerX, -centerY, 0));  //br below right左下
		bigObjPoints.push_back(Point3f(centerX, -centerY, 0));   //bl below left右下
		bigObjPoints.push_back(Point3f(centerX, centerY, 0));    //tr top right右上
		break;
	default: break;
	}
}

void PoseSolver::getImgpPoints(std::vector<Point2f> image_points)
{
	imagePoints.clear();
	for(int i=0;i<4;i++)
	{
		imagePoints.emplace_back(image_points[i]);
		//cout<<endl<<imagePoints[i]<<endl;
	}



}

void PoseSolver::solvePose(int armorType)
{	
	
	switch (armorType)
	{
	case smallArmor:
		solvePnP(smallObjPoints, imagePoints, instantMatrix, distortionCoeffs, rvec, tvec, false, SOLVEPNP_ITERATIVE); 
		cout<<"=============小装甲板============"<<endl;
		break;
	case bigArmor:
		solvePnP(bigObjPoints, imagePoints, instantMatrix, distortionCoeffs, rvec, tvec, false, SOLVEPNP_ITERATIVE); 
		cout<<"=============大装甲板============"<<endl;
		break;
	default:
		break;
	}



	//rotT.at<double>(1, 0) -= GUN_CAM_DISTANCE_Y;
	double x_pos = tvec.at<double>(0, 0);
	double y_pos = tvec.at<double>(1, 0);
	double z_pos = tvec.at<double>(2, 0);
	cout<<"================平移向量================\n"<<tvec<<endl;

	double tan_pitch = y_pos / sqrt(x_pos * x_pos + z_pos * z_pos);
	double tan_yaw = x_pos / z_pos;

	pnp_results.yaw_angle = static_cast<float>(atan(tan_yaw) * 180 / CV_PI);	
	pnp_results.pitch_angle = static_cast<float>(-atan(tan_pitch) * 180 / CV_PI);
	pnp_results.distance = static_cast<int>(sqrt(x_pos * x_pos + y_pos * y_pos + z_pos * z_pos));

}

float PoseSolver::getYawAngle()
{
	return pnp_results.yaw_angle;
}

float PoseSolver::getPitchAngle()
{
	return pnp_results.pitch_angle;
}

int PoseSolver::getDistance()
{
	return pnp_results.distance;
}

cv::Point3f PoseSolver::getCameraPose()
{
	camera_coord = cv::Point3f(tvec.at<double>(0, 0),tvec.at<double>(1, 0),tvec.at<double>(2, 0));
	return camera_coord;
}

void PoseSolver::showPredict(cv::Mat image2show, cv::Mat predict_coord)
{

    cv::Point2f pixel_coord;

	pixel_coord=cv::Point2f(predict_coord.at<double>(0,0),predict_coord.at<double>(1,0));
	
	cout<<"=================pixel_coord=================="<<endl<<pixel_coord<<endl;

	cv::circle(image2show, pixel_coord, 10, cv::Scalar(0, 255, 255), 5);


 }



cv::Mat PoseSolver::camera2pixel(cv::Point3f camera_coord)
{
	cv::Mat camera_coord_mat;
	
	camera_coord_mat = (cv::Mat_<double>(3, 1) << camera_coord.x,
                                                  camera_coord.y,
                                                  camera_coord.z);													 

	return instantMatrix * camera_coord_mat / camera_coord.z;

}

<<<<<<< HEAD
cv::Point3f PoseSolver::camera2earth(Eigen::Quaternionf q1, cv::Point3f camera_coord)
{
	camera_coord += camera2imu_offest;
    Eigen::Quaternionf p(0, camera_coord.z, -camera_coord.x, -camera_coord.y);
=======
/**      Z
 *       |   X
 *       |  /   
 *       | /
 * Y_____|/
 *      
 * IMU 的世界坐标系
 */
cv::Point3f PoseSolver::camera2world(Eigen::Quaternionf q1, cv::Point3f point, cv::Point3f trans_offset)
{
    point += trans_offset;
    Eigen::Quaternionf p(0, point.z, -point.x, -point.y);
>>>>>>> 9e882074549752b26dcbf28df87804cf7b599ca6

    Eigen::Quaternionf result = q1 * p *q1.inverse();
    return cv::Point3f(result.x(), result.y(), result.z());
}

<<<<<<< HEAD
cv::Point3f PoseSolver::earth2camera(Eigen::Quaternionf q1, cv::Point3f earth_coord)
{
    Eigen::Quaternionf p(0, earth_coord.x, earth_coord.y, earth_coord.z);

    Eigen::Quaternionf result = q1.inverse() * p * q1;
    return cv::Point3f(-result.y(), -result.z(), result.x()) - camera2imu_offest;
}

cv::Mat PoseSolver::earth2pixel(Eigen::Quaternionf q1, cv::Point3f earth_coord)
{
	cv::Mat camera_coord_mat;
	
	Eigen::Quaternionf p(0, earth_coord.x, earth_coord.y, earth_coord.z);

    Eigen::Quaternionf camera_coord_temp = q1.inverse() * p * q1;

	camera_coord_mat = (cv::Mat_<double>(3, 1) << camera_coord_temp.x(),
                                                  camera_coord_temp.y(),
                                                  camera_coord_temp.z());

	return instantMatrix * camera_coord_mat / camera_coord_temp.z();

=======
cv::Point3f PoseSolver::world2camera(Eigen::Quaternionf q1, cv::Point3f point, cv::Point3f trans_offset)
{
    Eigen::Quaternionf p(0, point.x, point.y, point.z);

    Eigen::Quaternionf result = q1.inverse() * p * q1;
    return cv::Point3f(-result.y(), -result.z(), result.x()) - trans_offset;
>>>>>>> 9e882074549752b26dcbf28df87804cf7b599ca6
}



