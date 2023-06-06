#include "PoseSolver.hpp"

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
	setObjPoints(smallArmor,135,55);
	setObjPoints(bigArmor,230,55);
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

Eigen::Vector3f PoseSolver::solvePose(int armorType , float bs)
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

	float x_pos= static_cast<float>(tvec.at<double>(0, 0)) + camera2gun_offest.x;
	float y_pos = static_cast<float>(tvec.at<double>(1, 0)) + camera2gun_offest.y;
	float z_pos = static_cast<float>(tvec.at<double>(2, 0)) + camera2gun_offest.z;
	
	cv::Point3f camera_coord=cv::Point3f(x_posm,y_pos,z_pos);
	setBulletSpeed(bs , camera_coord);


	float tan_pitch = y_pos / sqrt(x_pos * x_pos + z_pos * z_pos);
	float tan_yaw = x_pos / z_pos;
	float distance=sqrt(x_pos * x_pos + y_pos * y_pos + z_pos * z_pos);
	
	float pitch_angle_coeff = -atan(tan_pitch) * 180 / CV_PI-compensate(camera_coord);
	
	Eigen::Vector3f result(pitch_angle_coeff , atan(tan_yaw) * 180 / CV_PI , distance);
	
	return result;
}


Eigen::Vector2f PoseSolver::solvePose(int armorType,Eigen::Quaternionf q)
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

	cv::Point3f earth_coord(camera2earth(q,tvec));
	//std::cout<<"==============earth_coord================="<<endl<<earth_coord<<endl;

	cv::Point3f camera_coord(earth2camera(q,earth_coord));

	std::cout<<"==============camera_coord================="<<endl<<camera_coord<<endl;
	
	float tan_pitch=earth_coord.z/sqrt(earth_coord.x*earth_coord.x + earth_coord.z*earth_coord.z);
	
	float tan_yaw=earth_coord.x/earth_coord.z;

	Eigen::Vector2f result(-atan(tan_pitch) * 180 / CV_PI,atan(tan_yaw) * 180 / CV_PI);

	return result;

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

cv::Point3f PoseSolver::camera2earth(Eigen::Quaternionf q, cv::Point3f camera_coord)
{
	camera_coord += camera2imu_offest;
    Eigen::Quaternionf p(0, camera_coord.z, -camera_coord.x, -camera_coord.y);

    Eigen::Quaternionf result = q * p *q.inverse();
    return cv::Point3f(result.x(), result.y(), result.z());
}

cv::Point3f PoseSolver::earth2camera(Eigen::Quaternionf q, cv::Point3f earth_coord)
{
    Eigen::Quaternionf p(0, earth_coord.x, earth_coord.y, earth_coord.z);

    Eigen::Quaternionf result = q.inverse() * p * q;
	
	cv::Point3f sss(-result.y(),-result.z(),result.x());
	std::cout<<"-------------------------------"<<sss<<endl;
    
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

}

cv::Point3f PoseSolver::camera2earth(Eigen::Quaternionf q, cv::Mat tvec)
{	
	std::cout<<"==============tvec==============="<<endl<<tvec<<endl;
	cv::Point3f camera_coord(static_cast<float>(tvec.at<double>(0,0)),
							 static_cast<float>(tvec.at<double>(1,0)),
							 static_cast<float>(tvec.at<double>(2,0)));
	
	camera_coord += camera2imu_offest;
	
	//
	cv::Point3f imu_coord(camera_coord.x, camera_coord.z, -camera_coord.y);
	
	std::cout<<"==============imu_coord_mat==============="<<endl<<imu_coord<<endl;
    
	Eigen::Quaternionf p(0, camera_coord.z, -camera_coord.x, -camera_coord.y);

    Eigen::Quaternionf result = q * p *q.inverse();

	cv::Point3f xxx(result.x(),result.y(),result.z());
	std::cout<<"==============earth_coord==============="<<endl<<xxx<<endl;

    return cv::Point3f(result.x(), result.y(), result.z());
}

float PoseSolver::getFlyTime(float angle , cv::Point3f camera_coord)
{
	float x = sqrt(camera_coord.x * camera_coord.x + camera_coord.z * camera_coord.z) / 1000.0;

    return (exp(k * x) - 1.0) / (k * bs_coeff * bullet_speed * cos(angle)); //- dt - delay_time;
}

float PoseSolver::compensate(cv::Point3f camera_coord)
{
	 
    float dy, angle, y_actual;
    float t_actual = 0.0;
    float y_temp = -camera_coord.y / 1000.0;
    float y = y_temp;
    float x = sqrt(camera_coord.x * camera_coord.x + camera_coord.z * camera_coord.z) / 1000.0;

    for (int i = 0; i < 40; i++)
    {
    	angle = atan2(y_temp, x);
   		t_actual = getflytime(angle, camera_coord);
    	y_actual = float(bs_coeff * bullet_speed * sin(angle) * t_actual - g * t_actual * t_actual / 2.0);
    	dy = y - y_actual;
    	y_temp += dy;
        if (abs(dy) < 0.001)
        	break;
    }
    return cv::Point2d((angle) / M_PI * 180.0);
}

void PoseSolver::setBulletSpeed(float bs,cv::Point3f coord)
{
	bullet_speed = bs;
    if (bullet_speed <= 16)
    {
        bs_coeff = 0.85;
    }
    else if (bullet_speed > 17 && bullet_speed < 20)
    {
        bs_coeff = 0.95;
        if (-coord.y > 300)
        bs_coeff *= 0.98;
    }
    else if (bullet_speed > 28 && bullet_speed < 32)
    {
        bs_coeff = 1.00;
        if (-coord.y > 250)
            bs_coeff *= 0.90;
        if (-coord.y > 1000)
            bs_coeff *= 0.88;
    }
        else
            bs_coeff = 0.9;
}
