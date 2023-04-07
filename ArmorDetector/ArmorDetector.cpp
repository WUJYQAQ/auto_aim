#include"ArmorDetector.hpp"

ArmorDetector::ArmorDetector()
{

}

ArmorDetector::~ArmorDetector()
{

}
void ArmorDetector::runArmorDetector(const cv::Mat& srcImg, const EnemyColor enemy_color)
{
	release();
	imgPretreat(srcImg, enemy_color);

	if (isFindLights())
	{
		std::cout << "发现灯条"<<std::endl;
		findArmor();
	}
	else
		std::cout << "没有装甲板"<<std::endl  ;
	cv::namedWindow("output", 0);
								cv::resizeWindow("output",640,480);
								cv::imshow("output", image);
}

//释放
void ArmorDetector::release()
{
	light.clear();
	armor.clear();
}

//图像预处理
cv::Mat ArmorDetector::imgPretreat(const cv::Mat & srcImg,const EnemyColor enemy_color)
{
	image = srcImg;
	cv::split(srcImg, channels);//分离色彩通道

	if (enemy_color == BLUE)
	{
		split_image= channels.at(0) - channels.at(2);
		cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
		cv::bitwise_and(split_image, gray_image, _grayImg);
	}
	else
	{
		split_image = channels.at(2) - channels.at(0);
		//cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
		//cv::bitwise_and(split_image, gray_image, _grayImg);
	}
	cv::namedWindow("灰度图", 0);
	cv::resizeWindow("灰度图",640,480);
	cv::imshow("灰度图", split_image);

	cv::threshold(split_image, _binImg, 120, 255, cv::THRESH_BINARY);

	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));

	dilate(_binImg, _binImg, element);
	cv::namedWindow("二值图", 0);
	cv::resizeWindow("二值图",640,480);
	cv::imshow("二值图", _binImg);
	//cv::waitKey(0);
	return _binImg;

}

//寻找灯条
bool ArmorDetector::isFindLights()
{
	int perimeter = 0;
	cv::RotatedRect                     lightRect;
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(_binImg,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE);
	for (int i = 0; i < contours.size(); ++i) 
	{
		perimeter = arcLength(contours[i], true);

		if (perimeter < _lightConfigs.perimeter_min ||
			perimeter > _lightConfigs.perimeter_max ||
			contours[i].size() < 5) 
		{
			continue;
		}
		lightRect = cv::fitEllipse(cv::Mat(contours[i]));
		
		
		float height = MAX(lightRect.size.width, lightRect.size.height);
		float width  = MIN(lightRect.size.width, lightRect.size.height);
		float aspectRatio = height / width;
		if (lightRect.angle > 90.0f) 
		{
			lightRect.angle = lightRect.angle - 180.0f;
		}
		if (lightRect.angle < _lightConfigs.maxAngle &&
			lightRect.angle > -_lightConfigs.minAngle &&
			aspectRatio < _lightConfigs.maxAspectRatio &&
			aspectRatio > _lightConfigs.minAspectRatio)
		{
			 
			cv::ellipse(image, lightRect, cv::Scalar(0, 255, 255), 1);
			light.emplace_back(lightRect);
			if (_lightConfigs.light_draw == 1 || _lightConfigs.light_edit == 1)
			{
				cv::Point2f vertex[4];
				lightRect.points(vertex);
				
				for (size_t l = 0; l != 4; ++l)
				{
					cv::line(image,
						vertex[l], vertex[(l + 1) % 4],
						cv::Scalar(0, 255, 0), 1, 8);
				}
			}

		}
	}
	if (light.size() != 0)
	{
		std::cout << light.size() << "个灯条" << std::endl;
		return true;
	}
	else
	{
		return false;
	}

}

//寻找装甲板
int ArmorDetector::findArmor()
{
	for (int i = 0; i < light.size(); i++)
	{
		for (int j = i + 1; j < light.size(); j++)
		{
			int left_light = 0, right_light = 0;
			// 区分左右灯条
			if (light[i].center.x > light[j].center.x)
			{
				left_light = j;
				right_light = i;
			}
			else 
			{
				left_light = i;
				right_light = j;
			}

			armorData.leftLight = light[left_light];
			armorData.rightLight = light[right_light];
			
			// 装甲板倾斜弧度
			float error_angle =atan((light[right_light].center.y - light[left_light].center.y) /
					                (light[right_light].center.x - light[left_light].center.x));
			if (error_angle < 10.f) 
			{
				armorData.tan_angle = atan(error_angle) * 180 / CV_PI;
				// 拟合装甲板条件判断
		  		if (matchLights(left_light, right_light))
				{
					std::cout << "发现装甲板";
					// 装甲板内颜色平均强度
					if (10/*颜色强度*/< 30) 
					{
						// 储存装甲板
						//armor.push_back(armorData);
						if (_armorConfigs.armor_draw == 1 ||
							_armorConfigs.armor_edit == 1)
						{
							//cv::line(image, armorData.armor_lt, armorData.armor_rb, cv::Scalar(255, 255, 0), 5, 8);
							for (int i = 0; i < armor.size(); i++)
							{
								
								for (size_t i = 0; i < 4; i++)
								{
									cv::line(image, armorData.armor_points[i], armorData.armor_points[(i + 1) % 4], cv::Scalar(255, 255, 0
									), 2, 8, 0);
								}

								
								//cv::RotatedRect rects = cv::RotatedRect(armorData.p0, armorData.p1, armorData.p2);

								//cv::rectangle(image, armorData.armor_lt, armorData.armor_rb, cv::Scalar(255, 255, 0), 2);
								
								return 1;

							}
						}
					}
				}
			}
		}
	}
	return 0;
}

//灯条匹配
bool ArmorDetector::matchLights(const int i, const int j)
{
	armorData.left_light_height =
		MAX(light[i].size.height, light[i].size.width);
	armorData.left_light_width =
		MIN(light[i].size.height, light[i].size.width);
	armorData.right_light_height =
		MAX(light[j].size.height, light[j].size.width);
	armorData.right_light_width =
		MIN(light[j].size.height, light[j].size.width);
	armorData.light_height_aspect =
		armorData.left_light_height / armorData.right_light_height;
	armorData.light_width_aspect =
		armorData.left_light_width / armorData.right_light_width;
	
	// 左右灯条高宽比
	if (armorData.light_height_aspect < _armorConfigs.light_height_ratio_max * 0.1 &&
		armorData.light_height_aspect > _armorConfigs.light_height_ratio_min * 0.1 &&
		armorData.light_width_aspect  < _armorConfigs.light_width_ratio_max * 0.1 &&
		armorData.light_width_aspect  > _armorConfigs.light_height_ratio_min * 0.1) 
	{
		std::cout << "进行灯条匹配xxxx" << std::endl;
		armorData.height =
			MIN(armorData.leftLight.size.height, armorData.rightLight.size.height);
		// 灯条 y 轴位置差
		if (fabs(armorData.leftLight.center.y - armorData.rightLight.center.y) <
			armorData.height * _armorConfigs.light_y_different * 0.1) 
		{
			std::cout << "进行灯条匹配0000" << std::endl;
			// 灯条高度差
			if (fabs(armorData.leftLight.size.height - armorData.rightLight.size.height) <
				armorData.height * _armorConfigs.light_height_different * 0.1) 
			{
				std::cout << "进行灯条匹配1111" << std::endl;
				armorData.width =
					getDistance(armorData.leftLight.center,
						armorData.rightLight.center);

				armorData.aspectRatio = armorData.width / (MAX(armorData.leftLight.size.height, armorData.rightLight.size.height));
				// 灯条角度差
				if (fabs(armorData.leftLight.angle - armorData.rightLight.angle) <
					_armorConfigs.armor_angle_different * 0.1) 
				{
					cv::Point2f left_light_points[4];
					cv::Point2f right_light_points[4];
					armorData.leftLight.points(left_light_points);
					armorData.rightLight.points(right_light_points);
					armorData.p0 = left_light_points[0];
					armorData.p1 = left_light_points[1];
					armorData.p2= right_light_points[2];
					armorData.p3 = right_light_points[3];

					armorData.armor_points[0]= left_light_points[0];
					armorData.armor_points[1] = left_light_points[1];
					armorData.armor_points[2] = right_light_points[2];
					armorData.armor_points[3] = right_light_points[3];


					armorData.armor_lt = left_light_points[1];
					armorData.armor_rb = right_light_points[3];
					//armorData.armor_lt = (ppp[0] + ppp[1]) / 2;
					//armorData.armor_rb = (ppp2[2] + ppp2[3]) / 2;

					//armorData.armor_lt = cv::Point(armorData.leftLight.center.x - armorData.left_light_width / 2, armorData.leftLight.center.y - armorData.left_light_height / 2);
					//armorData.armor_rb = cv::Point(armorData.rightLight.center.x + armorData.right_light_width / 2, armorData.rightLight.center.y - armorData.right_light_height / 2);
					cv::RotatedRect rects = cv::RotatedRect(
						(armorData.leftLight.center + armorData.rightLight.center) / 2,
						cv::Size(armorData.width * 0.5, armorData.height * 0.5 + 100),
						armorData.tan_angle);
					//cv::RotatedRect rects = cv::RotatedRect(armorData.p0, armorData.p1, armorData.p2);
					//cv::Rect = cv::boundingRect(armorData.armor_points);
					armor.emplace_back(rects);
					// 装甲板保存灯条离中心点的距离
					armorData.distance_center =
						getDistance(rects.center,
							cv::Point(image.cols, image.rows));
					// 小装甲板比例范围
					if (armorData.aspectRatio > _armorConfigs.small_armor_aspect_min * 0.1 && 
						armorData.aspectRatio < _armorConfigs.armor_type_th * 0.1)
					{
						armorData.distinguish = 0;

						return true;
						// 大装甲板比例范围
					}
					else if (armorData.aspectRatio > _armorConfigs.armor_type_th * 0.1 &&
						armorData.aspectRatio < _armorConfigs.big_armor_aspect_max * 0.1)
					{
						armorData.distinguish = 1;

						return true;
					}
				}
			}
		}
	}

	return false;
}

//两点间距离
float ArmorDetector::getDistance(const cv::Point a, const cv::Point b)
{
	return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

int ArmorDetector::getArmorType()
{
	return armorData.distinguish;
}


std::vector<cv::Point2f> ArmorDetector::returnFinalArmorRect()
{	
	image_points.clear();

	for(int i=0;i<4;i++)
	{
		image_points.push_back(armorData.armor_points[i]);
		std::cout<<armorData.armor_points[i]<<std::endl;
	}

	return	image_points;
}











