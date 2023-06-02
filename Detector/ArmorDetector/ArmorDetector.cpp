#include"ArmorDetector.hpp"
using namespace armor_detector;

ArmorDetector::ArmorDetector()
{

}

ArmorDetector::~ArmorDetector()
{

}
bool ArmorDetector::runArmorDetector(const cv::Mat& srcImg, const EnemyColor enemy_color)
{
	release();
	imgPretreat(srcImg, enemy_color);

	if (isFindLights())
	{
		std::cout << "发现灯条"<<std::endl;
		findArmor();
	
	cv::namedWindow("output", 0);
					cv::resizeWindow("output",640,480);
					cv::imshow("output", image);
	return true;
	}
	else
	{
		std::cout << "没有装甲板"<<std::endl  ;
		return false;
	}
	
	
	
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

		if (perimeter < light_config.perimeter_min ||
			perimeter > light_config.perimeter_max ||
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
		if (lightRect.angle < light_config.maxAngle &&
			lightRect.angle > -light_config.minAngle &&
			aspectRatio < light_config.maxAspectRatio &&
			aspectRatio > light_config.minAspectRatio)
		{
			 
			cv::ellipse(image, lightRect, cv::Scalar(0, 255, 255), 1);
			light.emplace_back(lightRect);
			if (light_config.light_draw == 1 || light_config.light_edit == 1)
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
void ArmorDetector::findArmor()
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

			armor_data.leftLight = light[left_light];
			armor_data.rightLight = light[right_light];
			
			// 装甲板倾斜弧度
			float error_angle =atan((light[right_light].center.y - light[left_light].center.y) /
					                (light[right_light].center.x - light[left_light].center.x));
			if (error_angle < 10.f) 
			{
				armor_data.tan_angle = atan(error_angle) * 180 / CV_PI;
				// 拟合装甲板条件判断
		  		if (matchLights(left_light, right_light))
				{
					std::cout << "发现装甲板";
					// 装甲板内颜色平均强度
					if (10/*颜色强度*/< 30) 
					{
						// 储存装甲板             
						armor.push_back(armor_data);
						if (armor_config.armor_draw == 1 ||
							armor_config.armor_edit == 1)
						{
							
							for (int i = 0; i < armors_.size(); i++)
							{
								
								for (size_t i = 0; i < 4; i++)
								{
									cv::line(image, armor_data.armor_points[i], armor_data.armor_points[(i + 1) % 4], cv::Scalar(0, 255, 255), 2, 8, 0);
								}								

							}
						}
					}
				}
			}
		}
	}

}

//灯条匹配
bool ArmorDetector::matchLights(const int i, const int j)
{
	armor_data.left_light_height =
		MAX(light[i].size.height, light[i].size.width);
	armor_data.left_light_width =
		MIN(light[i].size.height, light[i].size.width);
	armor_data.right_light_height =
		MAX(light[j].size.height, light[j].size.width);
	armor_data.right_light_width =
		MIN(light[j].size.height, light[j].size.width);
	armor_data.light_height_aspect =
		armor_data.left_light_height / armor_data.right_light_height;
	armor_data.light_width_aspect =
		armor_data.left_light_width / armor_data.right_light_width;
	
	// 左右灯条高宽比
	if (armor_data.light_height_aspect < armor_config.light_height_ratio_max * 0.1 &&
		armor_data.light_height_aspect > armor_config.light_height_ratio_min * 0.1 &&
		armor_data.light_width_aspect  < armor_config.light_width_ratio_max * 0.1 &&
		armor_data.light_width_aspect  > armor_config.light_height_ratio_min * 0.1) 
	{
		armor_data.height =
			MIN(armor_data.leftLight.size.height, armor_data.rightLight.size.height);
		// 灯条 y 轴位置差
		if (fabs(armor_data.leftLight.center.y - armor_data.rightLight.center.y) <
			armor_data.height * armor_config.light_y_different * 0.1) 
		{
			
			// 灯条高度差
			if (fabs(armor_data.leftLight.size.height - armor_data.rightLight.size.height) <
				armor_data.height * armor_config.light_height_different * 0.1) 
			{
				armor_data.width =
					getDistance(armor_data.leftLight.center,
						armor_data.rightLight.center);

				armor_data.aspectRatio = armor_data.width / (MAX(armor_data.leftLight.size.height, armor_data.rightLight.size.height));
				// 灯条角度差
				if (fabs(armor_data.leftLight.angle - armor_data.rightLight.angle) <
					armor_config.armor_angle_different * 0.1) 
				{
					cv::Point2f left_light_points[4];
					cv::Point2f right_light_points[4];
					armor_data.leftLight.points(left_light_points);
					armor_data.rightLight.points(right_light_points);

					armor_data.armor_points[0]= left_light_points[1];
					armor_data.armor_points[1] = left_light_points[0];
					armor_data.armor_points[2] = right_light_points[3];
					armor_data.armor_points[3] = right_light_points[2];


					armor_data.armor_lt = left_light_points[1];
					armor_data.armor_rb = right_light_points[3];
					
					cv::RotatedRect rects = cv::RotatedRect(
						(armor_data.leftLight.center + armor_data.rightLight.center) / 2,
						cv::Size(armor_data.width * 0.5, armor_data.height * 0.5 + 100),
						armor_data.tan_angle);

					armor_data.center_distance =
						getDistance(rects.center,
							cv::Point(image.cols, image.rows));
					std::cout << "==================DEBUG=======================" << std::endl;

					armors_.push_back(rects);
					// 小装甲板比例范围
					if (armor_data.aspectRatio > armor_config.small_armor_aspect_min * 0.1 && 
						armor_data.aspectRatio < armor_config.armor_type_th * 0.1)
					{
						armor_data.distinguish = 0;

						return true;
						// 大装甲板比例范围
					}
					else if (armor_data.aspectRatio > armor_config.armor_type_th * 0.1 &&
						armor_data.aspectRatio < armor_config.big_armor_aspect_max * 0.1)
					{
						armor_data.distinguish = 1;

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
	return armor_data.distinguish;
}




bool ArmorDetector::getOptimalTarget(std::vector<cv::Point2f> &image_points)
{

  if(!armor.empty())
  {
    //fmt::print("[{}] Info, multiple armors\n", idntifier_green);
    // 离图像中心点大小排序从小到大
    std::sort(armor.begin(), armor.end(),
      [](ArmorData _a, ArmorData _b) 
	  {
      return _a.center_distance < _b.center_distance;
    });
	
	armor[0].delta_center=getDistance(armor[0].armor_center,last_armor_center);

    if (armor_config.armor_draw == 1 ||
        armor_config.armor_edit == 1) 
	{
		for (size_t i = 0; i < 4; i++)
		{
			cv::line(image, armor[0].armor_points[i], armor[0].armor_points[(i + 1) % 4], cv::Scalar(255, 255, 0), 2, 8, 0);
		}
    }
	image_points.clear();

	for(int i=0;i<4;i++)
	{
		image_points.push_back(armor[0].armor_points[i]);
		std::cout<<armor[0].armor_points[i]<<std::endl;
	}

	return	true;
  }

  return false;

  


}








