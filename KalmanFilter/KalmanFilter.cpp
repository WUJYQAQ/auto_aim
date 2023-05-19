#include "KalmanFilter.h"

kalmanFilter::kalmanFilter()
{
    cv::FileStorage fs("Configs/kalman_filter/Settings.xml", cv::FileStorage::READ);
    cv::Mat processNoise, measurementNoise;
    cv::Mat t;
    fs["kalman_Q"] >> processNoise;
    fs["kalman_R"] >> measurementNoise;
    fs["cam2gyro_offset"] >> cam2gyro_offset;
    fs["gun2cam_offset"] >> gun2cam_offset;
    fs.release();

    KF = std::make_shared<cv::KalmanFilter>(state_num, measure_num, 0);

    cv::Mat H = (cv::Mat_<float>(measure_num, state_num) << 1, 0, 0, 0, 0, 0, 
                                                            0, 0, 1, 0, 0, 0, 
                                                            0, 0, 0, 0, 1, 0);

    KF->processNoiseCov = processNoise;                        // Q 过程噪声
    KF->measurementNoiseCov = measurementNoise;                // R 测量噪声
    cv::setIdentity(KF->errorCovPost, cv::Scalar::all(1));     // P 卡尔曼增益
    cv::setIdentity(KF->transitionMatrix, cv::Scalar::all(1)); // F 状态转移矩阵
    KF->measurementMatrix = H;
}


cv::Point3f kalmanFilter::predict(cv::Point3f camera_coord,  uint32_t timestamp)
{
  
    cv::Mat correct_state;
    float dt = (float)(timestamp - last_t) / 1000.0;
    last_t = timestamp;
    cout << "SOLVE_WORLD: " << world_coord << endl;
    cv::Mat measurement = (cv::Mat_<float>(measure_num, 1) << world_coord.x, 
                                                              world_coord.y, 
                                                              world_coord.z);

    KF->transitionMatrix.at<float>(0, 1) = dt;
    KF->transitionMatrix.at<float>(2, 3) = dt;
    KF->transitionMatrix.at<float>(4, 5) = dt;

    KF->predict();

    correct_state = KF->correct(measurement);
    correct_coord = cv::Point3f(correct_state.at<float>(0, 0), correct_state.at<float>(2, 0), correct_state.at<float>(4, 0));


    setBS_coeff(correct_coord);

    pitch_time = compensate(correct_coord, dt);
    cout << correct_state << endl;
    cout << "Pitch: " << pitch_time.x << "Time: " << pitch_time.y << endl;
    cout << "BS: " << bullet_speed << endl;


    cv::Point3f world_next = predictNextpoint(correct_state, 300 + 10 + delay_time);
    return world_next;
}


void kalmanFilter::initState(cv::Point3f coord, bool spin_flag)
{
    if(spin_flag){
        KF->statePost = (cv::Mat_<float>(state_num, 1) <<  coord.x,
                                                                KF->statePost.at<float>(1, 0),
                                                                coord.y,
                                                                KF->statePost.at<float>(3, 0),
                                                                coord.z,
                                                                KF->statePost.at<float>(5, 0));
        }

    else{
        KF->statePost = (cv::Mat_<float>(state_num, 1) << coord.x,
                                                        0,
                                                        coord.y,
                                                        0,
                                                        coord.z,
                                                        0);


    }
       
}

/**
 *  @brief  重力补偿；
 *  @param  bullet_speed    弹速
 *  @param  correct_spin    KF得出的坐标状态值
 *  @param  dt  图片处理时间
 * @return  pitch, time
 */
cv::Point2d kalmanFilter::compensate(cv::Point3f correct_spin, double dt)
{
	correct_spin -= gun2cam_offset;
	double dy, angle, y_actual;
	double t_actual = 0.0;
	double y_temp = correct_spin.z / 1000.0;
	double y = y_temp;
	double x = sqrt(correct_spin.x * correct_spin.x + correct_spin.y * correct_spin.y) / 1000.0;

	for (int i = 0; i < iter_num; i++)
	{
		angle = atan2(y_temp, x);
		t_actual = getflytime(angle, correct_spin, t_actual, dt);
		y_actual = double(bs_coeff * bullet_speed * sin(angle) * t_actual - g * t_actual * t_actual / 2.0);
		dy = y - y_actual;
		y_temp += dy;
		if (abs(dy) < 0.001)
			break;
	}
	return cv::Point2d((angle) / M_PI * 180.0, t_actual);
}

/**
 *  @brief  计算子弹飞行时间
 *  @param  angle   pitch角
 *  @param  correct_state   KF得出的状态值
 *  @param  T   n-1状态中子弹飞行时间
 *  @param  dt  图片处理延迟
 *  @return 返回子弹在绝对坐标系中飞行的时间；
 */
double kalmanFilter::getflytime(double angle, cv::Point3f correct_spin, double T, double dt)
{
    double x = sqrt(correct_spin.x * correct_spin.x + correct_spin.y * correct_spin.y) / 1000.0;

    return (exp(k * x) - 1.0) / (k * bs_coeff * bullet_speed * cos(angle)); //- dt - delay_time;
}


void kalmanFilter::predictAsync(cv::Point3f last_coord, kalmanFilter kalman_filter, std::function<void(cv::Point3f)> callback) {
    auto future = std::async(std::launch::async, &kalmanFilter::predict, &kalman_filter, last_coord, 10);
    auto result = future.get();
    callback(result);
}
