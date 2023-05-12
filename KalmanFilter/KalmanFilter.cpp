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
    cout << "SOLVE_WORLD: " << camera_coord << endl;
    cv::Mat measurement = (cv::Mat_<float>(measure_num, 1) << camera_coord.x, 
                                                              camera_coord.y, 
                                                              camera_coord.z);

    KF->transitionMatrix.at<float>(0, 1) = dt;
    KF->transitionMatrix.at<float>(2, 3) = dt;
    KF->transitionMatrix.at<float>(4, 5) = dt;

    KF->predict();

    correct_state = KF->correct(measurement);
    correct_coord = cv::Point3f(correct_state.at<float>(0, 0), correct_state.at<float>(2, 0), correct_state.at<float>(4, 0));

    cout << correct_state << endl;

    cv::Point3f world_next = predictNextpoint(correct_state, 30 + dt + delay_time);
    return world_next;
    //return camera_coord;
}


void kalmanFilter::initState(cv::Point3f coord)
{

        KF->statePost = (cv::Mat_<float>(state_num, 1) << world_coord.x,
                                                        0,
                                                        world_coord.y,
                                                        0,
                                                        world_coord.z,
                                                        0);
    
}


