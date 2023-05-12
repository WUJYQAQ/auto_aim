#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <opencv2/opencv.hpp>
#include <iostream>
// #include "Tools.h"
// #include "Mat_time.h"
using namespace std;
class kalmanFilter
{
private:
    std::shared_ptr<cv::KalmanFilter> KF;
    const int measure_num = 3;  //测量值数量
    const int state_num = 6;    //状态值数量
    double bullet_speed = 15.0; //弹速
    uint32_t last_t = 0;    //上一时刻t

    double delay_time = 0.019; // 发弹延迟、通信延迟等较为固定延迟
    double k = 0.0402;         // 阻力系数
    double g = 9.75;           // 重力加速度
    int iter_num = 40;

    double bs_coeff = 0.9; // 初始弹速系数

public:
    cv::Point3f cam2gyro_offset; // 陀螺仪到相机的平移偏移量
    cv::Point3f gun2cam_offset;  // 枪口相对于相机的平移偏移量
    cv::Point2f pitch_time;      // 弹道补偿抬枪角度以及子弹飞行时间
    cv::Point3f world_coord;
    cv::Point3f correct_coord;

public:
    kalmanFilter();
    ~kalmanFilter() = default;

    /**
     *  @brief  计算KF->correct后下一点位置
     *  @param  result KF->correct的结果
     *  @param  time    图片处理延迟 + 子弹飞行时间
     */
    cv::Point3f predictNextpoint(cv::Mat result, float time)
    {
        float t = time;
        float x = result.at<float>(0, 0) + t * result.at<float>(1, 0);
        float y = result.at<float>(2, 0) + t * result.at<float>(3, 0);
        float z = result.at<float>(4, 0) + t * result.at<float>(5, 0);
        return cv::Point3f(x, y, z);
    }
 
    /**
     *  @brief  初始化状态值
     *  @param  camera_coord   相机坐标系下目标的坐标
     */
    void initState(cv::Point3f coord);


    /**
     *  @brief  根据相机坐标进行预测, 对距离滤波
     *  @param  camera_coord   相机坐标系下目标的坐标
     *  @return 相机坐标系预测点
     */
    cv::Point3f predict(cv::Point3f camera_coord, uint32_t timestamp);

  

    

};

#endif