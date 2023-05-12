#ifndef KALMAN_PREDICTOR_H
#define KALMAN_PREDICTOR_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>

namespace KalmanPredictor {

class Predictor 
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Predictor();
    //初始化
    void Init(double dT, double alpha, double sigma_a2, double R0);
    //
    void Reset(Eigen::Vector3d& x);
    //设置状态
    void SetState(Eigen::Vector3d& x);
    //状态更新
    void StateUpdate(double dT, bool flag);
    //预测
    void PredictWithVelocity(double dT);
    
    void MeasurementUpdate(double z, bool flag);

    Eigen::Vector3d GetState();

    Eigen::Vector3d GetPredictState(double dT);

    Eigen::Vector3d x_;
    double z_;

    Eigen::Matrix3d F_;
    Eigen::Matrix3d P_;
    Eigen::Matrix<double, 1, 3> H_;
    Eigen::Matrix3d Q_;
    double R_;
    Eigen::Matrix<double, 3, 1> K_;
    Eigen::Matrix3d P_offset_;
    Eigen::Matrix3d Q_offset_;
    double R_offSet_; 


    double alpha_;
    double sigma_a2_;
    double nominal_dT_;
    bool is_tracking_ = false;
    bool is_skip_ = false;
};

}

#endif
