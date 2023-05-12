#include "KalmanPredictor.h"

namespace KalmanPredictor {

Predictor::Predictor()
{

}

void Predictor::Init(double dT, double alpha, double sigma_a2, double R0)
{
    alpha_ = alpha;
    sigma_a2_= sigma_a2;

    nominal_dT_ = dT;
    F_ << 1,     dT,     (alpha_*dT + std::exp(-alpha_*dT) - 1) / (pow(alpha_, 2)),
          0,      1,                             (1 - std::exp(-alpha_*dT))/alpha_,
          0,      0,                                           std::exp(-alpha_*dT);

    H_ << 1, 0, 0;
    R_offSet_ = R0;
    P_offset_ << R0,      0,         0,
                  0,  R0/dT,         0,
                  0,      0,  R0/dT/dT;

    double q11 = 2*alpha_*sigma_a2_*(1 - std::exp(-2*alpha_*dT) + 2*alpha_*dT + 2/3*pow(alpha_, 3)*pow(dT, 3) - 2*pow(alpha_, 2)*pow(dT, 2) - 4*alpha_*dT*exp(-alpha_*dT)) / (2*pow(alpha_, 5));
    double q12 = 2*alpha_*sigma_a2_*(1 + std::exp(-2*alpha_*dT) - 2*std::exp(-alpha_*dT) + 2*alpha_*dT*exp(-alpha_*dT) - 2*alpha_*dT + pow(alpha_, 2)*pow(dT, 2)) / (2*pow(alpha_, 4));
    double q13 = 2*alpha_*sigma_a2_*(1 - std::exp(-2*alpha_*dT) - 2*alpha_*dT*std::exp(-alpha_*dT)) / (2*pow(alpha_, 3));
    double q22 = 2*alpha_*sigma_a2_*(4*std::exp(-alpha_*dT) - 3 - std::exp(-2*alpha_*dT) + 2*alpha_*dT) / (2*pow(alpha_, 3));
    double q23 = 2*alpha_*sigma_a2_*(1 + std::exp(-2*alpha_*dT) - 2*std::exp(-alpha_*dT)) / (2*pow(alpha_, 2));
    double q33 = 2*alpha_*sigma_a2_*(1 - std::exp(-2*alpha_*dT)) / (2*alpha_);

    Q_offset_ << q11, q12, q13,
                q12, q22, q23,
                q13, q23, q33;

}

void Predictor::Reset(Eigen::Vector3d& x)
{
    x_ = x;
    P_ = P_offset_;
    R_ = R_offSet_;
    is_tracking_ = true;
}

void Predictor::SetState(Eigen::Vector3d& x)
{
  x_ = x;
}

void Predictor::StateUpdate(double dT, bool flag)
{
    if(flag) {
        F_ << 1,     dT,     (alpha_*dT + std::exp(-alpha_*dT) - 1) / (pow(alpha_, 2)),
                0,      1,                             (1 - std::exp(-alpha_*dT))/alpha_,
                0,      0,                                           std::exp(-alpha_*dT);

        double q11 = 2*alpha_*sigma_a2_*(1 - std::exp(-2*alpha_*dT) + 2*alpha_*dT + 2/3*pow(alpha_, 3)*pow(dT, 3) - 2*pow(alpha_, 2)*pow(dT, 2) - 4*alpha_*dT*exp(-alpha_*dT)) / (2*pow(alpha_, 5));
        double q12 = 2*alpha_*sigma_a2_*(1 + std::exp(-2*alpha_*dT) - 2*std::exp(-alpha_*dT) + 2*alpha_*dT*exp(-alpha_*dT) - 2*alpha_*dT + pow(alpha_, 2)*pow(dT, 2)) / (2*pow(alpha_, 4));
        double q13 = 2*alpha_*sigma_a2_*(1 - std::exp(-2*alpha_*dT) - 2*alpha_*dT*std::exp(-alpha_*dT)) / (2*pow(alpha_, 3));
        double q22 = 2*alpha_*sigma_a2_*(4*std::exp(-alpha_*dT) - 3 - std::exp(-2*alpha_*dT) + 2*alpha_*dT) / (2*pow(alpha_, 3));
        double q23 = 2*alpha_*sigma_a2_*(1 + std::exp(-2*alpha_*dT) - 2*std::exp(-alpha_*dT)) / (2*pow(alpha_, 2));
        double q33 = 2*alpha_*sigma_a2_*(1 - std::exp(-2*alpha_*dT)) / (2*alpha_);

        Q_ << q11, q12, q13,
                q12, q22, q23,
                q13, q23, q33;

        x_ = F_*x_;
        P_ = F_*P_*(F_.transpose()) + Q_;
    }
    else {
        x_.y() = 0.0;
        x_.z() = 0.0;

        F_ << 1,     dT,     (alpha_*dT + std::exp(-alpha_*dT) - 1) / (pow(alpha_, 2)),
                0,      1,                             (1 - std::exp(-alpha_*dT))/alpha_,
                0,      0,                                           std::exp(-alpha_*dT);

        double q11 = 2*alpha_*sigma_a2_*(1 - std::exp(-2*alpha_*dT) + 2*alpha_*dT + 2/3*pow(alpha_, 3)*pow(dT, 3) - 2*pow(alpha_, 2)*pow(dT, 2) - 4*alpha_*dT*exp(-alpha_*dT)) / (2*pow(alpha_, 5));
        double q12 = 2*alpha_*sigma_a2_*(1 + std::exp(-2*alpha_*dT) - 2*std::exp(-alpha_*dT) + 2*alpha_*dT*exp(-alpha_*dT) - 2*alpha_*dT + pow(alpha_, 2)*pow(dT, 2)) / (2*pow(alpha_, 4));
        double q13 = 2*alpha_*sigma_a2_*(1 - std::exp(-2*alpha_*dT) - 2*alpha_*dT*std::exp(-alpha_*dT)) / (2*pow(alpha_, 3));
        double q22 = 2*alpha_*sigma_a2_*(4*std::exp(-alpha_*dT) - 3 - std::exp(-2*alpha_*dT) + 2*alpha_*dT) / (2*pow(alpha_, 3));
        double q23 = 2*alpha_*sigma_a2_*(1 + std::exp(-2*alpha_*dT) - 2*std::exp(-alpha_*dT)) / (2*pow(alpha_, 2));
        double q33 = 2*alpha_*sigma_a2_*(1 - std::exp(-2*alpha_*dT)) / (2*alpha_);

        Q_ << q11, q12, q13,
                q12, q22, q23,
                q13, q23, q33;

        x_ = F_*x_;
        P_ = F_*P_*(F_.transpose()) + Q_;
    }
}

void Predictor::PredictWithVelocity(double dT)
{
  F_ << 1,     dT,     0.0,
        0,      1,     0.0,
        0,      0,     1.0;

  double q11 = 2*alpha_*sigma_a2_*(1 - std::exp(-2*alpha_*dT) + 2*alpha_*dT + 2/3*pow(alpha_, 3)*pow(dT, 3) - 2*pow(alpha_, 2)*pow(dT, 2) - 4*alpha_*dT*exp(-alpha_*dT)) / (2*pow(alpha_, 5));
  double q12 = 2*alpha_*sigma_a2_*(1 + std::exp(-2*alpha_*dT) - 2*std::exp(-alpha_*dT) + 2*alpha_*dT*exp(-alpha_*dT) - 2*alpha_*dT + pow(alpha_, 2)*pow(dT, 2)) / (2*pow(alpha_, 4));
  double q13 = 2*alpha_*sigma_a2_*(1 - std::exp(-2*alpha_*dT) - 2*alpha_*dT*std::exp(-alpha_*dT)) / (2*pow(alpha_, 3));
  double q22 = 2*alpha_*sigma_a2_*(4*std::exp(-alpha_*dT) - 3 - std::exp(-2*alpha_*dT) + 2*alpha_*dT) / (2*pow(alpha_, 3));
  double q23 = 2*alpha_*sigma_a2_*(1 + std::exp(-2*alpha_*dT) - 2*std::exp(-alpha_*dT)) / (2*pow(alpha_, 2));
  double q33 = 2*alpha_*sigma_a2_*(1 - std::exp(-2*alpha_*dT)) / (2*alpha_);

  Q_ << q11, q12, q13,
          q12, q22, q23,
          q13, q23, q33;

  x_ = F_*x_;
  P_ = F_*P_*(F_.transpose()) + Q_;
}

void Predictor::MeasurementUpdate(double z, bool flag)
{ 
  if(flag) {
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    z_ = z;
    P_ = (H_.transpose() * (1 / R_) * H_ + P_.inverse()).inverse();
    K_ = P_*H_.transpose() * (1 / R_);
    x_ = x_ + K_*(z_ - H_*x_);      
  }
  else {
    Eigen::Vector3d x_hat = x_;
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    z_ = z;
    P_ = (H_.transpose() * (1 / R_) * H_ + P_.inverse()).inverse();
    K_ = P_*H_.transpose() * (1 / R_);
    x_ = x_ + K_*(z_ - H_*x_);
    x_.tail<2>() = x_hat.tail<2>();
  }

}

Eigen::Vector3d Predictor::GetState()
{
    return x_;
}

Eigen::Vector3d Predictor::GetPredictState(double dT)
{
    Eigen::Matrix3d transform;
    transform << 1,     dT,     0.5*dT*dT,
                0,      1,            dT,
                0,      0,             1;
    return transform * x_;
}

  
}