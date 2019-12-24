#ifndef UPDATER_HPP
#define UPDATER_HPP
#include <iostream>

//egien
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>

class ukf_updater
{
public:
    ukf_updater() {}
    ~ukf_updater() {}

public:
    void setUpdateParams(int num_state, int num_meas,
                         int num_sigma_points, float lamda,
                         Eigen::VectorXf Wm, Eigen::VectorXf Wc){

        state_size_       = num_state;
        measurement_size_ = num_meas;
        num_sigma_points_ = num_sigma_points;
        lamda_            = lamda;
        Wm_ = Wm; Wc_ = Wc;

        return;
    }

    Eigen::MatrixXf calculatePredictedMeasurement(Eigen::MatrixXf X_predicted, Eigen::MatrixXf Xsig_aug) {

        Eigen::MatrixXf Z_predicted; Z_predicted.resize(measurement_size_, num_sigma_points_);

        for(int i = 0; i < num_sigma_points_; ++i)
        {
            Z_predicted(0,i) = X_predicted(0,i) + Xsig_aug(13,i);
            Z_predicted(1,i) = X_predicted(2,i) + Xsig_aug(14,i);
            //Z_predicted(2,i) = X_predicted(4,i);
        }

        return Z_predicted;
    }

    Eigen::MatrixXf measurementUpdate(Eigen::MatrixXf Z_predicted, Eigen::MatrixXf X_prediction,
                                      Eigen::VectorXf X_estimate, Eigen::MatrixXf R){

        //Eigen::MatrixXf Wm_mat = Wm.transpose().replicate(Z_predicted.rows(),1);
        //Eigen::MatrixXf Wc_mat = Wc.transpose().replicate(X_predicted.rows(),1);

        Eigen::VectorXf Z_predicted_sum; Z_predicted_sum.setZero(measurement_size_);

        for(int i =0; i < num_sigma_points_; ++i)
        {
            Z_predicted_sum += Wm_(i) * Z_predicted.col(i);

        }
        Z_estimate_ = Z_predicted_sum;

        //measurement covariance [3*3]
        Eigen::MatrixXf Szz; Szz.setZero(measurement_size_, measurement_size_);
        //measurement cross covariance [13*3]
        Eigen::MatrixXf Szx; Szx.setZero(state_size_, measurement_size_);
        for(int i = 0; i < num_sigma_points_; ++i)
        {
            Eigen::VectorXf z_diff = Z_predicted.col(i) - Z_estimate_;
            Szz += Wc_(i) * (z_diff * z_diff.transpose());

            Eigen::VectorXf x_diff = X_prediction.col(i) - X_estimate;
            Szx +=  Wc_(i) * (x_diff * z_diff.transpose());
        }

        S_ = Szz;
        std::cout << "S_ " << S_ << std::endl;

        if(!S_.isZero())
            K_ = Szx * S_.inverse().eval();
        else
            K_.setZero(X_prediction.rows(), Z_predicted.rows());

        K_ = Szx * S_.inverse();
        std::cout << "kalman gain " << K_ << std::endl;


        return K_;
    }

    void updateMeanAndCovariance(Eigen::VectorXf& X_estimate, Eigen::MatrixXf& P,
                                 Eigen::VectorXf Z_measured)
    {
        std::cout << "Z measured " << Z_measured << std::endl;
        std::cout << "Z estimate " << Z_estimate_ << std::endl;
        X_estimate = X_estimate + K_ * (Z_measured - Z_estimate_);
        P          = P - K_ * S_ * K_.transpose();
    }

private:
    Eigen::MatrixXf prediction_f_;
    int state_size_, measurement_size_;
    int num_sigma_points_;
    float lamda_;
    Eigen::VectorXf Z_estimate_;
    Eigen::MatrixXf K_, S_;
    Eigen::VectorXf Wm_, Wc_;
};

#endif
