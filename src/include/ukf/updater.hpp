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
                         int num_sigma_points, float lamda){

        state_size_       = num_state;
        measurement_size_ = num_meas;
        num_sigma_points_ = num_sigma_points;
        lamda_            = lamda;
        state_size_aug_   = state_size_ + measurement_size_;

        return;

    }

    Eigen::MatrixXf calculatePredictedMeasurement(Eigen::MatrixXf X_predicted) {

        Eigen::MatrixXf Z_predicted; Z_predicted.resize(measurement_size_, num_sigma_points_);

        for(int i = 0; i < num_sigma_points_; ++i)
        {
            Z_predicted(0,i) = X_predicted(0,i);
            Z_predicted(1,i) = X_predicted(1,i);
            Z_predicted(2,i) = X_predicted(2,i);
        }

        return Z_predicted;
    }

    Eigen::MatrixXf measurementUpdate(Eigen::MatrixXf Z_predicted, Eigen::MatrixXf X_predicted, Eigen::VectorXf X_estimate,
                                      Eigen::VectorXf Wm, Eigen::VectorXf Wc){

        Eigen::MatrixXf Wm_mat = Wm.transpose().replicate(Z_predicted.rows(),1);
        Eigen::MatrixXf Wc_mat = Wc.transpose().replicate(Z_predicted.rows(),1);

        Eigen::MatrixXf Z_predicted_weighted = (Z_predicted.array() * Wm_mat.array()).matrix();
        Eigen::VectorXf Z_estimate = Z_predicted_weighted.rowwise().sum();

        Eigen::MatrixXf Z_mean_distance = (Z_predicted.colwise() - Z_estimate);
        Eigen::MatrixXf Z_weighted_mean_distance = (Z_mean_distance.array() * Wm_mat.array()).matrix();
        Eigen::MatrixXf S = Z_weighted_mean_distance * Z_mean_distance.transpose();

        Eigen::MatrixXf X_mean_distance = (X_predicted.colwise() - X_estimate);
        Eigen::MatrixXf X_mean_distance_weighted = (X_mean_distance.array() * Wc_mat.array()).matrix();
        Eigen::MatrixXf S_x_z = X_mean_distance_weighted * Z_mean_distance.transpose();

        Eigen::MatrixXf K;
        std::cout << "S " << S << std::endl;
        std::cout << "S_x_z " << S_x_z << std::endl;

        if(!S.isZero())
            K = S_x_z * S.inverse();
        else

        return K;
    }

private:
    Eigen::MatrixXf prediction_f_;
    int state_size_, measurement_size_, state_size_aug_ ;
    int num_sigma_points_;
    float lamda_;

};

#endif
