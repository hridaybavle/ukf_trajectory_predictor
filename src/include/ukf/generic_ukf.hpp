#include <ctime>
#include <mutex>
#include <atomic>
#include <memory>
#include <iomanip>
#include <iostream>
#include <unordered_map>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

//egien
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>

//ros
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

//predictor
#include "ukf/predictor.hpp"

//updater
#include "ukf/updater.hpp"

class generic_ukf 
{

public:
    generic_ukf();
    ~generic_ukf();

public:
    void init();
    void setUKFParams(int num_state, int num_state_noise, int num_meas,
                      Eigen::MatrixXf P, Eigen::MatrixXf Q, Eigen::MatrixXf R,
                      float alpha, float beta, float lamda);
    void setUKFQRMat();
    void setPredictionModel(Eigen::VectorXf model_f);
    void setStateInitValue(Eigen::VectorXf x);
    void initUKFParams();
    void UKFPrediction(float dt);
    void UKFUpdate(Eigen::VectorXf Z_measured);

    //predictor and updater
private:
    std::unique_ptr<ukf_predictor> ukf_predictor_ptr_;
    std::unique_ptr<ukf_updater> ukf_updater_ptr_;

    //UKF internal params
private:
    int num_state_, num_meas_;
    Eigen::VectorXf state_vec_x_, state_vec_x_aug_;
    Eigen::VectorXf meas_vec_z_;
    Eigen::MatrixXf predicted_cov_, predicted_cov_aug_;
    Eigen::MatrixXf meas_noise_cov_, process_noise_cov_;
    float alpha_, beta_, lamda_;
    Eigen::VectorXf weight_m_, weight_c_;
    int num_sigma_points_;
    Eigen::VectorXf model_f_;
    Eigen::MatrixXf X_predicted_, Xsig_aug_;
;


public:
    void getState(Eigen::VectorXf& X);
    void getStateCov(Eigen::MatrixXf& P);

};
