#include "ukf/generic_ukf.hpp"


generic_ukf::generic_ukf()
{
    std::cout << "generic ukf constructor " << std::endl;
    this->init();

}

generic_ukf::~generic_ukf()
{
    std::cout << "generic ukf destructor " << std::endl;
}

void generic_ukf::init()
{
    ukf_predictor_ptr_.reset(new ukf_predictor);
    ukf_updater_ptr_.reset(new ukf_updater);

    return;
}

void generic_ukf::setUKFParams(int num_state, int num_meas,
                               Eigen::MatrixXf P, Eigen::MatrixXf Q, Eigen::MatrixXf R,
                               double alpha, double beta, double lamda)
{
    num_state_ =  num_state; num_meas_ = num_meas;

    //state vector and covariance
    state_vec_x_.setZero(num_state);
    predicted_cov_.setZero(num_state, num_meas);

    //measurement vector
    meas_vec_z_.setZero(num_meas);

    //noise covariances Q and R
    process_noise_cov_.setZero(num_state, num_state);
    meas_noise_cov_.setZero(num_meas, num_meas);

    predicted_cov_ = P;
    process_noise_cov_ = Q;
    meas_noise_cov_ = R;

    alpha_ = alpha;
    beta_ = beta;
    lamda_ = lamda;

    num_sigma_points_ = 2 * num_state_ + 1;

    weight_m_.resize(num_sigma_points_);
    weight_c_.resize(num_sigma_points_);

    weight_m_(0) = lamda_ / (lamda_ + num_state_);
    weight_c_(0) = lamda_ / (lamda_ + num_state_) + (1 - pow(alpha_,2) +  beta_);

    for(int i=0; i < num_sigma_points_; ++i)
    {
        weight_m_(i) = 0.5 / (lamda_ + num_state_);
        weight_c_(i) = 0.5 / (lamda_ + num_state_);
    }
}

void generic_ukf::setStateInitValue(Eigen::VectorXf x)
{
    state_vec_x_ = x;
    return;
}

void generic_ukf::setPredictionModel(Eigen::VectorXf model_f)
{
    model_f_ = model_f;
    return;
}

void generic_ukf::getState(Eigen::VectorXf& X)
{
    X = state_vec_x_;
    return;
}

void generic_ukf::UKFPrediction(float dt)
{




}



