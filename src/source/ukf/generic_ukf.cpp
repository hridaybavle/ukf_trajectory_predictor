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
                               float alpha, float beta, float lamda)
{
    num_state_ =  num_state; num_meas_ = num_meas;

    //state vector and covariance
    state_vec_x_.setZero(num_state);
    predicted_cov_.setZero(num_state, num_state);

    //measurement vector
    meas_vec_z_.setZero(num_meas);

    //noise covariances Q and R
    process_noise_cov_.setZero(num_state, num_state);
    meas_noise_cov_.setZero(num_meas, num_meas);

    predicted_cov_     = P;
    process_noise_cov_ = Q;
    meas_noise_cov_    = R;

    alpha_  = alpha;
    beta_   = beta;
    lamda_  = lamda;

    int num_aug = num_state_ + num_meas_;
    num_sigma_points_ = 2 * (num_aug) + 1;

    weight_m_.resize(num_sigma_points_);
    weight_c_.resize(num_sigma_points_);

    std::cout << "alpha " << alpha_ << std::endl;
    std::cout << "beta " << beta_ << std::endl;
    std::cout << "lamda " << lamda_ << std::endl;

    weight_m_(0) = lamda_ / (lamda_ + num_aug);
    weight_c_(0) = lamda_ / (lamda_ + num_aug) + (1 - pow(alpha_,2) +  beta_);

    for(int i=1; i < num_sigma_points_; ++i)
    {
        weight_m_(i) = 0.5 / (lamda_ + num_aug);
        weight_c_(i) = 0.5 / (lamda_ + num_aug);
    }

    std::cout << "weight m " << weight_m_ << std::endl;
    std::cout << "weight c " << weight_c_ << std::endl;

    ukf_predictor_ptr_->setPredictionParams(num_state_, num_meas_,
                                            num_sigma_points_, lamda_,
                                            weight_m_, weight_c_);
    ukf_updater_ptr_->setUpdateParams(num_state_, num_meas_,
                                      num_sigma_points_, lamda_,
                                      weight_m_, weight_c_);

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

void generic_ukf::getStateCov(Eigen::MatrixXf& P)
{
    P = predicted_cov_;
    return;
}


void generic_ukf::UKFPrediction(float dt)
{
    Eigen::MatrixXf Xsig_aug;
    Xsig_aug = ukf_predictor_ptr_->generateSigmaPoints(state_vec_x_, predicted_cov_,
                                                       process_noise_cov_, meas_noise_cov_);

    X_predicted_ = ukf_predictor_ptr_->predictUsingSigmaPoints(Xsig_aug, dt);

    state_vec_x_ = ukf_predictor_ptr_->predictMeanAndCovariance(X_predicted_, predicted_cov_);

    std::cout << "State x predicted " << state_vec_x_ << std::endl;
    std::cout << "P Predicted" << predicted_cov_ <<  std::endl;
}

void generic_ukf::UKFUpdate(Eigen::VectorXf Z_measured)
{
    //using the predicted Z calculated to get the sigma points measurement update
    Eigen::MatrixXf Z_predicted;
    Z_predicted = ukf_updater_ptr_->calculatePredictedMeasurement(X_predicted_);

    //performing the update using the measurements
    Eigen::MatrixXf Kalman_gain;
    Kalman_gain = ukf_updater_ptr_->measurementUpdate(Z_predicted, X_predicted_, state_vec_x_,
                                                      meas_noise_cov_);

    //updating the state and its covariance
    ukf_updater_ptr_->updateMeanAndCovariance(state_vec_x_, predicted_cov_, Z_measured);

    std::cout << "State x corrected " << state_vec_x_ << std::endl;
    std::cout << "P corrected" << predicted_cov_ <<  std::endl;
}
