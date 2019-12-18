#include "ukf/generic_ukf.hpp"


generic_ukf::generic_ukf()
{
    std::cout << "generic ukf constructor " << std::endl;
    this->init();

}

generic_ukf::~generic_ukf()
{

}

void generic_ukf::init()
{
    ukf_predictor_ptr_.reset(new ukf_predictor);
    ukf_updater_ptr_.reset(new ukf_updater);
    return;
}

void generic_ukf::setStateSize(int num)
{
    state_vec_x_.setZero(num);
    return;
}

void generic_ukf::setStateInitVale(Eigen::VectorXf x)
{
    state_vec_x_ = x;
    return;
}

void generic_ukf::setMeasurementSize(int num)
{
    meas_vec_z_.setZero(num);
    return;
}

void generic_ukf::setPredictionModel(Eigen::VectorXf pred_f)
{
    //ukf_predictor_ptr_->setPredictionFunction(pred_f);
}


void generic_ukf::getState(Eigen::VectorXf& X)
{
    X = state_vec_x_;
    return;
}
