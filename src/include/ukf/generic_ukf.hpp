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

    void setStateSize(int num);
    void setMeasurementSize(int num);
    void setUKFParams();
    void setUKFQRMat();
    void setPredictionModel();
    void setStateInitVale(Eigen::VectorXf x);

private:
    Eigen::VectorXf state_vec_x_;
    Eigen::VectorXf meas_vec_z_;

 //predictor and updater
private:
    std::unique_ptr<ukf_predictor> ukf_predictor_ptr_;
    std::unique_ptr<ukf_updater> ukf_updater_ptr_;


};
