#ifndef PREDICTOR_HPP
#define PREDICTOR_HPP

//egien
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>


class ukf_predictor 
{

public:
    ukf_predictor() {}
    ~ukf_predictor() {}

public:
    void generate_sigma_points();
    void predict_sigma_points();
    void ukf_prediction();


    void setPredictionFunction(Eigen::MatrixXf f)
    {
        prediction_f_ = f;
    }

private:
    Eigen::MatrixXf prediction_f_;

};

#endif
