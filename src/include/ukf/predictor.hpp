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


    void setPredictionFunction(Eigen::MatrixXf f){
        prediction_f_ = f;
    }

    void generateSigmaPoints(float num_state, float num_meas, float num_sigma_points, float lamda,
                             Eigen::VectorXf X, Eigen::MatrixXf P,
                             Eigen::MatrixXf Q, Eigen::MatrixXf R)
    {
        //augmented state vector and covariance
        Eigen::VectorXf X_aug;
        X_aug.setZero(num_state+num_meas);
        X_aug.head(num_state) = X;

        Eigen::MatrixXf P_aug;
        P_aug.setZero(num_state+num_meas, num_state+num_meas);
        P_aug.topLeftCorner(num_state, num_state) = P;
        P_aug.bottomRightCorner(num_meas, num_meas) = Q;

        //create sigma points
        Eigen::MatrixXf Xsig_aug; Xsig_aug.resize(X_aug.rows(), num_sigma_points);

        //calculate the square rooot of P_aug
        Eigen::MatrixXf A = P_aug.llt().matrixL();

        //calculate sigma points
        Xsig_aug.col(0) = X_aug;

        Xsig_aug.block(0, 1, (num_state+num_meas), (num_state+num_meas)) = (std::sqrt(lamda + num_state+num_meas) * A).colwise() + X_aug;

        Xsig_aug.block(0,(num_state+num_meas)+1, (num_state+num_meas), (num_state+num_meas))
                = (-1 * sqrt(lamda + (num_state+num_meas)) * A).colwise() + X_aug;

    }

    void PredictUsingSigmaPoints(){

    }

private:
    Eigen::MatrixXf prediction_f_;

};

#endif
