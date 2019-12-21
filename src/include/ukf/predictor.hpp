#ifndef PREDICTOR_HPP
#define PREDICTOR_HPP
#include <cstdint>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <random>
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

    void setPredictionParams(int num_state, int num_meas,
                             int num_sigma_points, float lamda,
                             Eigen::VectorXf Wm, Eigen::VectorXf Wc){

        state_size_       = num_state;
        measurement_size_ = num_meas;
        num_sigma_points_ = num_sigma_points;
        lamda_            = lamda;
        state_size_aug_   = state_size_ + measurement_size_;
        Wm_ = Wm; Wc_ = Wc;

        return;

    }

    void setPredictionFunction(Eigen::MatrixXf f){
        prediction_f_ = f;
    }

    Eigen::MatrixXf generateSigmaPoints(Eigen::VectorXf X, Eigen::MatrixXf P,
                                        Eigen::MatrixXf Q, Eigen::MatrixXf R)
    {
        //augmented state vector and covariance
        Eigen::VectorXf X_aug;
        X_aug.setZero(state_size_aug_);
        X_aug.head(state_size_) = X;

        Eigen::MatrixXf P_aug;
        P_aug.setZero(state_size_aug_, state_size_aug_);
        P_aug.topLeftCorner(state_size_, state_size_) = P;
        P_aug.bottomRightCorner(measurement_size_, measurement_size_) = Q;

        //create sigma points
        Eigen::MatrixXf Xsig_aug; Xsig_aug.setZero(X_aug.rows(), num_sigma_points_);

        //calculate the square rooot of P_aug
        Eigen::MatrixXf A = P_aug.llt().matrixL();

        //calculate sigma points
        Xsig_aug.col(0) = X_aug;
        float square_root = std::sqrt(lamda_ + state_size_aug_);
        //std::cout << "square root " << square_root << std::endl;

        for(int i=0; i < state_size_aug_; ++i)
        {
            Xsig_aug.col(i + 1) = X_aug + (square_root * A.col(i));
            Xsig_aug.col(i + (1 + state_size_aug_)) = X_aug - (square_root * A.col(i));
        }

        return Xsig_aug;
    }

    Eigen::MatrixXf predictUsingSigmaPoints(Eigen::MatrixXf Xsig_aug, float dt){


        Eigen::MatrixXf X_predicted; X_predicted.setZero(state_size_, num_sigma_points_);

        for(int i= 0; i < Xsig_aug.cols(); ++i)
        {
            X_predicted(0,i) = Xsig_aug(0,i) + Xsig_aug(1,i) * dt;                                          //x
            X_predicted(1,i) = cos(Xsig_aug(7,i)) * Xsig_aug(9,i);                                          //x_d
            X_predicted(2,i) = Xsig_aug(2,i) + Xsig_aug(3,i) * dt;                                          //y
            X_predicted(3,i) = sin(Xsig_aug(7,i)) * Xsig_aug(9,i);                                          //y_d
            X_predicted(4,i) = Xsig_aug(4,i) + Xsig_aug(5,i) * dt + 0.5 * pow(dt, 2) * Xsig_aug(6,i);       //z
            X_predicted(5,i) = Xsig_aug(5,i) + Xsig_aug(6,i) * dt;                                          //z_d
            X_predicted(6,i) = Xsig_aug(6,i) + 1e-6;                                           //z_dd
            X_predicted(7,i) = Xsig_aug(7,i) + Xsig_aug(8,i) * dt;                                          //theta
            X_predicted(8,i) = Xsig_aug(9,i) * Xsig_aug(11,i);                                              //tetha_d
            X_predicted(9,i) = Xsig_aug(9,i) + Xsig_aug(10,i)* dt;                                          //vel
            X_predicted(10,i)= Xsig_aug(10,i) + 1e-4;                                                              //acc
            X_predicted(11,i)= Xsig_aug(11,i) + Xsig_aug(12,i) *dt;                                        //curv
            X_predicted(12,i)= Xsig_aug(12,i) + 1e-6;                                          //curv_d

            //            std::cout << "Xsig_aug rows "  << Xsig_aug.rows() << std::endl;
            //            std::cout << "Xsig_aug(13,i) " << Xsig_aug(13,i) << std::endl;
            //            std::cout << "Xsig_aug(14,i) " << Xsig_aug(14,i) << std::endl;
            //            std::cout << "Xsig_aug(15,i) " << Xsig_aug(15,i) << std::endl;

            //X_predicted.col(i) = X_predicted.col(i) + X_noise;
        }

        return X_predicted;

    }


    Eigen::VectorXf predictMeanAndCovariance(Eigen::MatrixXf X_predicted, Eigen::MatrixXf& P){

        Eigen::VectorXf X;
        X.setZero(state_size_); P.setZero(state_size_, state_size_);

        // predict the mean
        for(int i= 0; i < num_sigma_points_; ++i)
        {
            X += Wm_(i) * X_predicted.col(i);
        }

        // calculate the covariance
        for(int i =0; i < num_sigma_points_; ++i)
        {
            Eigen::MatrixXf X_mean_distance = (X_predicted.col(i) - X);
            P += Wc_(i) * (X_mean_distance * X_mean_distance.transpose());
        }

        return X;
    }

private:
    Eigen::MatrixXf prediction_f_;
    int state_size_, measurement_size_, state_size_aug_ ;
    int num_sigma_points_;
    float lamda_;
    Eigen::VectorXf Wm_, Wc_;
};

#endif
