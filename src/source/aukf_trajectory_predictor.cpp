//trajectory prediction
#include "aukf_trajectory_predictor.h"

ukf_traj_pre::ukf_traj_pre()
{
    std::cout << "ukf trajectory predictor constructor" << std::endl;
}

ukf_traj_pre::~ukf_traj_pre()
{

}

void ukf_traj_pre::init()
{
    this->readROSParams();
    received_odom_data_ = false;
    init_time_          = false;
    timePrev_ = 0; timeNow_ = 0;

    generic_ukf_ptr_.reset(new generic_ukf);

    Eigen::MatrixXf P;
    std::cout << "state size " << state_size_ << std::endl;
    std::cout << "measurement size " << measurement_size_ << std::endl;

    P.setZero(state_size_, state_size_);
    P.diagonal().fill(1e-5);
    std::cout << "P " << P << std::endl;
    Eigen::MatrixXf Q; Q.setZero(measurement_size_, measurement_size_);
    Q.diagonal().fill(1e-5);
    std::cout << "Q " << Q << std::endl;
    Eigen::MatrixXf R; R.setZero(measurement_size_, measurement_size_);
    R.diagonal().fill(0.001);
    std::cout << "R " << R << std::endl;
    float alpha = 1e-3;
    float beta  = 2;
    float lamda = 3 - (state_size_+measurement_size_);

    generic_ukf_ptr_->setUKFParams(state_size_, measurement_size_,
                                   P, Q, R, alpha, beta, lamda);

    this->generate_model_f(0);
    generic_ukf_ptr_->setPredictionModel(model_f_);
}

void ukf_traj_pre::readROSParams()
{
    ros::param::param<int>("~state_size",state_size_,13);
    ros::param::param<int>("~measurement_size",measurement_size_,13);
}

void ukf_traj_pre::ownSetUp()
{
    this->init();

}

void ukf_traj_pre::ownStart()
{


}


void ukf_traj_pre::ownStop()
{


}

void ukf_traj_pre::ownRun()
{
    this->getDronePoseTF();
    timeNow_ = (double) ros::Time::now().sec + ((double) ros::Time::now().nsec / (double) 1E9);

    if(!init_time_){
        deltaT_ = 0;
        init_time_ = true;
    }
    else {
        deltaT_ = timeNow_ - timePrev_;
    }

    std::cout << "dt out side " << deltaT_ << std::endl;
    //perform the prediction
    generic_ukf_ptr_->UKFPrediction(deltaT_);

    //perform the update
    if(received_odom_data_)
    {
        Eigen::Vector3f z_measured;
        z_measured(0) = measurements_.point.x;
        z_measured(1) = measurements_.point.y;
        z_measured(2) = measurements_.point.z;

        generic_ukf_ptr_->UKFUpdate(z_measured);
        received_odom_data_ = false;
    }

    timePrev_ = timeNow_;
    return;
}

void ukf_traj_pre::getDronePoseTF()
{
    //check for tf
    tf::StampedTransform drone_pose_transform;
    try{
        ros::Time now = ros::Time::now();
        drone_pose_listener_.waitForTransform("/odom","/drone_close_kalman", now, ros::Duration(0.1));
        drone_pose_listener_.lookupTransform("/odom","/drone_close_kalman", now, drone_pose_transform);
        received_odom_data_ = true;

        measurements_.header.stamp = ros::Time::now();
        measurements_.point.x      = drone_pose_transform.getOrigin().x();
        measurements_.point.y      = drone_pose_transform.getOrigin().y();
        measurements_.point.z      = drone_pose_transform.getOrigin().z();
    }
    catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
        received_odom_data_ = false;
    }
}

void ukf_traj_pre::generate_model_f(float dt)
{
    //create the prediction model matrix to pass it to the UKF
    model_f_.setZero(state_size_);

    /* This is the non-linear model for curve estimation
    X(12) = X(12);                                     //curv_d
    X(11) = X(11) + dt * X(12);                        //curv
    X(10) = X(10);                                     //acc
    X(9) = X(9) + dt * X(10);                          //vel
    X(8) = X(9) * X(11);                               //tetha_d
    X(7) = X(7) + dt * X(8);                           //theta
    X(6) = 0;//X(6);                                   //z_dd
    X(5) = X(5) + dt * X(6);                           //z_d
    X(4) = X(4) + dt * X(5) + 0.5 * pow(dt, 2) * X(6); //z
    X(3) = sin(X(7)) * X(9);                           //y_d
    X(2) = X(2) + dt * X(3);                           //y
    X(1) = cos(X(7)) * X(9);                           //x_d
    X(0) = X(0) + dt * X(1);                           //x */

    Eigen::VectorXf X;
    generic_ukf_ptr_->getState(X);

    model_f_(0)    = X(0) + X(1) * dt;                                 //x */
    model_f_(1)    = X(1) + cos(X(7)) * X(9);                          //x_d (using Xsig_pre(7), Xsig_pre(9) not a bug)
    model_f_(2)    = X(2) + X(3) * dt;                                 //y
    model_f_(3)    = X(3) + sin(X(7)) * X(9);                          //y_d (using Xsig_pre(7), Xsig_pre(9) not a bug)
    model_f_(4)    = X(4) + X(5) * dt + 0.5 * pow(dt, 2) * X(6);       //z
    model_f_(5)    = X(5) + X(6) * dt;                                //z_d
    model_f_(6)    = X(6);                                            //z_dd
    model_f_(7)    = X(7) + X(8) * dt;                                //theta
    model_f_(8)    = X(8) + X(9) * X(11);                             //tetha_d
    model_f_(9)    = X(9) + X(10)* dt;                                //vel
    model_f_(10)   = X(10);                                   //acc
    model_f_(11)   = X(11) + X(12) * dt;                               //curv
    model_f_(12)   = X(12);                                     //curv_d

}







