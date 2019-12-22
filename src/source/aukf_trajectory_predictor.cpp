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
    future_point_vec_.clear();

    generic_ukf_ptr_.reset(new generic_ukf);
    future_traj_pred_ptr_.reset(new future_trajectory_predictor);

    Eigen::MatrixXf P;
    std::cout << "state size " << state_size_ << std::endl;
    std::cout << "measurement size " << measurement_size_ << std::endl;

    P.setZero(state_size_, state_size_);
    //P.diagonal().fill(1e-5);
    P.diagonal() << 1e-2, 10, 1e-2, 0.1, 10, 10, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2;
    std::cout << "P " << P << std::endl;
    Q_.setZero(measurement_size_, measurement_size_);
    Q_.diagonal().fill(1e-3);
    std::cout << "Q " << Q_ << std::endl;
    R_.setZero(measurement_size_, measurement_size_);
    R_.diagonal().fill(1e-12);
    std::cout << "R " << R_ << std::endl;
    float alpha = 1e-3;
    float beta  = 2;
    float lamda = 3 - (state_size_+measurement_size_);

    generic_ukf_ptr_->setUKFParams(state_size_, measurement_size_,
                                   P, Q_, R_, alpha, beta, lamda);

    future_traj_pred_ptr_->setFutureTrajPredParam(state_size_, measurement_size_,
                                                  alpha, beta, lamda);
}

void ukf_traj_pre::readROSParams()
{
    ros::param::param<int>("~state_size",state_size_,13);
    ros::param::param<int>("~measurement_size",measurement_size_,13);
}

void ukf_traj_pre::ownSetUp()
{
    this->init();

    //publisher
    future_trajectory_pub_  = n.advertise<nav_msgs::Path>("drone_future_path", 1);
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

        //get the updated state and covariance to predict future trajectory
        Eigen::VectorXf X; Eigen::MatrixXf P;
        std::vector<Eigen::VectorXf> future_state_vec;
        generic_ukf_ptr_->getState(X); generic_ukf_ptr_->getStateCov(P);
        future_state_vec = future_traj_pred_ptr_->generateFutureTrajectory(X, P, Q_, R_, deltaT_);
        this->publishFutureTrajectory(future_state_vec);

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


void ukf_traj_pre::publishFutureTrajectory(std::vector<Eigen::VectorXf> future_state_vec)
{
    future_point_vec_.clear();
    for(size_t i=0; i < future_state_vec.size(); ++i)
    {
        geometry_msgs::PoseStamped point;
        point.header.frame_id = "odom";
        point.header.stamp = ros::Time::now();

        point.pose.position.x = future_state_vec[i](0);
        point.pose.position.y = future_state_vec[i](2);
        point.pose.position.z = future_state_vec[i](4);

        future_point_vec_.push_back(point);
    }

    nav_msgs::Path future_path;
    future_path.header.stamp = ros::Time::now();
    future_path.header.frame_id = "odom";
    future_path.poses = future_point_vec_;
    future_trajectory_pub_.publish(future_path);
}
