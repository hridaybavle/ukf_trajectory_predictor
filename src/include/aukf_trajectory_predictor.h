//STL
#include <ctime>
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include <numeric>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <iostream>
#include <string>
#include "cstdlib"

//ROS
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "gazebo_msgs/ModelStates.h"

//Drone module
#include "robot_process.h"

//tf
#include <tf/transform_listener.h>

//ukf library
#include "ukf/generic_ukf.hpp"

//future_traj_predictor
#include "ukf_future_trajectory_predictor.h"


class ukf_traj_pre : public RobotProcess
{
public:
    ukf_traj_pre();
    ~ukf_traj_pre();

public: 
    void ownSetUp();
    void ownStart();
    void ownStop();
    void ownRun();
    void init();
    void readROSParams();

protected:
    ros::NodeHandle n;
    ros::Publisher future_trajectory_pub_;
    ros::Subscriber drone_gazebo_pose_sub_;

    void droneGazeboPoseCallback(const gazebo_msgs::ModelStates msg);
private:
    void getDronePoseTF();
    tf::TransformListener drone_pose_listener_;

private:
    geometry_msgs::PointStamped measurements_;
    bool received_odom_data_;
    std::unique_ptr<generic_ukf> generic_ukf_ptr_;
    double deltaT_, timePrev_, timeNow_;
    bool init_time_;

    //ukf related
private:
    int state_size_, measurement_size_, state_noise_size_;
    bool simulation_;
    Eigen::MatrixXf Q_; Eigen::MatrixXf R_;

private:
    std::unique_ptr<future_trajectory_predictor> future_traj_pred_ptr_;
    void publishFutureTrajectory(std::vector<Eigen::VectorXf> future_state_vec);
    std::vector<geometry_msgs::PoseStamped> future_point_vec_;
};
