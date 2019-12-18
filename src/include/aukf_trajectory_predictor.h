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

//Drone module
#include "robot_process.h"

//tf
#include <tf/transform_listener.h>

//ukf library
#include "ukf/generic_ukf.hpp"

class aukf_traj_pre : public RobotProcess
{
public:
    aukf_traj_pre();
    ~aukf_traj_pre();

public: 
    void ownSetUp();
    void ownStart();
    void ownStop();
    void ownRun();
    void init();
    void readROSParams();

private:
    void getDronePoseTF();
    tf::TransformListener drone_pose_listener_;

private:
    struct {
        float x,y,z;
        ros::Time time_stamp;
    } measurements_;

    bool received_odom_data_;
    std::unique_ptr<generic_ukf> generic_ukf_ptr_;

    //ukf related
private:
    int state_size_, measurement_size_;
    void generate_model_f(float dt);
    Eigen::VectorXf model_f_;

};
