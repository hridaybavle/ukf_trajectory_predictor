//trajectory prediction
#include "aukf_trajectory_predictor.h"

aukf_traj_pre::aukf_traj_pre()
{

    this->init();

}

aukf_traj_pre::~aukf_traj_pre()
{


}

void aukf_traj_pre::init()
{
   received_odom_data_ = false;
}

void aukf_traj_pre::ownSetUp()
{

}

void aukf_traj_pre::ownStart()
{


}


void aukf_traj_pre::ownStop()
{


}

void aukf_traj_pre::ownRun()
{
    this->getDronePoseTF();

    return;
}

void aukf_traj_pre::getDronePoseTF()
{
    //check for tf
    tf::StampedTransform drone_pose_transform;
    try{
        ros::Time now = ros::Time::now();
        drone_pose_listener_.waitForTransform("/odom","/drone_close_kalman", now, ros::Duration(0.1));
        drone_pose_listener_.lookupTransform("/odom","/drone_close_kalman", now, drone_pose_transform);
        received_odom_data_ = true;

        measurements_.time_stamp = ros::Time::now();
        measurements_.x          = drone_pose_transform.getOrigin().x();
        measurements_.y          = drone_pose_transform.getOrigin().y();
        measurements_.z          = drone_pose_transform.getOrigin().z();
    }
    catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
        received_odom_data_ = false;
    }
}
