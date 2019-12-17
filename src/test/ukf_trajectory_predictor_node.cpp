//trajectory prediction
#include "aukf_trajectory_predictor.h"

int main(int argc, char **argv)
{
  //Init
  ros::init(argc, argv, "aukf_trajectory_predictor_node"); //Say to ROS the name of the node and the parameters

  double frequency;
  ros::param::get("~frequency", frequency);
  if(frequency == 0)
    frequency = 30;

  aukf_traj_pre aukf_traj_pre_process;
  //Open!
  aukf_traj_pre_process.setUp();
  //Start
  aukf_traj_pre_process.start();

  ros::Rate r(frequency);

  //Loop -> Ashyncronous Module
  while(ros::ok())
  {
    ros::spinOnce();
    aukf_traj_pre_process.run();
    r.sleep();
  }

  return 1;
}

