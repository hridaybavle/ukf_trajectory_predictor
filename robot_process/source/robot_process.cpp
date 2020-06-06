/*!*******************************************************************************************
 *  \file       robot_process.cpp
 *  \brief      RobotProcess implementation file.
 *  \details    This file implements the RobotProcess class.
 *  \authors    Enrique Ortiz, Yolanda de la Hoz, Martin Molina, David Palacios,
 *              Alberto Camporredondo
 *  \copyright  Copyright (c) 2018 Universidad Politecnica de Madrid
 *              All Rights Reserved
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ********************************************************************************/

#include "../include/robot_process.h"

RobotProcess::RobotProcess()
{
  watchdog_topic = "process_alive_signal";
  error_topic = "process_error";
  char buf[32];
  gethostname(buf, sizeof buf);
  hostname.append(buf);

  current_state = STATE_CREATED;  // This State is not going to be sent. It will we significant in the future when we
                                  // implement state checks.
}

RobotProcess::~RobotProcess()
{
  pthread_cancel(t1);
}

void RobotProcess::setUp()
{
  state_pub = node_handler_robot_process.advertise<aerostack_msgs::AliveSignal>(watchdog_topic, 10);
  error_pub = node_handler_robot_process.advertise<aerostack_msgs::ProcessError>(error_topic, 10);

  stop_server_srv = node_handler_robot_process.advertiseService(ros::this_node::getName() + "/stop",
                                                                &RobotProcess::stopSrvCall, this);
  start_server_srv = node_handler_robot_process.advertiseService(ros::this_node::getName() + "/start",
                                                                 &RobotProcess::startSrvCall, this);

  notifyState();  // First state nNotification, the current state is STATE_CREATED

  pthread_create(&t1, NULL, &RobotProcess::threadRun, this);
  ownSetUp();
  setState(STATE_READY_TO_START);
}

void RobotProcess::start()
{
  setState(STATE_RUNNING);
  ownStart();
}

void RobotProcess::stop()
{
  setState(STATE_READY_TO_START);
  ownStop();
}

std::string RobotProcess::stateToString(State state)
{
  std::string result;
  switch (state)
  {
    case STATE_CREATED:
      result = "Created";
      break;
    case STATE_READY_TO_START:
      result = "ReadyToStart";
      break;
    case STATE_RUNNING:
      result = "Running";
      break;
    case STATE_PAUSED:
      result = "Paused";
      break;
    case STATE_STARTED:
      result = "Started";
      break;
    case STATE_NOT_STARTED:
      result = "NotStarted";
      break;
    default:
      ROS_WARN("In node %s, method stateToString received a invalid State. Value received is %d.",
               ros::this_node::getName().c_str(), state);
      break;
  }

  return result;
}

RobotProcess::State RobotProcess::getState()
{
  return current_state;
}

void RobotProcess::setState(State new_state)
{
  if (new_state == STATE_CREATED || new_state == STATE_READY_TO_START || new_state == STATE_RUNNING ||
      new_state == STATE_PAUSED || new_state == STATE_STARTED || new_state == STATE_NOT_STARTED)
  {
    current_state = new_state;
    notifyState();
  }
  else
  {
    notifyError(SafeguardFatalError, 1, "setState(State new_state)", "New state is invalid");
    ROS_ERROR("In node %s, current state cannot be changed to new state %d", ros::this_node::getName().c_str(),
              new_state);
  }
}

void RobotProcess::notifyState()
{
  State _current_state = getState();
  if (_current_state == STATE_CREATED || _current_state == STATE_READY_TO_START || _current_state == STATE_RUNNING ||
      _current_state == STATE_PAUSED || _current_state == STATE_STARTED || _current_state == STATE_NOT_STARTED)
  {
    state_message.header.stamp = ros::Time::now();
    state_message.hostname = hostname;
    state_message.process_name = ros::this_node::getName();
    state_message.current_state.state = _current_state;
    state_pub.publish(state_message);
  }
  else
  {
    notifyError(SafeguardFatalError, 1, "notifyState()", "Current state is invalid");
    ROS_ERROR("In node %s, current state is invalid, therefore it is not sent", ros::this_node::getName().c_str());
  }
}

void RobotProcess::notifyState(State current_state)
{
  state_message.header.stamp = ros::Time::now();
  state_message.process_name = ros::this_node::getName();
  state_message.hostname = hostname;
  state_message.current_state.state = (int)current_state;
  state_pub.publish(state_message);
}

void RobotProcess::notifyError(Error type, int reference_code, std::string function, std::string description)
{
  aerostack_msgs::ProcessError error_message;
  error_message.header.stamp = ros::Time::now();
  error_message.error_type.value = (int)type;
  error_message.hostname = hostname;
  error_message.process_name = ros::this_node::getName();
  error_message.function = function;
  error_message.description = description;
  error_message.reference_code = reference_code;
  error_pub.publish(error_message);
}

// COMMON METHODS
void* RobotProcess::threadRun(void* argument)
{
  ((RobotProcess*)argument)->threadAlgorithm();
  return NULL;
}

void RobotProcess::threadAlgorithm()
{
  ros::Rate r(1);
  while (ros::ok())
  {
    notifyState();
    r.sleep();
  }
}

bool RobotProcess::stopSrvCall(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  if (current_state == STATE_RUNNING)
  {
    stop();
    return true;
  }
  else
  {
    ROS_WARN("Node %s received a stop call when it was already stopped", ros::this_node::getName().c_str());
    return false;
  }
}

bool RobotProcess::startSrvCall(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  if (current_state == STATE_READY_TO_START)
  {
    start();
    return true;
  }
  else
  {
    ROS_WARN("Node %s received a start call when it was already running", ros::this_node::getName().c_str());
    return false;
  }
}

void RobotProcess::run()
{
  if (current_state == STATE_RUNNING)
    ownRun();
}
