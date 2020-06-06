/*!*********************************************************************************
 *  \file       robot_process.h
 *  \brief      RobotProcess definition file.
 *  \details    This file contains the RobotProcess declaration. To obtain more information about
 *              it's definition consult the robot_process.cpp file.
 *  \authors    Enrique Ortiz, Yolanda de la Hoz, Martin Molina, David Palacios,
 *              Alberto Camporredondo
 *  \copyright  Copyright (c) 2018 Universidad Politecnica de Madrid
 *              All rights reserved
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

#ifndef ROBOT_PROCESS
#define ROBOT_PROCESS

#include <string>
#include <stdio.h>
#include <pthread.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <aerostack_msgs/AliveSignal.h>
#include <aerostack_msgs/ProcessState.h>
#include <aerostack_msgs/ProcessError.h>

#define STATE_CREATED aerostack_msgs::ProcessState::Created
#define STATE_READY_TO_START aerostack_msgs::ProcessState::ReadyToStart
#define STATE_RUNNING aerostack_msgs::ProcessState::Running
#define STATE_PAUSED aerostack_msgs::ProcessState::Paused

#define STATE_STARTED aerostack_msgs::ProcessState::Started
#define STATE_NOT_STARTED aerostack_msgs::ProcessState::NotStarted

/*!********************************************************************************************************************
 *  \class      RobotProcess
 *  \brief      This is the core class that every ROS node has to inherit.
 *  \details    The RobotProcess base class adds the following functionalities to the derived
 *              classes that inherits it:\n
 *              - Declaration of all the states a ROS node can be at.
 *              - Creation of a signal sending thread: By deriving this class the node will
 *                  create a thread with the only purpose of sending its state to a PerformanceMonitor,
 *                  that will be hearing at the 'State' topic.
 *              - Declaration of methods that the derived class will have to implement in order to
 *                  add the desired functionality to the ROS node.
 *
 *********************************************************************************************************************/
class RobotProcess
{
  // variables
public:
  //! States match the values defined in ProcessState.msg
  using State = uint8_t;

  //! This enum defines all posible RobotProcess errors that can be sent to the PerformanceMonitor.
  typedef enum
  {
    UnexpectedProcessStop,
    InvalidInputData,
    SafeguardRecoverableError,
    SafeguardFatalError,
  } Error;

protected:
  pthread_t t1;  //!< Thread handler.

  ros::NodeHandle node_handler_robot_process;

  std::string watchdog_topic;  //!< Attribute storing topic name to send alive messages to the PerformanceMonitor.
  std::string error_topic;     //!< Attribute storing topic name to send errors to the PerformanceMonitor.

  ros::ServiceServer start_server_srv;  //!< ROS service handler used to order a process to start.
  ros::ServiceServer stop_server_srv;   //!< ROS service handler used to order a process to stop.
  ros::ServiceServer is_running_srv;    //!< ROS service handler used to check if a process is in RUNNING state.

  ros::Publisher state_pub;  //!< ROS publisher handler used to send state messages.
  ros::Publisher error_pub;  //!< ROS publisher handler used to send error messages.

  aerostack_msgs::AliveSignal state_message;  //!< Message of type state.

protected:               //!< These attributes are protected because ProcessMonitor uses them.
  State current_state;   //!< Attribute storing current state of the process.
  std::string drone_id;  //!< Attribute storing the drone on which is executing the process.
  std::string hostname;  //!< Attribute storing the computer name on which the process is executing.

  // methods
public:
  //! Constructor.
  RobotProcess();

  ~RobotProcess();

  //!  This function calls to ownSetUp().
  void setUp();

  //!  This function calls to ownStart().
  //!  The first time is called this function calls to ownRun().
  void start();

  //!  This function calls to ownStop().
  void stop();

  /*!*****************************************************************************************************************
   * \brief This function calls to ownRun() when the process is Running.
   * \details This function must be called by the user in ownRun when he is implementing a synchronus execution, when
   *using ros::spinOnce(). Don't use this function if using ros::spin().
   *******************************************************************************************************************/
  void run();

  /*!*****************************************************************************************************************
   * \details If the node has an already defined state (Waiting, Running...) returns
   * the state as an Integer, if not it returns -1 to indicate the current state is undefined.
   * \return Void function
   *******************************************************************************************************************/
  State getState();

  /*!******************************************************************************************************************
   * \details The function accepts one of the already defined states to modify the 'curent_state' attribute.
   * It also sends and alive message to the PerformanceMonitor indicating the new state of the node.
   * \param   new_state The new state the process is going to have.
   * \return  The current state of the process.
   *******************************************************************************************************************/
  void setState(State new_state);

  /*!******************************************************************************************************************
   * \brief Send a RobotProcess.error to the Processs Monitor
   * \details This function is a first aproach at error handling. The error comunication between nodes
   * is performed by a two-part message. The first part indicates the type of message we are sending
   * (info, warning, error) while the second part offers a detailed description of the problem.
   * \param [in] type            The type of error we are going to send.
   * \param [in] reference_code  This is a numeric code that may be usefull during error processing.
   * \param [in] location        The location is a string that explains at which function the error occured.
   * \param [in] description     Another String for the human reader that explains in detail the error.
   *******************************************************************************************************************/
  void notifyError(Error type, int reference_code, std::string location, std::string description);

  /*!******************************************************************************************************************
   * \brief The stateToString method transforms the recieved state into a human readable String.
   * \param [in] state The received state that need to be transformed.
   * \return the state in a String form.
   *******************************************************************************************************************/
  std::string stateToString(State state);

protected:
  //!  This function sends an alive message to the PerformanceMonitor indicating the current node state.
  void notifyState();

  /*!******************************************************************************************************************
   * \brief This function sends an alive message to the PerformanceMonitor indicating the current node state.
   * \param [in] state State that has to be sent to the PerformanceMonitor.
   *******************************************************************************************************************/
  void notifyState(State state);

  /*!******************************************************************************************************************
   * \brief This function has the purpose to serve as the thread execution point.
   * \param [in] argument Function which has to be executed by the thread.
   *******************************************************************************************************************/
  static void* threadRun(void* argument);

  //!  This function implements the thread's logic.
  void threadAlgorithm();

  /*!******************************************************************************************************************
   * \brief This ROS service set RobotProcess in READY_TO_START state and calls function stop.
   * \details Currently, this service should only be called if the process is running. In the future
   * it will also be correct to call this service when the process is paused.
   * \param [in] request
   * \param [in] response
   *******************************************************************************************************************/
  bool stopSrvCall(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  /*!******************************************************************************************************************
   * \brief This ROS service set RobotProcess in RUNNING state and calls function start.
   * \details Currently, this service should only be called if the process is ready to start. In the future
   * it will also be correct to call this service when the process is paused or running.
   * \param [in] request
   * \param [in] response
   *******************************************************************************************************************/
  bool startSrvCall(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

protected:
  /*!******************************************************************************************************************
   * \details All functions starting with 'own' has to be implemented at the derived class.
   * This function is executed in setUp() and it's purpose is to set up all the parameters.
   * the developer considers necesary when needed.
   *******************************************************************************************************************/
  virtual void ownSetUp() = 0;

  /*!******************************************************************************************************************
   * \details All functions starting with 'own' has to be implemented at the derived class.
   * This function is executed in start(), and it's purpose is to connect to all topics and services.
   * Publisher, Subscriber, ServiceServer and ServiceClient definitions should be implemented here.
   *******************************************************************************************************************/
  virtual void ownStart() = 0;

  /*!******************************************************************************************************************
   * \details All functions starting with 'own' has to be implemented at the derived class.
   * This function is executed in stop(), and it's purpose is to shutdown all topics and services.
   * Shutdown must be called for every Publisher, Subscriber, ServiceServer and ServiceClient defined in ownStart
   *******************************************************************************************************************/
  virtual void ownStop() = 0;

  /*!******************************************************************************************************************
   * \details All functions starting with 'own' has to be implemented at the derived class.
   * This function is executed in run(), only if the process is Running.
   * The user should define this function only when implementing a synchronus execution
   *******************************************************************************************************************/
  virtual void ownRun() = 0;
};
#endif
