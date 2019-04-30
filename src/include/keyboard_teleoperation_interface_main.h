/*!*******************************************************************************************
 *  \file       keyboard_teleoperation_interface_main.h
 *  \brief      Keyboard teleoperation inferface implementation file.
 *  \details    The keyboard teleoperation interface provides control vehicle by keyboard only in SPEED MODE
 *              It allows multiple commands: Take off, land, move, hover, rotate...
 *  \authors    Alberto Rodelgo Perales
 *  \copyright  Copyright (c) 2019 Universidad Politecnica de Madrid
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
 ********************************************************************************/ 

#ifndef KEYBOARD_TELEOPERATION_INTERFACE_H
#define KEYBOARD_TELEOPERATION_INTERFACE_H

#include <string>
#include "ros/ros.h"
#include <sstream>
#include <stdio.h>
#include <curses.h>
#include <thread>
#include "droneMsgsROS/setControlMode.h"
#include "droneMsgsROS/droneCommand.h"
#include <droneMsgsROS/InitiateBehaviors.h>
#include "control/Controller_MidLevel_controlModes.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "mav_msgs/RollPitchYawrateThrust.h"
#include "aerostack_msgs/ControlMode.h"
#include <aerostack_msgs/BehaviorSrv.h>
#include "std_srvs/Empty.h"
//Inputs
#define ASCII_KEY_UP 65
#define ASCII_KEY_DOWN 66
#define ASCII_KEY_RIGHT 67
#define ASCII_KEY_LEFT 68

// Define controller commands define constants
#define CONTROLLER_CTE_COMMAND_SPEED (1.00)
#define CONTROLLER_STEP_COMMAND_ALTITUDE (0.25)
#define CONTROLLER_STEP_COMMAND_YAW (10.0 * (M_PI / 180.0))
//Loop rate
#define FREQ_INTERFACE 200.0

#define DISPLAY_COLUMN_SIZE 22

//Publishers
ros::Publisher command_publ;
ros::Publisher multirotor_command_publ;
ros::Publisher speed_reference_publ;
ros::Publisher pose_reference_publ;
//Subscribers
ros::Subscriber self_pose_sub;
ros::Subscriber control_mode;
ros::Subscriber speed_reference_sub;
//Services
ros::ServiceClient setControlModeClientSrv;
ros::ServiceClient startQuadrotorControllerClientSrv;
ros::ServiceClient initiate_behaviors_srv;
ros::ServiceClient activate_behavior_srv;

//MSG
aerostack_msgs::FlightMotionControlMode control_mode_msg;
mav_msgs::RollPitchYawrateThrust multirotor_command_msg;
droneMsgsROS::droneCommand command_order;
geometry_msgs::PoseStamped self_localization_pose_msg; 
geometry_msgs::TwistStamped speed_reference_msg;
geometry_msgs::TwistStamped current_speed_ref;

//Services variables
std_srvs::Empty req;
std::string initiate_behaviors;
std::string activate_behavior;
droneMsgsROS::InitiateBehaviors msg;
aerostack_msgs::BehaviorSrv::Request msg2;
aerostack_msgs::BehaviorSrv::Response res;
aerostack_msgs::BehaviorCommand behavior;

//Functions
void printout_stream(std::stringstream* pinterface_printout_stream, int* lineCommands, int* columCommands);
void printoutControls();
void publishCmd();
void takeOff();
void clearCmd();
void hover();
void land();
void emergencyStop();
void move();
void publishMultirotorCommand();
void selfLocalizationPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void controlModeCallback(const aerostack_msgs::FlightMotionControlMode::ConstPtr& msg);
void speedReferenceCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
void publishSpeedReference();
bool setControlMode(Controller_MidLevel_controlMode::controlMode new_control_mode);

//Topics
std::string drone_id_namespace;
std::string command_high_level_topic_name;
std::string quadrotor_command_topic_name;
std::string speed_ref_topic_name;
std::string pose_ref_topic_name;
std::string self_pose_topic_name;
std::string assumed_control_mode_topic_name;
std::string set_control_mode_service_name;

#endif