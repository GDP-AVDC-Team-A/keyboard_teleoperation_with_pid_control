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
#include <boost/thread/thread.hpp>
#include <curses.h>
#include <thread>
#include <locale.h>
#include "droneMsgsROS/droneCommand.h"
#include <droneMsgsROS/InitiateBehaviors.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "mav_msgs/RollPitchYawrateThrust.h"
#include "aerostack_msgs/SetControlMode.h"
#include "aerostack_msgs/QuadrotorPidControllerMode.h"
#include <aerostack_msgs/RequestBehavior.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_srvs/Empty.h"

#include "droneMsgsROS/dronePitchRollCmd.h"
#include "droneMsgsROS/droneDYawCmd.h"
#include "droneMsgsROS/droneDAltitudeCmd.h"
#include "droneMsgsROS/vector2Stamped.h"
//Inputs
#define ASCII_KEY_UP 65
#define ASCII_KEY_DOWN 66
#define ASCII_KEY_RIGHT 67
#define ASCII_KEY_LEFT 68

// Define controller commands define constants
#define CTE_SPEED (1.00)
#define CTE_POSE (1.00)
#define CTE_COMMANDS (0.20)
#define CTE_COMMANDS_TIME (0.50)
#define CTE_ALTITUDE (1.00)
#define CTE_YAW (0.1)
//Loop rate
#define FREQ_INTERFACE 200.0

int miliseconds = CTE_COMMANDS_TIME * 1000;
const int GROUND_SPEED = 1;
const int POSE = 2;
const int ATTITUDE = 3;

//Publishers
ros::Publisher command_publ;
ros::Publisher speed_reference_publ;
ros::Publisher pose_reference_publ;
ros::Publisher command_pitch_roll_publ;
//Subscribers
ros::Subscriber self_pose_sub;
ros::Subscriber control_mode;
ros::Subscriber speed_reference_sub;
ros::Subscriber command_pitch_roll_sub;
ros::Subscriber command_pitch_roll_stop_sub;
ros::Subscriber ground_speed_sub;
//Services
ros::ServiceClient setControlModeClientSrv;
ros::ServiceClient startQuadrotorControllerClientSrv;
ros::ServiceClient initiate_behaviors_srv;
ros::ServiceClient activate_behavior_srv;

//MSG
aerostack_msgs::QuadrotorPidControllerMode control_mode_msg;
droneMsgsROS::droneCommand command_order;
geometry_msgs::PoseStamped self_localization_pose_msg; 
geometry_msgs::PoseStamped motion_reference_pose_msg; 
geometry_msgs::TwistStamped speed_reference_msg;
geometry_msgs::TwistStamped current_speed_ref;
droneMsgsROS::vector2Stamped ground_speed_msg;

droneMsgsROS::dronePitchRollCmd command_pitch_roll_msg;
droneMsgsROS::dronePitchRollCmd command_pitch_roll_msg_temp;
//Services variables
std_srvs::Empty req;
droneMsgsROS::InitiateBehaviors msg;
aerostack_msgs::RequestBehavior::Request msg2;
aerostack_msgs::RequestBehavior::Response res;
aerostack_msgs::BehaviorCommand behavior;

//Functions
void printoutPoseControls();
void printoutGroundSpeedControls();
void printoutAttitudeControls();
void publishCmd();
void takeOff();
void hover();
void land();
void emergencyStop();
void move();
void selfLocalizationPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void controlModeCallback(const aerostack_msgs::QuadrotorPidControllerMode::ConstPtr& msg);
void speedReferenceCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
void commandPitchRollCallback(const droneMsgsROS::dronePitchRollCmd::ConstPtr& msg);
void commandPitchRollCallbackStop(const droneMsgsROS::dronePitchRollCmd::ConstPtr& msg);
void groundSpeedCallback(const droneMsgsROS::vector2Stamped::ConstPtr& msg){ground_speed_msg=*msg;}
void publishSpeedReference();
bool setControlMode(int new_control_mode);
void clearSpeedReferences();
int current_mode;
//Topics
std::string drone_id_namespace;
std::string command_high_level_topic_name;
std::string speed_ref_topic_name;
std::string pose_ref_topic_name;
std::string self_pose_topic_name;
std::string assumed_control_mode_topic_name;
std::string set_control_mode_service_name;
std::string command_pitch_roll_topic_name;
std::string ground_speed_topic_name;
//Attitude control
bool up;
bool right;
bool left;
bool down;

#endif
