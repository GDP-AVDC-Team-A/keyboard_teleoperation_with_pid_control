/*!*******************************************************************************************
 *  \file       keyboard_teleoperation_interface_main.cpp
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

#include "../include/keyboard_teleoperation_interface_main.h"

void spinnerThread(){
  ros::spin();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "KEYBOARD TELEOPERATION INTERFACE");
  ros::NodeHandle n;
  n.param<std::string>("drone_id_namespace", drone_id_namespace, "drone1");
  std::thread thr(&spinnerThread);
  printf("Starting Keyboard Teleoperation Interface...\n");

  ros::param::get("~command_high_level_topic_name", command_high_level_topic_name);
    if ( command_high_level_topic_name.length() == 0)
    {
        command_high_level_topic_name="command/high_level";
    }
   ros::param::get("~quadrotor_command_topic_name", quadrotor_command_topic_name);
    if ( quadrotor_command_topic_name.length() == 0)
    {
        quadrotor_command_topic_name="actuator_command/quadrotor_command";
    }
   ros::param::get("~speed_ref_topic_name", speed_ref_topic_name);
    if ( speed_ref_topic_name.length() == 0)
    {
        speed_ref_topic_name="motion_reference/speed";
    }
    ros::param::get("~pose_ref_topic_name", pose_ref_topic_name);
    if ( pose_ref_topic_name.length() == 0)
    {
        pose_ref_topic_name="motion_reference/pose";
    }
   ros::param::get("~self_pose_topic_name", self_pose_topic_name);
    if ( self_pose_topic_name.length() == 0)
    {
        self_pose_topic_name="self_localization/pose";
    }
   ros::param::get("~assumed_control_mode_topic_name", assumed_control_mode_topic_name);
    if ( assumed_control_mode_topic_name.length() == 0)
    {
        assumed_control_mode_topic_name="motion_reference/assumed_control_mode";
    }
    ros::param::get("~set_control_mode_service_name", set_control_mode_service_name);
    if ( set_control_mode_service_name.length() == 0)
    {
        set_control_mode_service_name="set_control_mode";
    }

  // ncurses initialization
  initscr();
  start_color();
  use_default_colors();  
  curs_set(0);
  noecho();
  nodelay(stdscr, TRUE);
  erase();
  refresh();
  init_pair(1, COLOR_RED, -1);
  init_pair(2, COLOR_GREEN, -1);

  //Print controls
  printoutControls();

  //Input variable
  char command = 0;

  //Publishers
  command_publ = n.advertise<droneMsgsROS::droneCommand>("/" + drone_id_namespace + "/"+ command_high_level_topic_name, 1, true);
  multirotor_command_publ = n.advertise<mav_msgs::RollPitchYawrateThrust>("/" + drone_id_namespace + "/"+quadrotor_command_topic_name, 1);
  speed_reference_publ = n.advertise<geometry_msgs::TwistStamped>("/"+drone_id_namespace+"/"+speed_ref_topic_name, 1, true);
  pose_reference_publ = n.advertise<geometry_msgs::PoseStamped>("/"+drone_id_namespace+"/"+pose_ref_topic_name, 1, true);

  //Subscribers
  self_pose_sub = n.subscribe("/"+drone_id_namespace+"/"+self_pose_topic_name, 1, selfLocalizationPoseCallback);
  control_mode = n.subscribe("/" + drone_id_namespace + "/" + assumed_control_mode_topic_name, 1, controlModeCallback);
  speed_reference_sub = n.subscribe("/"+drone_id_namespace+"/"+speed_ref_topic_name, 1, speedReferenceCallback);

  //Wait 3sec for initialization
  sleep(3);

  //Services
  startQuadrotorControllerClientSrv=n.serviceClient<std_srvs::Empty>("/"+drone_id_namespace+"/quadrotor_pid_controller_process/start");
  startQuadrotorControllerClientSrv.call(req);
  setControlModeClientSrv = n.serviceClient<aerostack_msgs::ControlMode>("/" + drone_id_namespace + "/"+set_control_mode_service_name);
  activate_behavior="activate_behavior";
  behavior.name = "SELF_LOCALIZE_BY_ODOMETRY";
  msg2.behavior = behavior;
  n.param<std::string>("initiate_behaviors", initiate_behaviors, "initiate_behaviors");
  initiate_behaviors_srv=n.serviceClient<droneMsgsROS::InitiateBehaviors>("/"+drone_id_namespace+"/"+initiate_behaviors);
  activate_behavior_srv=n.serviceClient<aerostack_msgs::BehaviorSrv>("/"+drone_id_namespace+"/"+activate_behavior);
  initiate_behaviors_srv.call(msg);
  activate_behavior_srv.call(msg2,res);

  //LOOP
  ros::Rate loop_rate(FREQ_INTERFACE);
  while (ros::ok()){
    // Read messages
    ros::spinOnce();

    move(11,0);
    printw("Last key pressed: ");

    //Read command
    command = getch();
    switch (command){
      case 't':  // Take off
        takeOff();
        printw("Take off");
        clrtoeol();
        move(13, 0); 
        printw("          Action: Executes the takeoff");
        clrtoeol();refresh();
        break;
      case 'y':  // Land
        hover();
        land();
        printw("Land");
        clrtoeol(); 
        move(13, 0); 
        printw("          Action: Stops the vehicle, proceeds the landing");
        clrtoeol(); refresh();              
        break;
      case ' ':  // Emergency stop
        emergencyStop();
        printw("Emergency stop");
        clrtoeol(); 
        move(13, 0); 
        printw("          Action: Forces a complete stop of the vehicle and the quadrotor controller without a proper landing");
        clrtoeol();refresh();   
        break;
      case 'h':  // Hover
        hover();
        printw("Hover");
        clrtoeol();
        move(13, 0); 
        printw("          Action: Proceeds to remain the vehicle in the current place");
        clrtoeol(); refresh();             
        break;
      case 'm':  // Move
        move();
        printw("Move");
        clrtoeol();
        move(13, 0); 
        printw("          Action: Executes the movement");
        clrtoeol(); refresh();      
        break;
      case 'q':  // Move upwards
        move();
        switch(control_mode_msg.command){
          case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
            printw("Control mode: trajectory. Command ignored");
            clrtoeol();
          break;
          case Controller_MidLevel_controlMode::POSITION_CONTROL:
            printw("Control mode: pose. Command ignored");
            clrtoeol();          
          break;
          case Controller_MidLevel_controlMode::SPEED_CONTROL:
            speed_reference_msg.twist.linear.x = current_speed_ref.twist.linear.x;
            speed_reference_msg.twist.linear.y = current_speed_ref.twist.linear.y;
            speed_reference_msg.twist.linear.z = current_speed_ref.twist.linear.z + CONTROLLER_STEP_COMMAND_ALTITUDE;
            speed_reference_msg.twist.angular = current_speed_ref.twist.angular;                                    
            publishSpeedReference();
            printw("Move upwards");
            clrtoeol();
            move(13, 0); 
            printw("          Action: Adds %.2f dz speed to the vehicle",CONTROLLER_STEP_COMMAND_ALTITUDE);
            clrtoeol(); refresh();
          break;
          case Controller_MidLevel_controlMode::UNKNOWN_CONTROL_MODE:
          default:
            hover();
            printw("Control mode: unknown. Hover");
            clrtoeol();
          break;
        }
        break;
      case 'a':  //Move downwards
        move();
        switch(control_mode_msg.command){
          case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
            printw("Control mode: trajectory. Command ignored");
            clrtoeol();
          break;
          case Controller_MidLevel_controlMode::POSITION_CONTROL:
              printw("Control mode: pose. Command ignored");
              clrtoeol();          
          break;
          case Controller_MidLevel_controlMode::SPEED_CONTROL:
            speed_reference_msg.twist.linear.x = current_speed_ref.twist.linear.x;
            speed_reference_msg.twist.linear.y = current_speed_ref.twist.linear.y;
            speed_reference_msg.twist.linear.z = current_speed_ref.twist.linear.z - CONTROLLER_STEP_COMMAND_ALTITUDE;
            speed_reference_msg.twist.angular = current_speed_ref.twist.angular;                                    
            publishSpeedReference();
            printw("Move downwards");
            clrtoeol();
            move(13, 0); 
            printw("          Action: Subtracts %.2f dz speed to the vehicle",CONTROLLER_STEP_COMMAND_ALTITUDE);
            clrtoeol();refresh();            
          break;
          case Controller_MidLevel_controlMode::UNKNOWN_CONTROL_MODE:
          default:
            hover();
            printw("Control mode: unknown. Hover");
            clrtoeol();
          break;
        }
        break;
      case 'z':  //(yaw) turn counter-clockwise
        move();
        
        switch(control_mode_msg.command){
          case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
            printw("Control mode: trajectory. Command ignored");
            clrtoeol();
          break;
          case Controller_MidLevel_controlMode::POSITION_CONTROL:
              printw("Control mode: pose. Command ignored");
              clrtoeol();          
          break;
          case Controller_MidLevel_controlMode::SPEED_CONTROL:
            speed_reference_msg.twist.linear= current_speed_ref.twist.linear;
            speed_reference_msg.twist.angular.x = current_speed_ref.twist.angular.x;          
            speed_reference_msg.twist.angular.y = current_speed_ref.twist.angular.x;   
            speed_reference_msg.twist.angular.z = current_speed_ref.twist.angular.z - CONTROLLER_STEP_COMMAND_YAW;                             
            publishSpeedReference();
            printw("Turn counter-clockwise");
            clrtoeol();
            move(13, 0); 
            printw("          Action: Subtracts %.2f dyaw speed to the vehicle",CONTROLLER_STEP_COMMAND_YAW);
            clrtoeol(); refresh();             
          break;
          case Controller_MidLevel_controlMode::UNKNOWN_CONTROL_MODE:
          default:
            hover();
            printw("Control mode: unknown. Hover");
            clrtoeol();
          break;
        }

        break;
      case 'x':  // (yaw) turn clockwise
        move();
        
        switch(control_mode_msg.command){
          case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
            printw("Control mode: trajectory. Command ignored");
            clrtoeol();
          break;
          case Controller_MidLevel_controlMode::POSITION_CONTROL:
              printw("Control mode: pose. Command ignored");
              clrtoeol();          
          break;
          case Controller_MidLevel_controlMode::SPEED_CONTROL:
            speed_reference_msg.twist.linear = current_speed_ref.twist.linear;
            speed_reference_msg.twist.angular.x = current_speed_ref.twist.angular.x;          
            speed_reference_msg.twist.angular.y = current_speed_ref.twist.angular.x;   
            speed_reference_msg.twist.angular.z = current_speed_ref.twist.angular.z + CONTROLLER_STEP_COMMAND_YAW;                             
            publishSpeedReference();
            printw("Turn clockwise");
            clrtoeol();
            move(13, 0); 
            printw("          Action: Adds %.2f dyaw speed to the vehicle",CONTROLLER_STEP_COMMAND_YAW);
            clrtoeol();   refresh();            
          break;
          case Controller_MidLevel_controlMode::UNKNOWN_CONTROL_MODE:
          default:
            hover();
            printw("Control mode: unknown. Hover");
            clrtoeol();
          break;
        }      
        break;
      case 'c':  // Set yaw reference to 0
        switch(control_mode_msg.command){
          case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
            printw("Control mode: trajectory. Command ignored");
            clrtoeol();
          break;
          case Controller_MidLevel_controlMode::POSITION_CONTROL:
            printw("Control mode: trajectory. Command ignored");
            clrtoeol();          
          break;
          case Controller_MidLevel_controlMode::SPEED_CONTROL:
            speed_reference_msg.twist.linear = current_speed_ref.twist.linear;
            speed_reference_msg.twist.angular.x = current_speed_ref.twist.angular.x;          
            speed_reference_msg.twist.angular.y = current_speed_ref.twist.angular.x;   
            speed_reference_msg.twist.angular.z = 0.0;                             
            publishSpeedReference();
            printw("Set yaw reference to 0");
            clrtoeol();
            move(13, 0); 
            printw("          Action: Sets yaw reference to 0. Parallel to x-axis");
            clrtoeol();  refresh();          
          break;
          case Controller_MidLevel_controlMode::UNKNOWN_CONTROL_MODE:
          default:
            hover();
            printw("Control mode: unknown. Hover");
            clrtoeol();
          break;
        }  
        break;
      case ASCII_KEY_RIGHT:
        move();
        switch(control_mode_msg.command){
          case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
            printw("Control mode: trajectory. Command ignored");
            clrtoeol();
          break;
          case Controller_MidLevel_controlMode::POSITION_CONTROL:
            printw("Control mode: pose. Command ignored");
            clrtoeol();          
          break;
          case Controller_MidLevel_controlMode::SPEED_CONTROL:
            speed_reference_msg.twist.linear.x = current_speed_ref.twist.linear.x;
            speed_reference_msg.twist.linear.y = current_speed_ref.twist.linear.y - CONTROLLER_CTE_COMMAND_SPEED;
            speed_reference_msg.twist.linear.z = current_speed_ref.twist.linear.z;
            speed_reference_msg.twist.angular = current_speed_ref.twist.angular;                         
            publishSpeedReference();
            printw("Key right");
            clrtoeol();
            move(13, 0); 
            printw("          Action: Subtracts %.2f dy speed to the vehicle",CONTROLLER_CTE_COMMAND_SPEED);
            clrtoeol();refresh();                  
          break;
          case Controller_MidLevel_controlMode::UNKNOWN_CONTROL_MODE:
          default:
            hover();
            printw("Control mode: unknown. Hover");
            clrtoeol();
          break;
        }
        break;
      case ASCII_KEY_LEFT:
        move();
        switch(control_mode_msg.command){
          case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
            printw("Control mode: trajectory. Command ignored");
            clrtoeol();
          break;
          case Controller_MidLevel_controlMode::POSITION_CONTROL:
            printw("Control mode: pose. Command ignored");
            clrtoeol();                
          break;
          case Controller_MidLevel_controlMode::SPEED_CONTROL:
            speed_reference_msg.twist.linear.x = current_speed_ref.twist.linear.x;
            speed_reference_msg.twist.linear.y = current_speed_ref.twist.linear.y + CONTROLLER_CTE_COMMAND_SPEED;
            speed_reference_msg.twist.linear.z = current_speed_ref.twist.linear.z;
            speed_reference_msg.twist.angular = current_speed_ref.twist.angular;                         
            publishSpeedReference();
            printw("Key left");
            clrtoeol();
            move(13, 0); 
            printw("          Action: Adds %.2f dy speed to the vehicle",CONTROLLER_CTE_COMMAND_SPEED);
            clrtoeol();     refresh();       
          break;
          case Controller_MidLevel_controlMode::UNKNOWN_CONTROL_MODE:
          default:
            hover();
            printw("Control mode: unknown. Hover");
            clrtoeol();
          break;
        }
        break;
      case ASCII_KEY_DOWN:
        move();
        switch(control_mode_msg.command){
          case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
            printw("Control mode: trajectory. Command ignored");
            clrtoeol();
          break;
          case Controller_MidLevel_controlMode::POSITION_CONTROL:
            printw("Control mode: pose. Command ignored");
            clrtoeol();                
          break;
          case Controller_MidLevel_controlMode::SPEED_CONTROL:
            speed_reference_msg.twist.linear.x = current_speed_ref.twist.linear.x - CONTROLLER_CTE_COMMAND_SPEED;
            speed_reference_msg.twist.linear.y = current_speed_ref.twist.linear.y; 
            speed_reference_msg.twist.linear.z = current_speed_ref.twist.linear.z;
            speed_reference_msg.twist.angular = current_speed_ref.twist.angular;                         
            publishSpeedReference();
            printw("Key down");
            clrtoeol();
            move(13, 0); 
            printw("          Action: Subtracts %.2f dx speed to the vehicle",CONTROLLER_CTE_COMMAND_SPEED);
            clrtoeol();  refresh();          
          break;
          case Controller_MidLevel_controlMode::UNKNOWN_CONTROL_MODE:
          default:
            hover();
            printw("Control mode: unknown. Hover");
            clrtoeol();
          break;
        }

        break;
      case ASCII_KEY_UP:
        move();
        switch(control_mode_msg.command){
          case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
            printw("Control mode: trajectory. Command ignored");
            clrtoeol();
          break;
          case Controller_MidLevel_controlMode::POSITION_CONTROL:
            printw("Control mode: pose. Command ignored");
            clrtoeol();                
          break;
          case Controller_MidLevel_controlMode::SPEED_CONTROL:
            speed_reference_msg.twist.linear.x = current_speed_ref.twist.linear.x + CONTROLLER_CTE_COMMAND_SPEED;
            speed_reference_msg.twist.linear.y = current_speed_ref.twist.linear.y; 
            speed_reference_msg.twist.linear.z = current_speed_ref.twist.linear.z;
            speed_reference_msg.twist.angular = current_speed_ref.twist.angular;                         
            publishSpeedReference();
            printw("Key up");
            clrtoeol();
            move(13, 0); 
            printw("          Action: Adds %.2f dx speed to the vehicle",CONTROLLER_CTE_COMMAND_SPEED);
            clrtoeol(); refresh();           
          break;
          case Controller_MidLevel_controlMode::UNKNOWN_CONTROL_MODE:
          default:
            hover();
            printw("Control mode: unknown. Hover");
            clrtoeol();
          break;
        }
      break;
    case 'r':
    {
      printw("Reset orientation (Parallel to x-axis)");
      clrtoeol();
      move(13, 0); 
      printw("          Action: Sets orientation to 0 in pose mode. Proceeds to change to speed mode.");
      clrtoeol(); 
      refresh();      

      //Sets speed references to 0
      speed_reference_msg.twist.linear.x = 0.0;
      speed_reference_msg.twist.linear.y = 0.0;
      speed_reference_msg.twist.linear.z = 0.0;
      speed_reference_msg.twist.angular.x = 0.0;
      speed_reference_msg.twist.angular.y = 0.0;
      speed_reference_msg.twist.angular.z = 0.0;                                
      publishSpeedReference();      

      //Change pose mdoe
      if (setControlMode(Controller_MidLevel_controlMode::POSITION_CONTROL)){
       while (control_mode_msg.command != Controller_MidLevel_controlMode::POSITION_CONTROL ){
          //Wait for delay
       }
        //Send current location + orientation to zero
        geometry_msgs::PoseStamped pose_reference_aux;
        pose_reference_aux.pose.position = self_localization_pose_msg.pose.position;
        pose_reference_aux.pose.orientation.w = 0.0;
        pose_reference_aux.pose.orientation.x = 0.0;
        pose_reference_aux.pose.orientation.y = 0.0;
        pose_reference_aux.pose.orientation.z = 0.0;
        pose_reference_publ.publish(pose_reference_aux);

        //Sleep 10 seconds (waiting for yaw = 0)
        sleep(10);

        //Change to speed mode
        if (setControlMode(Controller_MidLevel_controlMode::SPEED_CONTROL)){
          while (control_mode_msg.command != Controller_MidLevel_controlMode::SPEED_CONTROL ){
            //Wait for delay
          }

        //Pose references to zero (reset pose references)
        pose_reference_aux.pose.position.x = 0;
        pose_reference_aux.pose.position.y = 0;
        pose_reference_aux.pose.position.z = 0;       
        pose_reference_aux.pose.orientation.w = 0.0;
        pose_reference_aux.pose.orientation.x = 0.0;
        pose_reference_aux.pose.orientation.y = 0.0;
        pose_reference_aux.pose.orientation.z = 0.0;
        pose_reference_publ.publish(pose_reference_aux); 
        } else{
        printw("ERROR");
        clrtoeol(); 
        refresh();     
      }
      }else{
        printw("ERROR");
        clrtoeol(); 
        refresh();         
      }
    }
    break;
    }

    refresh();
    loop_rate.sleep();
  }

  endwin();

  printf("Keyboard Teleoperation ended..\n");

  return 0;
}

//Print stringstream
void printout_stream(std::stringstream* pinterface_printout_stream, int* lineCommands, int* columCommands)
{
  std::string line;
  move((*lineCommands), (*columCommands));
  while (std::getline(*pinterface_printout_stream, line, '\n'))
  {
    for (int i = line.size(); i < DISPLAY_COLUMN_SIZE; i++)
      line += " ";
    printw(line.c_str());
    move(++(*lineCommands), (*columCommands));
  }
}

//Controls
void printoutControls(){
  printw("                   ----KEYBOARD TELEOPERATION INTERFACE----");
  move(2,0);
  printw("Controls");
  move(3,0);
  attron(COLOR_PAIR(2));printw("  t:");attroff(COLOR_PAIR(2)); printw("        Take off                  ");  
  attron(COLOR_PAIR(2));printw("  key right:");attroff(COLOR_PAIR(2));printw(" Move right");
  move(4,0);
  attron(COLOR_PAIR(2));printw("  y:");attroff(COLOR_PAIR(2));printw("        Land                      ");
  attron(COLOR_PAIR(2));printw("  key left:");attroff(COLOR_PAIR(2));printw("  Move left   ");
  move(5,0);
  attron(COLOR_PAIR(2));printw("  space:   ");attroff(COLOR_PAIR(2));printw(" Emergency stop            ");
  attron(COLOR_PAIR(2));printw("  key down:");attroff(COLOR_PAIR(2));printw("  Move backwards        ");
  move(6,0);
  attron(COLOR_PAIR(2));printw("  h:");attroff(COLOR_PAIR(2));printw("        Hover                     ");
  attron(COLOR_PAIR(2));printw("  key up:");attroff(COLOR_PAIR(2));printw("    Move frontwards    ");
  move(7,0);
  attron(COLOR_PAIR(2));printw("  m:");attroff(COLOR_PAIR(2));printw("        Move                      ");
  attron(COLOR_PAIR(2));printw("  r:");attroff(COLOR_PAIR(2));printw("         Reset orientation    ");  
  move(8,0);
  attron(COLOR_PAIR(2));printw("  q:");attroff(COLOR_PAIR(2));printw("        Move upwards              ");
  attron(COLOR_PAIR(2));printw("  a:");attroff(COLOR_PAIR(2));printw("         Move downwards            ");
  move(9,0);
  attron(COLOR_PAIR(2));printw("  z:");attroff(COLOR_PAIR(2));printw("        Turn counter-clockwise    ");
  attron(COLOR_PAIR(2));printw("  x:");attroff(COLOR_PAIR(2));printw("         Turn clockwise            ");
  refresh();
}

//High level command
void publishCmd(){
  command_publ.publish(command_order);
}

//Take off
void takeOff(){
  clearCmd();
  command_order.command = droneMsgsROS::droneCommand::TAKE_OFF;
  publishCmd();
}

//Hover
void hover(){
  clearCmd();
  command_order.command = droneMsgsROS::droneCommand::HOVER;
  publishCmd();
}

//Land
void land(){
  clearCmd();
  command_order.command = droneMsgsROS::droneCommand::LAND;
  publishCmd();
}

//Emergency Stop
void emergencyStop(){
  clearCmd();
  command_order.command = droneMsgsROS::droneCommand::RESET;
  publishCmd();
}

//Move
void move(){
  command_order.command = droneMsgsROS::droneCommand::MOVE;
  publishCmd();
}

//Clear commands
void clearCmd(){
  geometry_msgs::Vector3 thrust;
  thrust.x = 0.0;
  thrust.y = 0.0;
  thrust.z = 0.0;
  multirotor_command_msg.roll = 0.0;
  multirotor_command_msg.pitch = 0.0;
  multirotor_command_msg.yaw_rate = 0.0;
  multirotor_command_msg.thrust = thrust;
}

//Publish multirotor Command
void publishMultirotorCommand(){
  multirotor_command_publ.publish(multirotor_command_msg);
}

//Publish Speed Reference
void publishSpeedReference(){
  speed_reference_publ.publish(speed_reference_msg);
}

//Speed reference callback
void speedReferenceCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
  current_speed_ref = (*msg);
}

//Control mode callback
void controlModeCallback(const aerostack_msgs::FlightMotionControlMode::ConstPtr& msg){
  control_mode_msg = (*msg);
}

//Set control mode
bool setControlMode(Controller_MidLevel_controlMode::controlMode new_control_mode){
  // Prepare service message
  aerostack_msgs::ControlMode setControlModeSrv;
  setControlModeSrv.request.controlMode.command = new_control_mode;
  // use service
  if (setControlModeClientSrv.call(setControlModeSrv)){
    return setControlModeSrv.response.ack;
  }else{
    return false;
  }
}

//Self localization callback
void selfLocalizationPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  self_localization_pose_msg = (*msg);
}

