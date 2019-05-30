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
  ros::NodeHandle n("~");
  n.param<std::string>("drone_id_namespace", drone_id_namespace, "drone1");
  std::thread thr(&spinnerThread);
  printf("Starting Keyboard Teleoperation Interface...\n");

  ros::param::get("~command_high_level_topic_name", command_high_level_topic_name);
    if ( command_high_level_topic_name.length() == 0)
    {
        command_high_level_topic_name="command/high_level";
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
  setlocale(LC_ALL, "");
  initscr();
  start_color();
  use_default_colors();  
  curs_set(0);
  noecho();
  nodelay(stdscr, TRUE);
  erase();
  refresh();
  init_pair(1, COLOR_BLUE, -1);
  init_pair(2, COLOR_GREEN, -1);
  init_pair(3, COLOR_CYAN, -1);
  init_pair(4, COLOR_RED, -1);
  init_pair(5, COLOR_YELLOW, -1);


  advanced = false;

  //Input variable
  char command = 0;

  //Publishers
  command_publ = n.advertise<droneMsgsROS::droneCommand>("/" + drone_id_namespace + "/"+ command_high_level_topic_name, 1, true);
  speed_reference_publ = n.advertise<geometry_msgs::TwistStamped>("/"+drone_id_namespace+"/"+speed_ref_topic_name, 1, true);
  pose_reference_publ = n.advertise<geometry_msgs::PoseStamped>("/"+drone_id_namespace+"/"+pose_ref_topic_name, 1, true);

  //Subscribers
  self_pose_sub = n.subscribe("/"+drone_id_namespace+"/"+self_pose_topic_name, 1, selfLocalizationPoseCallback);
  control_mode = n.subscribe("/" + drone_id_namespace + "/" + assumed_control_mode_topic_name, 1, controlModeCallback);
  speed_reference_sub = n.subscribe("/"+drone_id_namespace+"/"+speed_ref_topic_name, 1, speedReferenceCallback);

  //Wait 3sec for initialization
  sleep(3);


  current_mode = GROUND_SPEED;

  //Services
  startQuadrotorControllerClientSrv=n.serviceClient<std_srvs::Empty>("/"+drone_id_namespace+"/quadrotor_pid_controller_process/start");
  startQuadrotorControllerClientSrv.call(req);
  setControlModeClientSrv = n.serviceClient<aerostack_msgs::SetControlMode>("/" + drone_id_namespace + "/"+set_control_mode_service_name);
  behavior.name = "SELF_LOCALIZE_BY_ODOMETRY";
  msg2.behavior = behavior;
  initiate_behaviors_srv=n.serviceClient<droneMsgsROS::InitiateBehaviors>("/"+drone_id_namespace+"/initiate_behaviors");
  activate_behavior_srv=n.serviceClient<aerostack_msgs::RequestBehavior>("/"+drone_id_namespace+"/activate_behavior");
  initiate_behaviors_srv.call(msg);
  activate_behavior_srv.call(msg2,res);
  move(0,0);clrtoeol();
  printw("                       -KEYBOARD TELEOPERATION INTERFACE-");
  move(3,0);clrtoeol();
  printw("--------------------------------------------------------------------------------");
  //Print controls
  //printoutPoseControls();
  printoutGroundSpeedControls();
  setControlMode(aerostack_msgs::QuadrotorPidControllerMode::GROUND_SPEED);
  //LOOP
  ros::Rate loop_rate(FREQ_INTERFACE);

  while (ros::ok()){
    // Read messages
    ros::spinOnce();

    //Print control mode
    switch(current_mode){
          case POSE:
            move(2, 0);
            attron(COLOR_PAIR(2));printw("                        Teleoperation mode: Pose      "); attroff(COLOR_PAIR(2));
            clrtoeol(); refresh();  
          break;
          case ATTITUDE:
            move(2, 0);
            attron(COLOR_PAIR(3));printw("                           Teleoperation mode: Attitude      "); attroff(COLOR_PAIR(3));
            clrtoeol(); refresh();  
          break;
          case GROUND_SPEED:
            move(2, 0);
            attron(COLOR_PAIR(4));printw("                        Teleoperation mode: Ground speed         "); attroff(COLOR_PAIR(4));
            clrtoeol(); refresh();  
          break;
          default:
            move(2, 0);
            printw("                           Teleoperation mode: Unknown          ");
            clrtoeol(); refresh();   
          break;      
    }

    move(16,0);
    printw("                        Last key pressed: ");
    //Read command
    command = getch();
    switch (command){
      case 't':  // Take off
        takeOff();
        printw("t       ");
        clrtoeol();
                  motion_reference_pose_msg.pose.position.z = 1;
          pose_reference_publ.publish(motion_reference_pose_msg);
        move(17, 0); 
        //printw("             Action: Executes the takeoff");
        printw("                        Last command:     Take off           ");
        clrtoeol();refresh();
        break;
      case 'y':  // Land
        hover();
        land();
        printw("y        ");
        clrtoeol(); 
        move(17, 0); 
        //printw("             Action: Stops the vehicle, proceeds the landing");
        printw("                        Last command:     Land             ");
        clrtoeol(); refresh();              
        break;
      case ' ':  // Emergency stop 
        emergencyStop();
        printw("space     ");
        clrtoeol(); 
        move(17, 0); 
        //printw("             Action: Executes an emergency stop");
        printw("                        Last command:     Emergency stop            ");
        clrtoeol();refresh();   
        break;
      case 'h':  // Hover   
        hover();
        printw("h      ");
        if (current_mode == POSE){
          motion_reference_pose_msg.pose= self_localization_pose_msg.pose;
          pose_reference_publ.publish(motion_reference_pose_msg);
        }

        if (current_mode == GROUND_SPEED){
            clearSpeedReferences();
            publishSpeedReference();
        }
        clrtoeol();
        move(17, 0); 
        //printw("             Action: Proceeds to remain the vehicle in the current place");
        printw("                        Last command:     Hovering             ");
        clrtoeol(); refresh();             
        break;
      case 'q':  // Move upwards
        move();
        if (current_mode == GROUND_SPEED || current_mode == POSE){
          motion_reference_pose_msg.pose.position.z = motion_reference_pose_msg.pose.position.z + CONTROLLER_STEP_COMMAND_ALTITUDE;
          motion_reference_pose_msg.pose.orientation = motion_reference_pose_msg.pose.orientation;
          pose_reference_publ.publish(motion_reference_pose_msg);
        }

        if (current_mode == ATTITUDE){
          
        }
        speed_reference_msg.twist.linear.x = current_speed_ref.twist.linear.x;
        speed_reference_msg.twist.linear.y = current_speed_ref.twist.linear.y;
        speed_reference_msg.twist.linear.z = current_speed_ref.twist.linear.z + CONTROLLER_STEP_COMMAND_ALTITUDE;
        speed_reference_msg.twist.angular = current_speed_ref.twist.angular;                                    
        publishSpeedReference();
        printw("q      ");
        clrtoeol();
        move(17, 0); 
        //printw("             Action: Adds %.2f dz speed to the vehicle",CONTROLLER_STEP_COMMAND_ALTITUDE);
        printw("                        Last command:     Increase altitude         ");
        clrtoeol(); refresh();
        break;
      case 'a':  //Move downwards
        move();
        if (current_mode == GROUND_SPEED || current_mode == POSE){
          motion_reference_pose_msg.pose.position.z = motion_reference_pose_msg.pose.position.z - CONTROLLER_STEP_COMMAND_ALTITUDE;
          motion_reference_pose_msg.pose.orientation = motion_reference_pose_msg.pose.orientation;
          pose_reference_publ.publish(motion_reference_pose_msg);
        }

        if (current_mode == ATTITUDE){
          
        }
        printw("a        ");
        clrtoeol();
        move(17, 0); 
        //printw("             Action: Subtracts %.2f dz speed to the vehicle",CONTROLLER_STEP_COMMAND_ALTITUDE);
        printw("                        Last command:     Decrease altitude         ");
        clrtoeol();refresh();   
        break;         
      case 'z':  //(yaw) turn counter-clockwise
      {        
        move();       
        if (current_mode == GROUND_SPEED || current_mode == POSE){
          tf2::Quaternion q_rot;
          double r=0, p=0, yaw = 0;
          if(motion_reference_pose_msg.pose.orientation.w == 0 && motion_reference_pose_msg.pose.orientation.x == 0 && motion_reference_pose_msg.pose.orientation.y == 0 && motion_reference_pose_msg.pose.orientation.z == 0){
            yaw = 0;
          }else{
            yaw   = atan2(2.0 * (motion_reference_pose_msg.pose.orientation.z * motion_reference_pose_msg.pose.orientation.w + motion_reference_pose_msg.pose.orientation.x * motion_reference_pose_msg.pose.orientation.y) ,
           - 1.0 + 2.0 * (motion_reference_pose_msg.pose.orientation.w * motion_reference_pose_msg.pose.orientation.w + motion_reference_pose_msg.pose.orientation.x * motion_reference_pose_msg.pose.orientation.x));
          }
          
          yaw = yaw + CONTROLLER_STEP_COMMAND_YAW;
          q_rot.setRPY(r, p, yaw);
          motion_reference_pose_msg.pose.orientation.w = q_rot.getW();
          motion_reference_pose_msg.pose.orientation.x = q_rot.getX();
          motion_reference_pose_msg.pose.orientation.y = q_rot.getY();
          motion_reference_pose_msg.pose.orientation.z = q_rot.getZ();
          motion_reference_pose_msg.pose.position = motion_reference_pose_msg.pose.position;
          pose_reference_publ.publish(motion_reference_pose_msg);
        }
        printw("z       ");
        clrtoeol();
        move(17, 0); 
        //printw("             Action: Subtracts %.2f dyaw speed to the vehicle",CONTROLLER_STEP_COMMAND_YAW);
        printw("                        Last command:     Turn counter-clockwise        ");
        clrtoeol(); refresh();   
        break;    
      }      
      case 'x':  // (yaw) turn clockwise
      {      
        move();       
        if (current_mode == GROUND_SPEED || current_mode == POSE){
          tf2::Quaternion q_rot;
          double r=0, p=0, yaw = 0;
          if(motion_reference_pose_msg.pose.orientation.w == 0 && motion_reference_pose_msg.pose.orientation.x == 0 && motion_reference_pose_msg.pose.orientation.y == 0 && motion_reference_pose_msg.pose.orientation.z == 0){
            yaw = 0;
          }else{
            yaw   = atan2(2.0 * (motion_reference_pose_msg.pose.orientation.z * motion_reference_pose_msg.pose.orientation.w + motion_reference_pose_msg.pose.orientation.x * motion_reference_pose_msg.pose.orientation.y) ,
           - 1.0 + 2.0 * (motion_reference_pose_msg.pose.orientation.w * motion_reference_pose_msg.pose.orientation.w + motion_reference_pose_msg.pose.orientation.x * motion_reference_pose_msg.pose.orientation.x));
          }
          
          yaw = yaw - CONTROLLER_STEP_COMMAND_YAW;
          q_rot.setRPY(r, p, yaw);
          motion_reference_pose_msg.pose.orientation.w = q_rot.getW();
          motion_reference_pose_msg.pose.orientation.x = q_rot.getX();
          motion_reference_pose_msg.pose.orientation.y = q_rot.getY();
          motion_reference_pose_msg.pose.orientation.z = q_rot.getZ();
          motion_reference_pose_msg.pose.position = motion_reference_pose_msg.pose.position;
          pose_reference_publ.publish(motion_reference_pose_msg);
        }
        printw("x      ");
        clrtoeol();
        move(17, 0); 
        //printw("             Action: Adds %.2f dyaw speed to the vehicle",CONTROLLER_STEP_COMMAND_YAW);
        printw("                        Last command:     Turn clockwise          ");
        clrtoeol();   refresh();
      }
        break;                 
      case ASCII_KEY_RIGHT:
        move();
        if (current_mode == GROUND_SPEED){
          speed_reference_msg.twist.linear.x = current_speed_ref.twist.linear.x;
          speed_reference_msg.twist.linear.y = current_speed_ref.twist.linear.y - CONTROLLER_CTE_COMMAND_SPEED;                       
          publishSpeedReference();
        }
        if (current_mode == POSE){
          motion_reference_pose_msg.pose.orientation = motion_reference_pose_msg.pose.orientation;
          motion_reference_pose_msg.pose.position.x = motion_reference_pose_msg.pose.position.x;
          motion_reference_pose_msg.pose.position.y = motion_reference_pose_msg.pose.position.y - CONTROLLER_CTE_COMMAND_POSE;
          motion_reference_pose_msg.pose.position.z = motion_reference_pose_msg.pose.position.z;
          pose_reference_publ.publish(motion_reference_pose_msg);
        }

        printw("\u2192            ");
        clrtoeol();
        move(17, 0); 
        //printw("             Action: Subtracts %.2f dy speed to the vehicle",CONTROLLER_CTE_COMMAND_SPEED);
        printw("                        Last command:     Increase movement to the right        ");
        clrtoeol();refresh();   
        break;               
      case ASCII_KEY_LEFT:
        move();
        if (current_mode == GROUND_SPEED){
          speed_reference_msg.twist.linear.x = current_speed_ref.twist.linear.x;
          speed_reference_msg.twist.linear.y = current_speed_ref.twist.linear.y + CONTROLLER_CTE_COMMAND_SPEED;                       
          publishSpeedReference();
        }
        if (current_mode == POSE){
          motion_reference_pose_msg.pose.orientation = motion_reference_pose_msg.pose.orientation;
          motion_reference_pose_msg.pose.position.x = motion_reference_pose_msg.pose.position.x;
          motion_reference_pose_msg.pose.position.y = motion_reference_pose_msg.pose.position.y + CONTROLLER_CTE_COMMAND_POSE;
          motion_reference_pose_msg.pose.position.z = motion_reference_pose_msg.pose.position.z;
          pose_reference_publ.publish(motion_reference_pose_msg);
        }
        printw("\u2190            ");
        clrtoeol();
        move(17, 0); 
        //printw("             Action: Adds %.2f dy speed to the vehicle",CONTROLLER_CTE_COMMAND_SPEED);
        printw("                        Last command:     Increase movement to the left         ");
        clrtoeol();     refresh();
        break;       
      case ASCII_KEY_DOWN:
        move();
        if (current_mode == GROUND_SPEED){
          speed_reference_msg.twist.linear.x = current_speed_ref.twist.linear.x - CONTROLLER_CTE_COMMAND_SPEED;
          speed_reference_msg.twist.linear.y = current_speed_ref.twist.linear.y;                       
          publishSpeedReference();
        }
        if (current_mode == POSE){
          motion_reference_pose_msg.pose.orientation = motion_reference_pose_msg.pose.orientation;
          motion_reference_pose_msg.pose.position.x = motion_reference_pose_msg.pose.position.x - CONTROLLER_CTE_COMMAND_POSE;
          motion_reference_pose_msg.pose.position.y = motion_reference_pose_msg.pose.position.y;
          motion_reference_pose_msg.pose.position.z = motion_reference_pose_msg.pose.position.z;
          pose_reference_publ.publish(motion_reference_pose_msg);
        }
        printw("\u2193     ");
        clrtoeol();
        move(17, 0); 
        //printw("             Action: Subtracts %.2f dx speed to the vehicle",CONTROLLER_CTE_COMMAND_SPEED);
        printw("                        Last command:     Increase backward      ");
        clrtoeol();  refresh();  
        break;        
      case ASCII_KEY_UP:
        move();
        if (current_mode == GROUND_SPEED){
          speed_reference_msg.twist.linear.x = current_speed_ref.twist.linear.x + CONTROLLER_CTE_COMMAND_SPEED;
          speed_reference_msg.twist.linear.y = current_speed_ref.twist.linear.y;                       
          publishSpeedReference();
        }
        if (current_mode == POSE){
          motion_reference_pose_msg.pose.orientation = motion_reference_pose_msg.pose.orientation;
          motion_reference_pose_msg.pose.position.x = motion_reference_pose_msg.pose.position.x + CONTROLLER_CTE_COMMAND_POSE;
          motion_reference_pose_msg.pose.position.y = motion_reference_pose_msg.pose.position.y;
          motion_reference_pose_msg.pose.position.z = motion_reference_pose_msg.pose.position.z;
          pose_reference_publ.publish(motion_reference_pose_msg);
        }
        printw("\u2191    ");
        clrtoeol();
        move(17, 0); 
        //printw("             Action: Adds %.2f dx speed to the vehicle",CONTROLLER_CTE_COMMAND_SPEED);
        printw("                        Last command:     Increase forward            ");
        clrtoeol(); refresh(); 
        break;         
    case 'r':
    {
      printw("r        ");
      clrtoeol();
      move(17, 0); 
      //printw("             Action: Sets orientation to 0 in SPEED 3D mode.");
      printw("                        Last command:     Reset orientation           ");
      clrtoeol(); 
      refresh();      
  
      if (current_mode == POSE){
        motion_reference_pose_msg.pose.orientation.w = 0;
        motion_reference_pose_msg.pose.orientation.x = 0;
        motion_reference_pose_msg.pose.orientation.y = 0;
        motion_reference_pose_msg.pose.orientation.z = 0;
        motion_reference_pose_msg.pose.position = motion_reference_pose_msg.pose.position;
        pose_reference_publ.publish(motion_reference_pose_msg);
      }
      if (current_mode == GROUND_SPEED){
        motion_reference_pose_msg.pose.orientation.w = 0;
        motion_reference_pose_msg.pose.orientation.x = 0;
        motion_reference_pose_msg.pose.orientation.y = 0;
        motion_reference_pose_msg.pose.orientation.z = 0;
        motion_reference_pose_msg.pose.position = motion_reference_pose_msg.pose.position;
        pose_reference_publ.publish(motion_reference_pose_msg);
      }      
    }
    break;
    case '1':
    {
      printw("1        ");
      clrtoeol();
      move(17, 0); 
      //printw("             Action: Sets orientation to 0 in SPEED 3D mode.");
      printw("                        Last command:     Ground speed           ");
      clrtoeol(); 
      refresh();          
      if (setControlMode(aerostack_msgs::QuadrotorPidControllerMode::GROUND_SPEED)){
        current_mode = GROUND_SPEED;
        printoutGroundSpeedControls();
      }

    }
    break;    
    case '2':
    {
      printw("2        ");
      clrtoeol();
      move(17, 0); 
      //printw("             Action: Sets orientation to 0 in SPEED 3D mode.");
      printw("                        Last command:     Pose          ");
      clrtoeol(); 
      refresh();          
      if (setControlMode(aerostack_msgs::QuadrotorPidControllerMode::POSE)){
         printoutPoseControls();   
         current_mode = POSE;    
      }
      

    }
    break;  
    case '3':
    {



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
void printoutPoseControls(){
  move(4,0);clrtoeol();
  printw(" Basic motions:                    Pose control");
  move(5,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   t:");attroff(COLOR_PAIR(5)); printw("      Take off              ");  
  attron(COLOR_PAIR(5));printw("   \u2191:");attroff(COLOR_PAIR(5));printw("  Increase forward position %.2f m  ",CONTROLLER_CTE_COMMAND_POSE);
  
  move(6,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   y:");attroff(COLOR_PAIR(5)); printw("      Land                  ");
  attron(COLOR_PAIR(5));printw("   \u2193:");attroff(COLOR_PAIR(5));printw("  Increase backward position %.2f m  ",CONTROLLER_CTE_COMMAND_POSE);
  
  move(7,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   h:");attroff(COLOR_PAIR(5)); printw("      Keep hovering         ");
  attron(COLOR_PAIR(5));printw("   \u2192:");attroff(COLOR_PAIR(5));printw("  Increase position to the right %.2f m  ",CONTROLLER_CTE_COMMAND_POSE);  

  move(8,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   space: ");attroff(COLOR_PAIR(5));printw(" Emergency stop        ");
  attron(COLOR_PAIR(5));printw("   \u2190:");attroff(COLOR_PAIR(5));printw("  Increase position to the left %.2f m  ",CONTROLLER_CTE_COMMAND_POSE);
  
  move(9,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   r:");attroff(COLOR_PAIR(5));printw("      Reset orientation    ");  
  
  move(10,0);clrtoeol();
  printw("                                   ");
  attron(COLOR_PAIR(5));printw(" q:");attroff(COLOR_PAIR(5));printw("  Increase altitude %.2f m           ",CONTROLLER_STEP_COMMAND_ALTITUDE);
  move(11,0);clrtoeol();
  printw(" Teleoperation mode selection:     ");
  attron(COLOR_PAIR(5));printw(" a:");attroff(COLOR_PAIR(5));printw("  Decrease altitude %.2f m            ",CONTROLLER_STEP_COMMAND_ALTITUDE);
  move(12,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   1:");attroff(COLOR_PAIR(5));attron(COLOR_PAIR(4));printw("      Ground speed    ");attroff(COLOR_PAIR(4));
  
  move(13,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   2:");attroff(COLOR_PAIR(5));attron(COLOR_PAIR(2));printw("      Pose                 ");attroff(COLOR_PAIR(2));  
  attron(COLOR_PAIR(5));printw("    z:");attroff(COLOR_PAIR(5));printw("  Turn counter-clockwise %.2f rad      ",CONTROLLER_STEP_COMMAND_YAW);
  
  move(14,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   3:");attroff(COLOR_PAIR(5));attron(COLOR_PAIR(3));printw("      Attitude              "); attroff(COLOR_PAIR(3)); 
  attron(COLOR_PAIR(5));printw("   x:");attroff(COLOR_PAIR(5));printw("  Turn clockwise %.2f rad        ",CONTROLLER_STEP_COMMAND_YAW);
  move(15,0);clrtoeol();
  printw("--------------------------------------------------------------------------------");
  refresh();
}

//Advanced Controls
void printoutGroundSpeedControls(){
  move(4,0);clrtoeol();
  printw(" Basic motions:                    Ground speed control:");
  move(5,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   t:");attroff(COLOR_PAIR(5)); printw("      Take off              ");  
  attron(COLOR_PAIR(5));printw("   \u2191:");attroff(COLOR_PAIR(5));printw("  Increase forward speed %.2f m/s  ",CONTROLLER_CTE_COMMAND_SPEED);
  
  move(6,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   y:");attroff(COLOR_PAIR(5)); printw("      Land                  ");
  attron(COLOR_PAIR(5));printw("   \u2193:");attroff(COLOR_PAIR(5));printw("  Increase backward speed %.2f m/s  ",CONTROLLER_CTE_COMMAND_SPEED);
  
  move(7,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   h:");attroff(COLOR_PAIR(5)); printw("      Keep hovering         ");
  attron(COLOR_PAIR(5));printw("   \u2192:");attroff(COLOR_PAIR(5));printw("  Increase speed to the right %.2f m/s  ",CONTROLLER_CTE_COMMAND_SPEED);  

  move(8,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   space: ");attroff(COLOR_PAIR(5));printw(" Emergency stop        ");
  attron(COLOR_PAIR(5));printw("   \u2190:");attroff(COLOR_PAIR(5));printw("  Increase speed to the left %.2f m/s  ",CONTROLLER_CTE_COMMAND_SPEED);
  
  move(9,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   r:");attroff(COLOR_PAIR(5));printw("      Reset orientation    ");  
  
  move(10,0);clrtoeol();
  printw("                                   Pose control:");
  
  move(11,0);clrtoeol();
  printw(" Teleoperation mode selection:     ");
  attron(COLOR_PAIR(5));printw(" q:");attroff(COLOR_PAIR(5));printw("  Move upwards              ");
  
  move(12,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   1:");attroff(COLOR_PAIR(5));attron(COLOR_PAIR(4));printw("      Ground speed    ");attroff(COLOR_PAIR(4));
  attron(COLOR_PAIR(5));printw("         a:");attroff(COLOR_PAIR(5));printw("  Move downwards            ");
  move(13,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   2:");attroff(COLOR_PAIR(5));attron(COLOR_PAIR(2));printw("      Pose                 ");  attroff(COLOR_PAIR(2));
  attron(COLOR_PAIR(5));printw("    z:");attroff(COLOR_PAIR(5));printw("  Turn counter-clockwise %.2f rad      ",CONTROLLER_STEP_COMMAND_YAW);
  
  move(14,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   3:");attroff(COLOR_PAIR(5));attron(COLOR_PAIR(3));printw("      Attitude              "); attroff(COLOR_PAIR(3));
  attron(COLOR_PAIR(5));printw("   x:");attroff(COLOR_PAIR(5));printw("  Turn clockwise %.2f rad        ",CONTROLLER_STEP_COMMAND_YAW);
  move(15,0);clrtoeol();
  printw("--------------------------------------------------------------------------------");
  refresh();
}

//High level command
void publishCmd(){
  command_publ.publish(command_order);
}

//Take off
void takeOff(){
  command_order.command = droneMsgsROS::droneCommand::TAKE_OFF;
  publishCmd();
}

//Hover
void hover(){
  clearSpeedReferences();
  publishSpeedReference();
  command_order.command = droneMsgsROS::droneCommand::HOVER;
  publishCmd();
}

//Land
void land(){
  clearSpeedReferences();
  publishSpeedReference();
  command_order.command = droneMsgsROS::droneCommand::LAND;
  publishCmd();
}

//Emergency Stop
void emergencyStop(){
  clearSpeedReferences();
  publishSpeedReference();
  command_order.command = droneMsgsROS::droneCommand::EMERGENCY_STOP;
  publishCmd();
}

//Move
void move(){
  command_order.command = droneMsgsROS::droneCommand::MOVE;
  publishCmd();
}

//Clear speed references
void clearSpeedReferences(){
  speed_reference_msg.twist.linear.x = 0.0;
  speed_reference_msg.twist.linear.y = 0.0;
  speed_reference_msg.twist.linear.z = 0.0;
  speed_reference_msg.twist.angular.x = 0.0;
  speed_reference_msg.twist.angular.y = 0.0;
  speed_reference_msg.twist.angular.z = 0.0;
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
void controlModeCallback(const aerostack_msgs::QuadrotorPidControllerMode::ConstPtr& msg){
  control_mode_msg = (*msg);
}

//Set control mode
bool setControlMode(int new_control_mode){
  // Prepare service message
  aerostack_msgs::SetControlMode setControlModeSrv;
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

