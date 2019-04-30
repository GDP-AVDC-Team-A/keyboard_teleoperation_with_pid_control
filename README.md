# Keyboard Teleoperation Interface

# Brief

![Keyboard Teleoperation Interface](https://i.ibb.co/yBzwPSx/keyboard-teleoperation.png)

# Services

- **set_control_mode** ([aerostack_msgs/controlMode](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/6928ccc6afeb3250bc3e4b285ccfc252d213cb3e/srv/ControlMode.srv))  
This service establishes the trajectory's control mode. The mode can be POSE, SPEED or TRAJECTORY. If the mode is POSE, the reference used is the pose reference (references for speed and trajectory are ignored). If the mode is SPEED, the reference used is speed reference (references for pose and trajectory are ignored). If the mode is TRAJECTORY, the reference used is the trajectory reference (references for pose and speed are ignored).

- **quadrotor_pid_controller_process/start** ([std_srvs/Empty](http://docs.ros.org/lunar/api/std_srvs/html/srv/Empty.html))  
This service starts the necessary contoller quadrotor_pid_controller_process.

- **initiate_behaviors** ([droneMsgsROS/InitiateBehaviors](https://bitbucket.org/joselusl/dronemsgsros/src/master/srv/InitiateBehaviors.srv))  
Initialize the behavior system.

- **activate_behavior** ([aerostack_msgs/BehaviorSrv](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/master/srv/BehaviorSrv.srv))  
This service starts or activates or inhibits a behavior.

# Subscribed topics

- **motion_reference/assumed_control_mode** ([aerostack_msgs/FlightMotionControlMode](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/6928ccc6afeb3250bc3e4b285ccfc252d213cb3e/msg/FlightMotionControlMode.msg))  
Current controller's control mode.

- **motion_reference/speed** ([geometry_msgs/TwistStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/TwistStamped.html))  
Speed reference for the controller. This is the reference used by the controller when the control mode is SPEED.

- **self_localization/pose** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))      
Current pose of the vehicle


# Published topics

- **motion_reference/pose** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))  
Pose reference for the controller. This is the reference used by the controller when the control mode is POSE.

- **motion_reference/speed** ([geometry_msgs/TwistStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/TwistStamped.html))  
Speed reference for the controller. This is the reference used by the controller when the control mode is SPEED.

- **actuator_command/quadrotor_command** ([mav_msgs/RollPitchYawrateThrust](http://docs.ros.org/api/mav_msgs/html/msg/RollPitchYawrateThrust.html))           
Actuator command for the quadrotor specifying the derivative of the robot altitude (thrust), yaw rate, roll and pitch.

## Deprecated published topics
- **command/high_level** ([droneMsgsROS/droneCommand](https://bitbucket.org/joselusl/dronemsgsros/src/2b47c507de4b636562f313f07abf07991b2432c4/msg/droneCommand.msg?at=master&fileviewer=file-view-default))           
It changes the status of a quadrotor vehicle. 

---
# Contributors
**Code Maintainer:** Alberto Rodelgo Perales

**Author:** Alberto Rodelgo Perales