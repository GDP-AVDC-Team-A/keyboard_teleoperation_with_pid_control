# Keyboard Teleoperation Interface

<p align="center">
<img src="https://i.ibb.co/m5ngjvQ/keyboardground.png" title="Ground speed teleoperation control mode">
<div align="center">Ground speed teleoperation control mode</div>
</p>
<p align="center">
<img src="https://i.ibb.co/HGc47fV/keyboardpose.png" title="Pose teleoperation control mode">
<div align="center">Pose teleoperation control mode</div>
</p>
<p align="center">
<img src="https://i.ibb.co/hX7dkH2/keyboardattitude.png" title="Attitude teleoperation control mode">
<div align="center">Attitude teleoperation control mode</div>
</p>
# Subscribed topics

- **self_localization/pose** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))      
Current pose of the vehicle

- **motion_reference/speed** ([geometry_msgs/TwistStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/TwistStamped.html))  
Speed reference for the controller.

- **motion_reference/assumed_control_mode** ([aerostack_msgs/QuadrotorPidControllerMode](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/master/msg/QuadrotorPidControllerMode.msg))  
Current controller's control mode.

- **command/pitch_roll/temp** ([droneMsgsROS/dronePitchRollCmd](https://bitbucket.org/joselusl/dronemsgsros/src/master/msg/dronePitchRollCmd.msg))  
Pitch and roll commands from controllers.

- **ground_speed** ([droneMsgsROS/Vector2Stamped](https://bitbucket.org/joselusl/dronemsgsros/src/fa03af3fb09b943ea728d28683ff7b6032f74d66/msg/vector2Stamped.msg?at=master))   
Ground speed, i.e., horizontal speed of the vehicle relative to the ground.
# Published topics

- **motion_reference/pose** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))  
Pose reference for the controller.

- **motion_reference/speed** ([geometry_msgs/TwistStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/TwistStamped.html))  
Speed reference for the controller.

- **command/high_level** ([droneMsgsROS/droneCommand](https://bitbucket.org/joselusl/dronemsgsros/src/2b47c507de4b636562f313f07abf07991b2432c4/msg/droneCommand.msg?at=master&fileviewer=file-view-default))           
It changes the status of a quadrotor vehicle. 

- **command/pitch_roll** ([droneMsgsROS/dronePitchRollCmd](https://bitbucket.org/joselusl/dronemsgsros/src/master/msg/dronePitchRollCmd.msg))  
Publishes pitch and roll commands for controllers.

---
# Contributors
**Code Maintainer:** Alberto Rodelgo Perales

**Author:** Alberto Rodelgo Perales