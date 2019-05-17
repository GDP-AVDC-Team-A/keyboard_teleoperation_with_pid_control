# Keyboard Teleoperation Interface

![Keyboard Teleoperation Interface](https://i.ibb.co/yBzwPSx/keyboard-teleoperation.png)

# Subscribed topics

- **self_localization/pose** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))      
Current pose of the vehicle

- **motion_reference/speed** ([geometry_msgs/TwistStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/TwistStamped.html))  
Speed reference for the controller.

- **motion_reference/assumed_control_mode** ([aerostack_msgs/QuadrotorPidControllerMode](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/master/msg/QuadrotorPidControllerMode.msg))  
Current controller's control mode.


# Published topics

- **motion_reference/pose** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))  
Pose reference for the controller.

- **motion_reference/speed** ([geometry_msgs/TwistStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/TwistStamped.html))  
Speed reference for the controller.

- **command/high_level** ([droneMsgsROS/droneCommand](https://bitbucket.org/joselusl/dronemsgsros/src/2b47c507de4b636562f313f07abf07991b2432c4/msg/droneCommand.msg?at=master&fileviewer=file-view-default))           
It changes the status of a quadrotor vehicle. 

---
# Contributors
**Code Maintainer:** Alberto Rodelgo Perales

**Author:** Alberto Rodelgo Perales