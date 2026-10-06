# How to use the nero_ctrl package


## Using can communication

To activate can communication
```
cd ~/agx_arm_ws/src/agx_arm_ros/scripts 
bash can_activate.sh
```
Then

```
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Enable and disable the robot
To enable the arm, run the following command in a terminal:

```
ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"
```
To disable the arm, run the following command in a terminal:

```
ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: false}"
```
