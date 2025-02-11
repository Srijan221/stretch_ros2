![](../images/banner.png)

## Overview

*stretch_demos* provides simple demonstrations for the Stretch mobile manipulator from Hello Robot Inc. For an overview of the demos, we recommend you look at the following forum post: https://forum.hello-robot.com/t/autonomy-video-details

## Getting Started Demos

**Please be aware that these demonstrations typically do not perform careful collision avoidance. Instead, they expect to operate in freespace and detect contact through motor current if an obstacle gets in the way. Please be careful when trying out these demonstrations.**

### Handover Object Demo

First, place the robot near you so that it can freely move back and forth and reach near your body. Then, launch the handover object demo using the following command: 

```
ros2 launch stretch_demos handover_object.launch.py
```

Launch the keyboard teleop node with the handover_object param set as True in a separate terminal:
```
ros2 run stretch_core keyboard_teleop --ros-args -p handover_object_on:=True
```

For this demonstration, the robot will pan its head back and forth looking for a face. It will remember the 3D location of the mouth of the nearest face that it has detected. If you press "y" or "Y" on the keyboard in the terminal, the robot will move the grasp region of its gripper toward a handover location below and away from the mouth. 

The robot will restrict itself to Cartesian motion to do this. Specifically, it will move its mobile base backward and forward, its lift up and down, and its arm in and out. If you press "y" or "Y" again, it will retract its arm and then move to the most recent mouth location it has detected. 

At any time, you can also use the keyboard teleoperation commands in the terminal window. With this, you can adjust the gripper, including pointing it straight out and making it grasp an object to be handed over.

### Grasp Object Demo

For this demonstration, the robot will look for the nearest elevated surface, look for an object on it, and then attempt to grasp the largest object using Cartesian motions. Prior to running the demo, you should move the robot so that its workspace will be able to move its gripper over the surface while performing Cartesian motions. 

Once the robot is in position, retract and lower the arm so that the robot can clearly see the surface when looking out in the direction of its arm. 

Now that the robot is ready, launch the demo with the following command:

```
ros2 launch stretch_demos grasp_object.launch.py
```

Launch the keyboard teleop node with the grasp_object param set as True in a separate terminal:
```
ros2 run stretch_core keyboard_teleop --ros-args -p grasp_object_on:=True
```

Then, press the key with ‘ and “ on it while in the terminal to initiate a grasp attempt.

While attempting the grasp the demo will save several images under the ./stretch_user/debug/ directory within various grasping related directories. You can view these images to see some of what the robot did to make its decisions.

### Clean Surface Demo

For this demonstration, the robot will look for the nearest elevated surface, look for clear space on it, and then attempt to wipe the clear space using Cartesian motions. Prior to running the demo, you should move the robot so that its workspace will be able to move its gripper over the surface while performing Cartesian motions. 

**You should also place a soft cloth in the robot's gripper.**

Once the robot is in position with a cloth in its gripper, retract and lower the arm so that the robot can clearly see the surface when looking out in the direction of its arm. 

Now that the robot is ready, launch the demo with the following command:

```
ros2 launch stretch_demos clean_surface.launch.py
```


Launch the keyboard teleop node with the clean_surface param set as True in a separate terminal:
```
ros2 run stretch_core keyboard_teleop --ros-args -p clean_surface_on:=True
```
Then, press the key with the / and ? on it while in the terminal to initiate a surface cleaning attempt.

### Open Drawer Demo

For this demonstration, the robot will use contact detection with its arm to attempt to open a drawer. Prior to running the demo, you should move the robot so that its workspace will be able to move its arm towards a drawer while extending its arm. 

The robot will not use the camera to detect the drawer, so ensure that there are no obstacles in the way.

Now that the robot is ready, launch the demo with the following command:

```
ros2 launch stretch_demos open_drawer.launch.py
```

Launch the keyboard teleop node with the open_drawer param set as True in a separate terminal:
```
ros2 run stretch_core keyboard_teleop --ros-args -p open_drawer_on:=True
```
Then, press the 'z' or 'Z' key while in the terminal to initiate an open drawer with downward hook motion.
Alternatively, press the '.' or '>' key while in the terminal to initiate an open drawer with upward hook motion.

<!-- ### Autodocking with Nav Stack

For this demo, the robot will look for a docking station with an ArUco marker, align itself to the docking station using the ROS Nav Stack, and then back up into the docking station using an error-based controller. Prior to running this demo, you should stow the robot, ensure that you have a pregenerated map and the docking station is resting against a wall. Go through [this](https://docs.hello-robot.com/0.2/stretch-tutorials/ros1/autodocking_nav_stack/) tutorial to understand how the demo works.

You can launch this demo using the following command:

```
roslaunch stretch_demos autodocking.launch map_yaml:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml -->
```

## License

For license information, please see the LICENSE files. 