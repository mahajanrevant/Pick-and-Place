# Robotic arm - Pick & Place project
---

Make sure you are using robo-nd VM or have Ubuntu+ROS installed locally.

## Installation

### One time Gazebo setup step:
Check the version of gazebo installed on your system using a terminal:
```sh
$ gazebo --version
```
To run projects from this repository you need version 7.7.0+
If your gazebo version is not 7.7.0+, perform the update as follows:
```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo7
```

Once again check if the correct version was installed:
```sh
$ gazebo --version
```
### For the rest of this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly

If you do not have an active ROS workspace, you can create one by:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that you have a workspace, clone or download this repo into the **src** directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/RoboND-Kinematics-Project.git
```

Now from a terminal window:

```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ sudo chmod +x target_spawn.py
$ sudo chmod +x IK_server.py
$ sudo chmod +x safe_spawner.sh
```
Build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```

Add following to your .bashrc file
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/models

source ~/catkin_ws/devel/setup.bash
```

For demo mode make sure the **demo** flag is set to _"true"_ in `inverse_kinematics.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch

In addition, you can also control the spawn location of the target object in the shelf. To do this, modify the **spawn_location** argument in `target_description.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch. 0-9 are valid values for spawn_location with 0 being random mode.

You can launch the project by
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ ./safe_spawner.sh
```

If you are running in demo mode, this is all you need. To run your own Inverse Kinematics code change the **demo** flag described above to _"false"_ and run your code (once the project has successfully loaded) by:
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ rosrun kuka_arm IK_server.py
```
Once Gazebo and rviz are up and running, make sure you see following in the gazebo world:

	- Robot
	
	- Shelf
	
	- Blue cylindrical target in one of the shelves
	
	- Dropbox right next to the robot
	

If any of these items are missing, report as an issue.

Once all these items are confirmed, open rviz window, hit Next button.

To view the complete demo keep hitting Next after previous action is completed successfully. 

Since debugging is enabled, you should be able to see diagnostic output on various terminals that have popped up.

The demo ends when the robot arm reaches at the top of the drop location. 

There is no loopback implemented yet, so you need to close all the terminal windows in order to restart.

## Introduction 
Robots have started to replace humans to perform manually simple and iterative steps. This project deals with a simulation of a Kuka arm to pick and drop objects. A similar challenge is performed during amazon robotics challenge. The target object is a blue cylinder that spawns on 9 different locations on the rack at random and the goal is to drop the cylinder in the bin kept right next to the robot.

## Denavit-Hartenberg Diagram
Here is a Denavit-Hartenberg (DH) diagram of the Kuka KR210 by Udacity:

The arm consistes of 6 revolute joints and has a spherical wrist.

## Denavit-Hartenberg Table
DH parameter given by __Craig, J. J.__ are used to complete this project. 

| n |  theta |   d   |    a   | alpha |
|:-:|:------:|:-----:|:------:|:-----:|
| 0 |   -    |   -   |    0   |   0   |
| 1 | theta1 |  0.75 |  0.35  | -pi/2 |
| 2 | theta2 |   0   |  1.25  |   0   |
| 3 | theta3 |   0   | -0.054 | -pi/2 |
| 4 | theta4 |  1.5  |    0   |  pi/2 |
| 5 | theta5 |   0   |    0   | -pi/2 |
| 6 | theta6 | 0.303 |    0   |   0   |

# Transformation Matrices

All the joints have a transformation matrix which gives their relative orientation with reference to the previous joint. The TF matrix for any joint can be calculated by substituting the DH parameters in the below function:-
``` 
    def TF_matrix(alpha, a, d, q):
    T = Matrix([[            cos(q),           -sin(q),           0,             a],
                [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                 0,                 0,           0,             1]])
    return T
```
Individual Joints can be created as follows:-

Joint 0 to 1
```
T0_1 = TF_matrix(alpha0, a0, d1, q1).subs(s)
```
Joint 1 to 2
```
T1_2 = TF_matrix(alpha1, a1, d2, q2).subs(s)
```
Joint 2 to 3
```
T2_3 = TF_matrix(alpha2, a2, d3, q3).subs(s)
```
Joint 3 to 4
```
T3_4 = TF_matrix(alpha3, a3, d4, q4).subs(s)
```
Joint 4 to 5
```
T4_5 = TF_matrix(alpha4, a4, d5, q5).subs(s)
```
Joint 5 to 6
```
T5_6 = TF_matrix(alpha5, a5, d6, q6).subs(s)
```
Joint 6 to end-effector
```
T6_EE = TF_matrix(alpha6, a6, d7, q7).subs(s)
```

Multiplying these joint matrices we get the joiint matrix form joint 0 to the end effector.
```
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
```


