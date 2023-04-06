# Robotics Software Engineer Task

You are asked to design and implement a pick-and-place application using a Gazebo simulation environment. The expected result of this task is a docker image and a GitHub repo with the source code.

## Goal

The scope of this assignment is limited to the robot control part of a continuous pick-and-place cycle. There are two tables; one with 5 blocks and one empty. The goal is to move all of the blocks onto the other table.

No vision/perception is needed. Poses of the blocks are accessible through the ros_service:

`/gazebo/get_model_state`

The gripper can be controlled through a topic:

`/gripper_joint_position/command`

## What was done:

1) moveit package ur5e_rg2 for using moveit_commander in python scripts
2) ros package with pick_n_place node (requests for "block_*" position and tries to transfer it to table2)
3) after few experiments the following setting were chosen: ompl planning pipeline with RRTConnect, KPIECE, SBL and SPARStwo planners

## Build && Run

### Using docker image

```bash
$ docker pull ghcr.io/remyrobotics/robotics-test:latest
$ xhost local:root
$ docker-compose up
```

Alternatively, you can build manually with the given Dockerfile.

### Building from Source

```bash
$ ./build.sh
$ cd catkin_ws
$ catkin build
```

Then start roscore in first terminal
```bash
$ source devel/setup.bash
$ roslaunch simple_scene gazebo.launch
```

launch gazebo scene in second terminal
```bash
$ source devel/setup.bash
$ roslaunch simple_scene gazebo.launch
```

and launch pick and place controller in third terminal
```bash
$ source devel/setup.bash
$ roslaunch roslaunch pick_n_place pick_n_place.launch
```

## Submission

A docker image and the source code are required. Please explain your design choices. If it is not possible due to circumstances, please send us source codes with clear instructions and explanations.

## Additional Questions

- How would you improve the efficiency and reliability?
  - Start from benchmark different planners for gauranteed getting time-optimal trajectories in parallel manner.
  - For more difficult application draw Gantt diagram or cyclogram and analyze it for the bottlenecks.
  - Cover code with tests.

- What would you do if we needed multiple robots in a system?

  - Trying to divide tasks in space/time for separately acting robots.
  - Think about centralized or decentralized design depending on the circumtances. In decentralized system robots can be ranged by priority (less priority robots must avoid higher priority robots). In centralized design main system can plan all the actions of robots in single whole formalizated task.
  

- How would you deploy the application to multiple physical locations? What is needed to scale it?
  - Ensure network reliability
  - Something like Gitlab CI/CD pipeline
  - Testing environment (digital copies of real environment)
  - Provide logging and alerts to some centralized system
  - Provide duplication/reservation of setups
  


