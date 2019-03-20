# tsid jet mujoco
tsid demo for jet with mujoco

## Installation
* clone this Repository and [mujoco_ros Repository](https://github.com/saga0619/mujoco_ros_sim) to your ros workspace (commonly at catkin_ws)
* install [pinocchio library](https://github.com/stack-of-tasks/pinocchio)
* install [tsid library](https://github.com/stack-of-tasks/tsid)
* then build (catkin_make )

## Mujoco License file 
For mujoco user, you need to edit the position of license in launch/sim.launch with your license file's location
(mjkey.txt at Home folder is default configuration)

## How to run
* launch sim.launch with roslaunch

```sh
roslaunch jet_python_mujoco sim.launch
```
* press run button from mujoco

* press 'i' key from terminal to start tsid demo

