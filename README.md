<!-- TOC -->

- [cartpole_ws](#cartpole_ws)
- [1. environment](#1-environment)
- [2. Setup env](#2-setup-env)
- [3. Build env](#3-build-env)
  - [3.1 ros1](#31-ros1)
  - [3.2 ros2](#32-ros2)
  - [3.3 ros1_bridge](#33-ros1_bridge)
- [4. Run](#4-run)
  - [4.1 cartpole simulation](#41-cartpole-simulation)
  - [4.2 ros1_bridge](#42-ros1_bridge)
  - [4.3 cartpole components test](#43-cartpole-components-test)
  - [4.4 rqt_gui](#44-rqt_gui)
- [5. components](#5-components)
  - [5.1 cartpole_interface](#51-cartpole_interface)
  - [5.2 cartpole_controller](#52-cartpole_controller)
- [TODO](#todo)
- [Bug](#bug)
- [Reference](#reference)

<!-- /TOC -->

# cartpole_ws

* ros1 and ros2 practice 1

# 1. environment  
| names  | version        |
| ---    | ---            |
| gazebo | 11.1.0         |
| ros1   | noetic         |
| ros2   | foxy-release_4 |

# 2. Setup env  
* download packages  
  
```bash
  cd <cartpole_ws>/ros2/src
  git clone https://github.com/ros2/ros1_bridge.git -b foxy
```  
  
# 3. Build env  

## 3.1 ros1  
* shall A  

```bash
  $ cd <path_of_ros1>
  $ source <install-space-with-ros1>/setup.bash
  $ catkin_make
```  

## 3.2 ros2  
* shall B  

```bash
  $ cd <path_of_ros2>
  $ source <install-space-with-ros2>/setup.bash
  $ colcon build --symlink-install --packages-skip ros1_bridge
```  

## 3.3 ros1_bridge  
* shall C  

```bash
  $ cd <path_of_ros2>
  $ source <install-space-with-ros1>/setup.bash
  $ source <install-space-to-ros1-overlay-ws>/setup.bash
  $ source <install-space-to-ros2-overlay-ws>/setup.bash
  $ colcon build --symlink-install --packages-skip ros1_bridge
```

# 4. Run

## 4.1 cartpole simulation  

```bash
  $ cd <install-space-to-ros1-overlay-ws>/
  $ source <install-space-to-ros1-overlay-ws>/setup.bash
  $ roslaunch cartpole_gazebo cartpole_gazebo.launch
```  

## 4.2 ros1_bridge

```bash
  $ cd <path_of_ros2>
  $ source <install-space-to-ros1-overlay-ws>/setup.bash
  $ source <install-space-to-ros2-overlay-ws>/setup.bash
  
  # maybe use
  # $ export ROS_MASTER_URI=http://localhost:11311

  $ ros2 run ros1_bridge dynamic_bridge
```  

## 4.3 cartpole components test
* components :
  * cartpole_interface
  * cartpole_controller

```bash
  $ cd <path_of_ros2>
  $ source <install-space-with-ros2>/setup.bash
  $ source <install-space-to-ros2-overlay-ws>/local_setup.bash
  $ ros2 launch cartpole_controller cartpole_all_test_launch.py
```  

## 4.4 rqt_gui  
* plugins
  * Service Caller
  * Dynamic Reconfigure  

```bash
  $ source <install-space-with-ros2>/setup.bash
  $ source <install-space-to-ros2-overlay-ws>/local_setup.bash
  $ ros2 run rqt_gui rqt_gui
```

# 5. components

## 5.1 cartpole_interface  
* topic
  * output
    * stand_cart_position_controller/command (std_msgs/float64)
* service
  * reset_env (cartpole_msgs/ResetEnv)
  * gazebo/pause_physics (std_srvs/empty)
  * gazebo/unpause_physics (std_srvs/empty)
  * reset_cartpole_controller (std_srvs/empty)
  * gazebo/set_model_configuration (gazebo_msgs/SetModelConfiguration)

## 5.2 cartpole_controller  
* topic
  * input 
    * joint_states (sensor_msgs/joint_state)
  * output
    * stand_cart_position_controller/command (std_msgs/float64)
* service
  * reset_cartpole_controller (std_srvs/empty)
* parameter
  * cartpole/cart_target   (double)
  * cartpole/max_limit_pos (double)
  * cartpole/pid/cart_d    (double)
  * cartpole/pid/cart_i    (double)
  * cartpole/pid/cart_p    (double)
  * cartpole/pid/pole_d    (double)
  * cartpole/pid/pole_i    (double)
  * cartpole/pid/pole_p    (double)
  * cartpole/pole_target   (double)

# TODO  
  
- [x] cartpole_interface
- [x] cartpole_controller : ros2 parameter
- [ ] cartpole_controller : find pid parameters

# Bug
  * cartpole_interface
    * Use service command line : Set the models success but the models is not moved in the gazebo gui.
    * Use `service Caller` in the rqt_gui : Set the models success and the models can be moved in the gazebo gui.  
    
  * cartpole_controller 
    * Using component node to load params can **NOT** success in launch file  
      * for launch_ros 0.11.1
      * solution: change yaml file format
        * normal format
        ```yaml
          node_name:
            ros__parameters:
              param1: 0.0
              param2: 0.1
        ```  

        * special format
        ```yaml
          param1: 0.0
          param2: 0.1
        ```  
        
    * ros2 param command line is sometimes NOT working
      * all processing restart  
    
    * Starting controller is sometimes not better.
      * /reset_env is not correctly implemented.
        * "/stand_cart_position_controller/command" 
        * "reset_controller"
      * parameters are not better 
    
# Reference

* [ppo_gazebo_tf](https://github.com/nav74neet/ppo_gazebo_tf)
  * cartpole_env
* [ros1_bridge](https://github.com/ros2/ros1_bridge)
* [launch_ros issue](https://github.com/ros2/launch_ros/issues/156#issuecomment-759644376)
  * component node bug 
  