<!-- TOC -->

- [cartpole_ws](#cartpole_ws)
- [1. environment](#1-environment)
- [2. Setup env](#2-setup-env)
- [3. Build env](#3-build-env)
  - [3.1 ros1](#31-ros1)
  - [3.2 ros2](#32-ros2)
  - [3.3 ros1_bridge](#33-ros1_bridge)
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

# TODO  
  
- [x] cartpole_interface
- [ ] cartpole_controller
- [x] cartpole_controller : ros2 parameter
- [ ] cartpole_controller : cart pid

# Bug
  * cartpole_interface
    * Use service command line : Set the models success but the models is not moved in the gazebo gui.
    * Use `service Caller` in the rqt_gui : Set the models success and the models can be moved in the gazebo gui.  
    
  * cartpole_controller 
    * Using component node to load params can **NOT** success in launch file  
      * for launch_ros 0.11.1
      * `Running ros2 param list is NOT working`
      * solution: change yaml file format
        * normal format
        ```yaml
          node_name:
            ros_parameters:
              param1: 0.0
              param2: 0.1
        ```  

        * special format
        ```yaml
          param1: 0.0
          param2: 0.1
        ```  
        
    * ros2 param command line is NOT working
      * use `dynamic reconfigure` in the rqt_gui
    
# Reference

* [ppo_gazebo_tf](https://github.com/nav74neet/ppo_gazebo_tf)
  * cartpole_env
* [ros1_bridge](https://github.com/ros2/ros1_bridge)
* [launch_ros issue](https://github.com/ros2/launch_ros/issues/156#issuecomment-759644376)
  * component node bug 
  