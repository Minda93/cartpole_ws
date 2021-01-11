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

# Bug
  * cartpole_interface
    * Set models success but models is not moved in the gazebo gui  

# Reference

* [ppo_gazebo_tf](https://github.com/nav74neet/ppo_gazebo_tf)
  * cartpole_env
* [ros1_bridge](https://github.com/ros2/ros1_bridge)
  