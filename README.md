<!-- TOC -->

- [cartpole_ws](#cartpole_ws)
- [1. environment](#1-environment)
- [2. Setup env](#2-setup-env)
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

# TODO  
  
- [ ] cartpole_interface
- [ ] cartpole_controller

# Bug
  * cartpole_interface
    * Set models success but models is not moved in the gazebo gui  

# Reference

* [ppo_gazebo_tf](https://github.com/nav74neet/ppo_gazebo_tf)
  * cartpole_env
* [ros1_bridge](https://github.com/ros2/ros1_bridge)
  