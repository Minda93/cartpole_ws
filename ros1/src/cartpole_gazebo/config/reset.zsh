#!/bin/zsh

POS_COMMAND=0.0
POLE_COMMAND=0.05

rostopic pub --once /stand_cart_position_controller/command std_msgs/Float64 "data: ${POS_COMMAND}"
rosservice call /gazebo/pause_physics "{}"
rosservice call /gazebo/set_model_configuration "{model_name: 'cartpole', urdf_param_name: '', joint_names: ['stand_cart', 'cart_pole'], joint_positions: [${POS_COMMAND}, ${POLE_COMMAND}]}"
