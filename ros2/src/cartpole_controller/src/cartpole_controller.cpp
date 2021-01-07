#include "cartpole_controller/cartpole_controller.hpp"
#include "cartpole_controller/controller/pid_controller.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/empty.hpp"

#include <memory>
#include <cmath>
#include <functional>
#include <chrono>

namespace cartpole{

  CartpoleController::CartpoleController(const rclcpp::NodeOptions &options)
    : Node("cartpole_controller", options)
  {
    // declare param and get param
    maxLimitPosition_ = 0.43;
    
    // init
    cartpoleState_.resize(4);

    // topic
    auto qosSensor = rclcpp::QoS(
      rclcpp::QoSInitialization(
        rmw_qos_profile_sensor_data.history,
        rmw_qos_profile_sensor_data.depth
      ),
      rmw_qos_profile_sensor_data);
    
    jointState_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 
      qosSensor,
      std::bind(&CartpoleController::joint_state_callback, 
                this, std::placeholders::_1));

    cartPosition_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/stand_cart_position_controller/command", 10);

    // service
    auto reset_fn{
      [this](
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response){
          this->reset();
        }
    };
    reset_srv_ = create_service<std_srvs::srv::Empty>(
      "reset_cartpole_controller", reset_fn);
    
    // timer
    clock_ = rclcpp::Clock(RCL_ROS_TIME);
    prevTime_ = clock_.now();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&CartpoleController::compute, this));
    
    pidCtrl_ptr_ = std::make_shared<controller::PIDController>(
      0.0, 2.0, 0.48, 0.13, // pole_pid
      0.0, 0, 0.0, 0.0);  // cart_pid
    
    reset();
  }

  void CartpoleController::reset()
  {
    pidCtrl_ptr_->reset();
    initFlag_ = false;

    publish(0.0);
    prevTime_ = clock_.now();

    std::cout << " reset \n";
    std::flush(std::cout);
  }

  void CartpoleController::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "I heard: [%f %f %f %f]", msg->position[0], msg->position[1], msg->velocity[0], msg->velocity[1]);
    // std::flush(std::cout);
    if(initFlag_ == false){
      initFlag_ = true;
    }

    cartpoleState_[0] = msg->position[0];
    cartpoleState_[1] = msg->position[1];
    
    cartpoleState_[2] = msg->velocity[0];
    cartpoleState_[3] = msg->velocity[1];
  }

  void CartpoleController::publish(double command)
  {
    auto pub_msg = std::make_unique<std_msgs::msg::Float64>();

    if(command > maxLimitPosition_)
      pub_msg->data = maxLimitPosition_;
    else if(command < -maxLimitPosition_)
      pub_msg->data = -maxLimitPosition_;
    else
      pub_msg->data = command;
    
    cartPosition_pub_->publish(std::move(pub_msg));

    // RCLCPP_INFO(this->get_logger(), "I heard: [%f]", command);
    // std::cout<< command << "\n";
    // std::flush(std::cout);
  }

  void CartpoleController::compute()
  { 
    rclcpp::Duration delta = clock_.now() - prevTime_ ;

    if(initFlag_){
      publish(pidCtrl_ptr_->compute(cartpoleState_, delta.seconds()));
    }
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(cartpole::CartpoleController)