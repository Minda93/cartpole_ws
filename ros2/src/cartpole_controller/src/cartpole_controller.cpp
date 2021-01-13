#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "cartpole_controller/cartpole_controller.hpp"
#include "cartpole_controller/cartpole_param.hpp"
#include "cartpole_controller/controller/pid_controller.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/empty.hpp"

#include <memory>
#include <cmath>
#include <functional>
#include <chrono>

namespace cartpole{

  CartpoleController::CartpoleController(const rclcpp::NodeOptions &options)
    : Node("cartpole_controller", options),
    mode_{IDLE}
  {
    // declare param and get param
    this->declare_parameter<double>("cartpole/max_limit_pos", 0.43);
    this->declare_parameter<double>("cartpole/pole_target", 0.0);
    this->declare_parameter<double>("cartpole/cart_target", 0.0);

    this->declare_parameter<double>("cartpole/pid/pole_p", 2.0);
    this->declare_parameter<double>("cartpole/pid/pole_i", 0.48);
    this->declare_parameter<double>("cartpole/pid/pole_d", 0.13);

    this->declare_parameter<double>("cartpole/pid/cart_p", 0.0);
    this->declare_parameter<double>("cartpole/pid/cart_i", 0.0);
    this->declare_parameter<double>("cartpole/pid/cart_d", 0.0);
    
    load_config();
    
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
    
    startTimer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&CartpoleController::start, this));
    
    // pidCtrl_ptr_ = std::make_shared<controller::PIDController>(
    //   0.0, 2.0, 0.48, 0.13, // pole_pid
    //   0.0, 0, 0.0, 0.0);  // cart_pid

    pidCtrl_ptr_ = std::make_shared<controller::PIDController>(config_);
    
    reset();
  }

  void CartpoleController::load_config(){
    // this->declare_parameter<double>("cartpole/max_limit_pos", 0.43);
    // this->declare_parameter<double>("cartpole/pole_target", 0.0);
    // this->declare_parameter<double>("cartpole/cart_target", 0.0);

    // this->declare_parameter<double>("cartpole/pid/pole_p", 2.0);
    // this->declare_parameter<double>("cartpole/pid/pole_i", 0.48);
    // this->declare_parameter<double>("cartpole/pid/pole_d", 0.13);

    // this->declare_parameter<double>("cartpole/pid/cart_p", 0.0);
    // this->declare_parameter<double>("cartpole/pid/cart_i", 0.0);
    // this->declare_parameter<double>("cartpole/pid/cart_d", 0.0);

    get_parameter_or<double>(
      "cartpole/max_limit_pos",
      config_.maxLimitPos,
      0.43);
    
    get_parameter_or<double>(
      "cartpole/pole_target",
      config_.poleTarget,
      0.0);
    
    get_parameter_or<double>(
      "cartpole/cart_target",
      config_.cartTarget,
      0.0);
    
    get_parameter_or<double>(
      "cartpole/pid/pole_p",
      config_.polePID.kp,
      0.0);
    
    get_parameter_or<double>(
      "cartpole/pid/pole_i",
      config_.polePID.ki,
      0.0);
    
    get_parameter_or<double>(
      "cartpole/pid/pole_d",
      config_.polePID.kd,
      0.0);
    
    get_parameter_or<double>(
      "cartpole/pid/cart_p",
      config_.cartPID.kp,
      0.0);
    
    get_parameter_or<double>(
      "cartpole/pid/cart_i",
      config_.cartPID.ki,
      0.0);
    
    get_parameter_or<double>(
      "cartpole/pid/cart_d",
      config_.cartPID.kd,
      0.0);
  }

  void CartpoleController::reset()
  {
    mode_ = IDLE;
    pidCtrl_ptr_->reset();

    publish(config_.cartTarget);
    prevTime_ = clock_.now();

    startTimer_->reset();

    std::cout << " reset \n";
    std::flush(std::cout);
  }

  void CartpoleController::start()
  {
    mode_ = INACTIVE;
    startTimer_->cancel();

    // std::cout << " inactive \n";
    // std::flush(std::cout);
  }

  void CartpoleController::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "I heard: [%f %f %f %f]", msg->position[0], msg->position[1], msg->velocity[0], msg->velocity[1]);
    // std::flush(std::cout);
    if(mode_ == INACTIVE){
      mode_ = START_UP;
    }

    cartpoleState_[0] = msg->position[0];
    cartpoleState_[1] = msg->position[1];
    
    cartpoleState_[2] = msg->velocity[0];
    cartpoleState_[3] = msg->velocity[1];
  }

  void CartpoleController::publish(double command)
  {
    auto pub_msg = std::make_unique<std_msgs::msg::Float64>();

    if(command > config_.maxLimitPos)
      pub_msg->data = config_.maxLimitPos;
    else if(command < -config_.maxLimitPos)
      pub_msg->data = -config_.maxLimitPos;
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

    if(mode_ == START_UP){
      publish(pidCtrl_ptr_->compute(cartpoleState_, delta.seconds()));
    }
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(cartpole::CartpoleController)