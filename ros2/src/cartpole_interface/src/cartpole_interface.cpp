#include "cartpole_interface/cartpole_interface.hpp"


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "cartpole_msgs/srv/reset_env.hpp"
#include "gazebo_msgs/srv/set_model_configuration.hpp"

#include <functional>
#include <chrono>

namespace interface{
  CartpoleInterface::CartpoleInterface(const rclcpp::NodeOptions &options) 
    : Node("cartpole_interface", options)
  {
    // param
    cartPos_ = 0.0;
    poleAng_ = 0.0;
    maxLimitCartPos_ = 0.43;
    timer_ = create_wall_timer(std::chrono::seconds(3), std::bind(&CartpoleInterface::reset, this));

    // topic
    cartPos_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/stand_cart_position_controller/command", 10);
    
    // server
    auto reset_fn{
      [this](
        const std::shared_ptr<cartpole_msgs::srv::ResetEnv::Request> request,
        std::shared_ptr<cartpole_msgs::srv::ResetEnv::Response> response){
          
          if(cartPos_ > maxLimitCartPos_){
            cartPos_ = maxLimitCartPos_;
            RCLCPP_INFO(this->get_logger(), "cartPos over positive max limit");
          }else if(cartPos_ < -maxLimitCartPos_){
            cartPos_ = -maxLimitCartPos_;
            RCLCPP_INFO(this->get_logger(), "cartPos over negative max limit");
          }else{
            cartPos_ = request->cart_pos;
          }

          poleAng_ = request->pole_ang;

          response->success = this->reset();
        }
    };
    reset_srv_ = create_service<cartpole_msgs::srv::ResetEnv>(
      "/reset_env", reset_fn);

    // client
    pauseEnv_client_ = create_client<std_srvs::srv::Empty>("/gazebo/pause_physics");
    unpauseEnv_client_ = create_client<std_srvs::srv::Empty>("/gazebo/unpause_physics");
    resetCtrl_client_ = create_client<std_srvs::srv::Empty>("/reset_cartpole_controller");
    setModel_client_ = create_client<gazebo_msgs::srv::SetModelConfiguration>("/gazebo/set_model_configuration");
  }
  
  bool CartpoleInterface::reset()
  {
    unpause();
    
    set_model_state(cartPos_, poleAng_);
    init_cartPos(cartPos_);
    
    pause();

    set_model_state(cartPos_, poleAng_);
    reset_cartpole_ctrl();

    pause();
    
    timer_->cancel();
    return true;
  }

  void CartpoleInterface::pause()
  {
    if(!pauseEnv_client_->wait_for_service(std::chrono::seconds(1))) {
      if(!rclcpp::ok()){
        RCLCPP_ERROR(
          this->get_logger(),
          "Interrupted while waiting for the service. Exiting");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "/gazebo/pause Service not available afater waiting");
      return;
    }

    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto future_result = pauseEnv_client_->async_send_request(request);
  }

  void CartpoleInterface::unpause()
  {
    if(!unpauseEnv_client_->wait_for_service(std::chrono::seconds(1))) {
      if(!rclcpp::ok()){
        RCLCPP_ERROR(
          this->get_logger(),
          "Interrupted while waiting for the service. Exiting");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "/gazebo/unpause Service not available afater waiting");
      return;
    }

    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto future_result = unpauseEnv_client_->async_send_request(request);
  }

  void CartpoleInterface::reset_cartpole_ctrl()
  {
    if(!resetCtrl_client_->wait_for_service(std::chrono::seconds(1))) {
      if(!rclcpp::ok()){
        RCLCPP_ERROR(
          this->get_logger(),
          "Interrupted while waiting for the service. Exiting");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "/reset_cartpole_controller Service not available afater waiting");
      return;
    }

    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto future_result = resetCtrl_client_->async_send_request(request);
  }

  void CartpoleInterface::set_model_state(double cartPos, double poleAng)
  {
    if(!setModel_client_->wait_for_service(std::chrono::seconds(1))) {
      if(!rclcpp::ok()){
        RCLCPP_ERROR(
          this->get_logger(),
          "Interrupted while waiting for the service. Exiting");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "/gazebo/set_model_configuration Service not available afater waiting");
      return;
    }

    if(cartPos > maxLimitCartPos_){
      cartPos = maxLimitCartPos_;
      RCLCPP_INFO(this->get_logger(), "cartPos over positive max limit");
    }else if(cartPos < -maxLimitCartPos_){
      cartPos = -maxLimitCartPos_;
      RCLCPP_INFO(this->get_logger(), "cartPos over negative max limit");
    }

    auto request = std::make_shared<gazebo_msgs::srv::SetModelConfiguration::Request>();
    request->model_name = "cartpole";
    request->joint_names = std::vector<std::string>{"stand_cart", "cart_pole"};
    request->joint_positions = std::vector<double>{cartPos, poleAng};
    
    using ServiceResponseFuture =
    rclcpp::Client<gazebo_msgs::srv::SetModelConfiguration>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
      RCLCPP_INFO(this->get_logger(), "Got success: [%d]", future.get()->success);
    };

    auto future_result = setModel_client_->async_send_request(request, response_received_callback);

  }

  void CartpoleInterface::init_cartPos(double pos)
  {
    auto pub_msg = std::make_unique<std_msgs::msg::Float64>();
    
    if(pos > maxLimitCartPos_)
      pub_msg->data = maxLimitCartPos_;
    else if(pos < -maxLimitCartPos_)
      pub_msg->data = -maxLimitCartPos_;
    else
      pub_msg->data = pos;

    cartPos_pub_->publish(std::move(pub_msg));
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(interface::CartpoleInterface)