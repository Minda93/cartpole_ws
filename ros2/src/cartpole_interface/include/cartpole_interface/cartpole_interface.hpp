#ifndef CARTPOLE_INTERFACE_HPP_
#define CARTPOLE_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "cartpole_msgs/srv/reset_env.hpp"
#include "gazebo_msgs/srv/set_model_configuration.hpp"

namespace interface{
  class CartpoleInterface : public rclcpp::Node{
  public:
    explicit CartpoleInterface(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~CartpoleInterface() = default;
    
    bool reset();
    void pause();
    void unpause();
    void reset_cartpole_ctrl();
    void set_model_state(double cartPos = 0.0, double poleAng = 0.0);

  private:
    void init_cartPos(double pos);
  
  private:
    // topic
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cartPos_pub_;

    // server
    rclcpp::Service<cartpole_msgs::srv::ResetEnv>::SharedPtr reset_srv_;

    // Client
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr pauseEnv_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr unpauseEnv_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr resetCtrl_client_;
    rclcpp::Client<gazebo_msgs::srv::SetModelConfiguration>::SharedPtr setModel_client_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

  private:
    double cartPos_;
    double poleAng_;
    double maxLimitCartPos_;

  }; //class Listener
} // interface

#endif // CARTPOLE_INTERFACE_HPP_