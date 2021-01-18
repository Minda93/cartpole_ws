#ifndef CARTPOLE_CONTROLLER_HPP_
#define CARTPOLE_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/empty.hpp"

#include "cartpole_controller/cartpole_param.hpp"
#include "cartpole_controller/controller/pid_controller.hpp"

#include <vector>
#include <memory>

namespace cartpole {
  class CartpoleController : public rclcpp::Node{
  
  public:
    explicit CartpoleController(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~CartpoleController() = default;

    void reset();
    void start();
    void compute();

  private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void publish(double command);

    void load_config();
    rcl_interfaces::msg::SetParametersResult dynamic_load_params(
      const std::vector<rclcpp::Parameter> &params);

  private:
    ControlMode mode_;
    CartpoleParam config_;
    
    std::vector<double> cartpoleState_;
    double maxLimitPosition_;
    bool initFlag_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointState_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cartPosition_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;

    // controller
    std::shared_ptr<controller::PIDController> pidCtrl_ptr_;

    // timer
    rclcpp::Clock clock_;
    rclcpp::Time prevTime_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr startTimer_;
    
  }; // CartpoleController
} // cartpole

#endif // CARTPOLE_CONTROLLER_HPP_