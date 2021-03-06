#include "cartpole_controller/controller/pid_controller.hpp"

#include "cartpole_controller/cartpole_param.hpp"
#include "cartpole_controller/controller/pid.hpp"

#include <iostream>
#include <vector>
#include <memory>
#include <cmath>

namespace cartpole{
  namespace controller{
    PIDController::PIDController(
      double target_1, double kp_1, double ki_1, double kd_1,
      double target_2, double kp_2, double ki_2, double kd_2)
    { 
      // 0.0 28 0.01 0.1
      poleAngCtrl_ptr_ = std::make_shared<controller::PID>(
        target_1, kp_1, ki_1, kd_1);
      
      // 0.0 0.1 0.0 0.5
      cartPosCtrl_ptr_ = std::make_shared<controller::PID>(
        target_2, kp_2, ki_2, kd_2);
    }

    PIDController::PIDController(CartpoleParam config)
    : PIDController(
        config.poleTarget, config.polePID.kp, config.polePID.ki, config.polePID.kd,
        config.cartTarget, config.cartPID.kp, config.cartPID.ki, config.cartPID.kd
      )
    {

    }

    void PIDController::reset()
    {
      poleAngCtrl_ptr_->reset();
      cartPosCtrl_ptr_->reset();
    }

    void PIDController::set_params(const CartpoleParam config)
    {
      poleAngCtrl_ptr_->set_target(config.poleTarget);
      poleAngCtrl_ptr_->set_kp(config.polePID.kp);
      poleAngCtrl_ptr_->set_ki(config.polePID.ki);
      poleAngCtrl_ptr_->set_kd(config.polePID.kd);

      cartPosCtrl_ptr_->set_target(config.cartTarget);
      cartPosCtrl_ptr_->set_kp(config.cartPID.kp);
      cartPosCtrl_ptr_->set_ki(config.cartPID.ki);
      cartPosCtrl_ptr_->set_kd(config.cartPID.kd);
    }

    double PIDController::compute(const std::vector<double> &state, double delta)
    {
      double polePID{poleAngCtrl_ptr_->run(state[0], delta)};
      double cartPID{cartPosCtrl_ptr_->run(state[1], delta)};
      double cartVec{0.0};

      cartVec = state[1]+0.1*normalize(polePID+cartPID);
      
      // std::cout<< state[0] << " " << state[1] << " " 
      //         << polePID << " " << cartPID << " " // << " \n ";
      //         << cartVec << "\n" ;
      // std::flush(std::cout);

      return cartVec;
    }

    double PIDController::normalize(double command)
    {
      return tanh(command);
    }
  }
}