#ifndef CARTPOLE_PID_CONTROLLER_HPP_
#define CARTPOLE_PID_CONTROLLER_HPP_

#include "cartpole_controller/cartpole_param.hpp"
#include "cartpole_controller/controller/pid.hpp"

#include <memory>
#include <vector>

namespace cartpole{
  namespace controller{
    class PIDController{
    
    public:
      explicit PIDController(
        double target_1, double kp_1, double ki_1, double kd_1,
        double target_2, double kp_2, double ki_2, double kd_2);
      
      explicit PIDController(CartpoleParam config);

      ~PIDController();
      
      void reset();
      void set_params(const CartpoleParam config);
      
      double compute(const std::vector<double> &state, double delta);

    private:
      double normalize(double error);

    private:
      std::shared_ptr<controller::PID> poleAngCtrl_ptr_;
      std::shared_ptr<controller::PID> cartPosCtrl_ptr_;

    }; // class PIDController 
  } // namespace controller
} // namespace cartpole

#endif // CARTPOLE_PID_CONTROLLER_HPP_