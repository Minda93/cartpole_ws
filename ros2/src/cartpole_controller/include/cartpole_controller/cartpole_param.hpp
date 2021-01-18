#ifndef CARTPOLE_PARAM_HPP_
#define CARTPOLE_PARAM_HPP_

namespace cartpole{
  enum ControlMode{
    IDLE,
    INACTIVE,
    START_UP,
    FINISH
  }; // enum ControlMode

  struct PIDParam
  {
    double kp;
    double ki;
    double kd;
  }; // struct PIDParam

  class CartpoleParam{
      
  public:
    // model
    double maxLimitPos;

    // target
    double poleTarget;
    double cartTarget;
    
    // pid controller
    PIDParam polePID;
    PIDParam cartPID;
  }; // class CartpoleParam
}

#endif // CARTPOLE_PARAM_HPP_