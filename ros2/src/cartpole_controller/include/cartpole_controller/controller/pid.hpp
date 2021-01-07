#ifndef CARTPOLE_PID_HPP_
#define CARTPOLE_PID_HPP_

namespace cartpole{
  namespace controller{
    class PID{

    public:
      PID(double target, double kp, double ki, double kd);
      ~PID() = default;

      double run(double state, long double delta);
      void reset();
      void set_target(double target);

    private:
      double target_;
      double kp_;
      double ki_;
      double kd_;

      double prevError_{0.0};
      double integral_{0.0};
    }; // class PID
  } // namespace controller
} // namespace cartpole

#endif // CARTPOLE_PID_HPP_
