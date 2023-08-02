#include "../include/ismpc_walking/Walking_controller.h"

#include <mc_joystick_plugin/joystick_inputs.h>

void Walking_controller::JoystickInputs()
{

  if(datastore().has("Joystick::connected"))
  {

    if(datastore().get<bool>("Joystick::connected"))
    {
      auto & buttonEvent_func = datastore().get<std::function<bool(joystickButtonInputs)>>("Joystick::ButtonEvent");
      auto & button_func = datastore().get<std::function<bool(joystickButtonInputs)>>("Joystick::Button");
      auto & trigger_func = datastore().get<std::function<double(joystickAnalogicInputs)>>("Joystick::Trigger");
      auto & stick_func = datastore().get<std::function<Eigen::Vector2d(joystickAnalogicInputs)>>("Joystick::Stick");

      if(buttonEvent_func(joystickButtonInputs::START) && button_func(joystickButtonInputs::START))
      {
        mc_rtc::log::info("ISMPC Control {}", active);
        if(!active)
        {
          activate();
        }
        else
        {
          deactivate();
        }
      }
      if(buttonEvent_func(joystickButtonInputs::B) && button_func(joystickButtonInputs::B))
      {
        mc_rtc::log::error_and_throw<std::runtime_error>("Hard Emergency triggered");
        emergencyFlag = true;
      }
      if(buttonEvent_func(joystickButtonInputs::A) && button_func(joystickButtonInputs::A))
      {
        mc_rtc::log::info("Start/Stop Walking");

        if(stabilizer_active_ && Stop)
        {
          compute_trajectory_once.notify_all();
          Stop = false;
        }
        else
        {
          Stop = true;
        }
      }
      reference_velocity.setZero();
      double vx_forward = -trigger_func(joystickAnalogicInputs::RT) + 1;
      double vx_backward = -trigger_func(joystickAnalogicInputs::LT) + 1;
      if(vx_forward > 0.1)
      {
        reference_velocity.x() = vx_forward;
      }
      if(vx_backward > 0.1)
      {
        reference_velocity.x() = -vx_backward;
      }
      double omega = stick_func(joystickAnalogicInputs::R_STICK).y() - 0.5;
      double vy = stick_func(joystickAnalogicInputs::L_STICK).y() - 0.5;
      if(std::abs(omega) > 0.1)
      {
        reference_velocity.z() = omega;
      }
      if(std::abs(vy) > 0.15)
      {
        reference_velocity.y() = vy;
      }
    }
  }
}
