#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>
#include "../../include/ismpc_walking/Walking_controller.h"

struct Initial : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl_) override;
  bool run(mc_control::fsm::Controller & ctl_) override;
  void teardown(mc_control::fsm::Controller & ctl_) override;

private:
};
