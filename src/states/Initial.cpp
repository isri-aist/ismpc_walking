#include "Initial.h"
#include <mc_control/fsm/Controller.h>

void Initial::configure(const mc_rtc::Configuration & config) {}

void Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<Walking_controller &>(ctl_);
}

bool Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<Walking_controller &>(ctl_);
  output("OK");
  return true;
}

void Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<Walking_controller &>(ctl_);
}

EXPORT_SINGLE_STATE("Initial", Initial)