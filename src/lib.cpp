#include <ismpc_walking/Walking_controller.h>

extern "C"
{

  CONTROLLER_MODULE_API void MC_RTC_CONTROLLER(std::vector<std::string> & names)
  {
    CONTROLLER_CHECK_VERSION("ismpc_walking");
    names = {"ismpc_walking"};
  }

  CONTROLLER_MODULE_API void destroy(mc_control::MCController * ptr)
  {
    delete ptr;
  }

  CONTROLLER_MODULE_API unsigned int create_args_required()
  {
    return 4;
  }

  CONTROLLER_MODULE_API mc_control::MCController * create(const std::string &,
                                                          const mc_rbdyn::RobotModulePtr & robot,
                                                          const double & dt,
                                                          const mc_control::Configuration & conf)
  {
    return new Walking_controller(robot, dt, conf,
                                  mc_control::ControllerParameters{}.load_robot_config_into({}).overwrite_config(true));
  }
}
