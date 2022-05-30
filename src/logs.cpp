#include "Walking_controller.h"

void Walking_controller::AddToLog()
{

  logger().addLogEntry("tau", [this]() {
    const auto & rjo = robot().refJointOrder();
    std::vector<double> ret;
    ret.reserve(rjo.size());
    for(const auto & j : rjo)
    {
      ret.push_back(robot().mbc().jointTorque[robot().jointIndexByName(j)][0]);
    }
    return ret;
  });

  logger().addLogEntry("Kinematic LeftFoot ratio", [this]() { return LeftFootRatio; });
  logger().addLogEntry("SwingFoot Vel", [this]() -> const Eigen::Vector3d & { return SwingFootVel; });
  logger().addLogEntry("SwingFoot Accel", [this]() -> const Eigen::Vector3d & { return SwingFootAcc; });
  logger().addLogEntry("RealRobot LeftFoot Accel",
                       [this]() -> const Eigen::Vector3d & { return robot().bodyAccB(LeftFootLinkName_).linear(); });
  logger().addLogEntry("RealRobot RightFoot Accel",
                       [this]() -> const Eigen::Vector3d & { return robot().bodyAccB(RightFootLinkName_).linear(); });
  logger().addLogEntry("ISMPC_NextTs", [this]() -> const double & { return mpc_state_.input_timesteps_[0]; });
  logger().addLogEntry("ISMPC_t", [this]() -> const double & { return t; });
  logger().addLogEntry("ISMPC_stab-error", [this]() -> const double { return this->mpc_state_.stab_error; });
  logger().addLogEntry("ISMPC_State_CoM", [this]() -> const Eigen::Vector3d {
    return mpc_state_.Get_PlannedFootstep(0).rotation() * mpc_state_.getPck();
  });
  logger().addLogEntry("ISMPC_State_CoMd", [this]() -> const Eigen::Vector3d {
    return mpc_state_.Get_PlannedFootstep(0).rotation() * mpc_state_.getVck();
  });
  logger().addLogEntry("ISMPC_State_ZMP", [this]() -> const Eigen::Vector3d {
    return mpc_state_.Get_PlannedFootstep(0).rotation() * mpc_state_.getPzk();
  });
  logger().addLogEntry("ISMPC_State_DCM", [this]() -> Eigen::Vector3d {
    return mpc_state_.Get_PlannedFootstep(0).rotation() * (mpc_state_.Pck + mpc_state_.getVck() / eta());
  });
  logger().addLogEntry("ISMPC_Feasibility_min", [this]() -> const Eigen::Vector2d & { return mpc_state_.Pu_min; });
  logger().addLogEntry("ISMPC_Feasibility_max", [this]() -> const Eigen::Vector2d & { return mpc_state_.Pu_max; });
  logger().addLogEntry("ISMPC_StopPhase", [this]() -> const double {
    if(Stop)
    {
      return 1.0;
    }
    else
    {
      return 0.;
    }
  });
  logger().addLogEntry("ISMPC_Tail-used", [this]() -> const double {
    if(mpc_state_.Tail)
    {
      return 1.;
    }
    else
    {
      return 0.;
    }
  });
  logger().addLogEntry("ISMPC_QPSuccess", [this]() -> const double {
    if(mpc_state_.QPSuccess)
    {
      return 1.;
    }
    else
    {
      return 0.;
    }
  });
}