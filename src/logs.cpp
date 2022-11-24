#include "../include/ismpc_walking/Walking_controller.h"

void Walking_controller::AddToLog()
{

  logger().addLogEntry("Contact point angular momentum / (m*H)", [this]() -> Eigen::Vector3d {
    return compute_momentum_contact_point().couple() / (robot().mass() * controller_config_.Stab_config.comHeight);
  });
  logger().addLogEntry("Kinematic LeftFoot ratio", [this]() -> double { return LeftFootRatio; });
  logger().addLogEntry("RealRobotCoM", [this]() -> Eigen::Vector3d { return StabTask->measuredCoM(); });
  logger().addLogEntry("SwingFoot Vel", [this]() -> const Eigen::Vector3d & { return SwingFootVel; });
  logger().addLogEntry("SwingFoot Accel", [this]() -> const Eigen::Vector3d & { return SwingFootAcc; });
  logger().addLogEntry("foot_wrench_LeftFoot",
                       [this]() -> const sva::ForceVecd { return leftSwingFootTask->frame().wrench(); });
  logger().addLogEntry("foot_wrench_RightFoot",
                       [this]() -> const sva::ForceVecd { return rightSwingFootTask->frame().wrench(); });
  logger().addLogEntry("foot_wrench_SwingFoot",
                       [this]() -> const sva::ForceVecd { return SwingFootTask->frame().wrench(); });
  logger().addLogEntry("foot_vertical_force_offset", [this]() -> const double & { return vertical_force_offset_; });
  logger().addLogEntry("RealRobot LeftFoot Accel",
                       [this]() -> const Eigen::Vector3d & { return robot().bodyAccB(LeftFootLinkName_).linear(); });
  logger().addLogEntry("RealRobot RightFoot Accel",
                       [this]() -> const Eigen::Vector3d & { return robot().bodyAccB(RightFootLinkName_).linear(); });
  logger().addLogEntry("ISMPC_NextTs", [this]() -> double {
    std::vector<double> ts(mpc_state_.getTimeStamp());
    if(ts.size() != 0)
    {
      return ts[0];
    }
    return 0.;
  });
  logger().addLogEntry("ISMPC_NextTds", [this]() -> double { return mpc_state_.get_tds(); });
  logger().addLogEntry("ISMPC_input_tds", [this]() -> double { return mpc_state_.input_tds; });
  logger().addLogEntry("ISMPC_input_ts", [this]() -> double {
    std::vector<double> ts(mpc_state_.input_timesteps_);
    if(ts.size() != 0)
    {
      return ts[0];
    }
    return 0.;
  });
  logger().addLogEntry("ISMPC_t", [this]() -> const double & { return t; });
  logger().addLogEntry("ISMPC_stab-error", [this]() -> const double { return this->mpc_state_.stab_error; });
  logger().addLogEntry("ISMPC_process-time", [this]() -> const double { return this->mpc_thread_process_time; });

  logger().addLogEntry("ISMPC_TargetForce", [this]() -> const Eigen::Vector3d {

    return target_force_;
  });

  logger().addLogEntry("ISMPC_State_CoM", [this]() -> const Eigen::Vector3d {
    if(MPC_thread_on)
    {
      return mpc_state_.getPck();
    }
    return Eigen::Vector3d::Zero();
  });
  logger().addLogEntry("ISMPC_State_CoMd", [this]() -> const Eigen::Vector3d {
    if(MPC_thread_on)
    {
      return mpc_state_.getVck();
    }
    return Eigen::Vector3d::Zero();
  });
  logger().addLogEntry("ISMPC_State_ZMP", [this]() -> const Eigen::Vector3d {
    if(MPC_thread_on)
    {
      return mpc_state_.getPzk();
    }
    return Eigen::Vector3d::Zero();
  });
  logger().addLogEntry("ISMPC_Target_ZMP", [this]() -> const Eigen::Vector3d {
    if(MPC_thread_on)
    {
      return mpc_state_.Get_ZMP_planarTarget(mpc_state_.Index);
    }
  });
  // logger().addLogEntry("ISMPC_State_ZMP_kinmes", [this]() -> const Eigen::Vector3d {

  //   return Eigen::Vector3d{0,0,1}.cross( robot().com().cross(robot().mass()*mc_rtc::constants::gravity) ) /
  //                     ( (robot().mass()*(mc_rtc::constants::gravity - robot().comAcceleration())).transpose() *
  //                     Eigen::Vector3d{0,0,1} );
  // });
  logger().addLogEntry("ISMPC_State_DCM", [this]() -> Eigen::Vector3d {
    if(MPC_thread_on)
    {
      return  (mpc_state_.Pck + mpc_state_.getVck() / eta());
    }
    return Eigen::Vector3d::Zero();
  });
  logger().addLogEntry("ISMPC_lambda", [this]() -> double {
    if(MPC_thread_on)
    {
      return  MPCSolver.get_lambda();
    }
    return 0;
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