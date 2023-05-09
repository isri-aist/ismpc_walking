#include "../include/ismpc_walking/Walking_controller.h"

void Walking_controller::AddToLog()
{

  logger().addLogEntry("Contact point angular momentum / (m*H)", [this]() -> Eigen::Vector3d {
    return compute_momentum_contact_point().couple() / (robot().mass() * controller_config_.Stab_config.comHeight);
  });
  logger().addLogEntry("Kinematic LeftFoot ratio", [this]() -> double { return LeftFootRatio; });
  logger().addLogEntry("RealRobotCoM", [this]() -> Eigen::Vector3d { return stabTask->measuredCoM(); });
  logger().addLogEntry("SwingFoot Vel", [this]() -> const Eigen::Vector3d & { return SwingFootVel; });
  logger().addLogEntry("SwingFoot Accel", [this]() -> const Eigen::Vector3d & { return SwingFootAcc; });
  logger().addLogEntry("foot_wrench_LeftFoot",
                       [this]() -> const sva::ForceVecd { return leftSwingFootTask->frame().wrench(); });
  logger().addLogEntry("foot_wrench_RightFoot",
                       [this]() -> const sva::ForceVecd { return rightSwingFootTask->frame().wrench(); });
  logger().addLogEntry("foot_wrench_SwingFoot",
                       [this]() -> const sva::ForceVecd { return SwingFootTask->frame().wrench(); });
  logger().addLogEntry("foot_vertical_force_offset", [this]() -> const double & { return vertical_force_offset_; });
  // logger().addLogEntry("RealRobot LeftFoot Accel",
  //                      [this]() -> const Eigen::Vector3d & { return robot().bodyAccB(LeftFootLinkName_).linear(); });
  // logger().addLogEntry("RealRobot RightFoot Accel",
  //                      [this]() -> const Eigen::Vector3d & { return robot().bodyAccB(RightFootLinkName_).linear(); });

  logger().addLogEntry("ISMPC_measured_lambda",[this] () -> Eigen::Vector2d {return estimated_lambda();} );
  logger().addLogEntry("ISMPC_measured_zmpvel",[this] () -> Eigen::Vector2d {return zmp_vel_.eval().segment(0,2);} );
  logger().addLogEntry("ISMPC_NextTds", [this]() -> double { return mpc_state_.get_tds(); });
  logger().addLogEntry("ISMPC_NextTs", [this]() -> double { return mpc_state_.get_Ts(0); });
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
  logger().addLogEntry("ISMPC_CoMAccZ", [this]() -> const double & { return comAccZ; });
  logger().addLogEntry("ISMPC_stab-error", [this]() -> const Eigen::Vector2d & { return this->mpc_state_.stab_error; });
  logger().addLogEntry("perf_ISMPC", [this]() -> const double { return this->mpc_thread_process_time; });

  logger().addLogEntry("ISMPC_Target_Wrench", [this]() -> const sva::ForceVecd {

    return target_wrench_;
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
  logger().addLogEntry("ISMPC_State_Ld", [this]() -> const Eigen::Vector3d {
    return Ldot;
  });
  logger().addLogEntry("ISMPC_Target_ZMP", [this]() -> const Eigen::Vector3d {
    return zmpTarget;
  });
  logger().addLogEntry("ISMPC_Target_u", [this]() -> const Eigen::Vector3d {
    return admittanceTarget;
  });
  logger().addLogEntry("ISMPC_Target_LcDot", [this]() -> const Eigen::Vector3d {
    return LcDotTarget;
  });
  logger().addLogEntry("ISMPC_Target_u_delay", [this]() -> const Eigen::Vector3d {
    return MPCSolver.Uk();
  });
  logger().addLogEntry("ISMPC_Target_Index", [this]() -> const double {
    if(MPC_thread_on)
    {
      return mpc_state_.Index - 1;
    }
    return 0;
  });
  logger().addLogEntry("ISMPC_alpha", [this]() -> const double {
    return mpc_state_.alpha;
  });
  logger().addLogEntry("ISMPC_ref_zmp", [this]() -> const Eigen::Vector2d {
    return mpc_state_.ref_zmp_;
  });
  logger().addLogEntry("ISMPC_perturbation_offset", [this]() -> const Eigen::Vector2d  {
    return w_.segment(0,2);
  });
  logger().addLogEntry("ISMPC_perturbation_omega", [this]() -> const double  {
    return sqrt(eta2_cstr);
  });
  logger().addLogEntry("ISMPC_perturbation_kappa", [this]() -> const double  {
    return kappa_;
  });
  logger().addLogEntry("ISMPC_perturbation_Ldot/mHw2", [this]() -> const  Eigen::Vector3d & {
    return Ldot_offset;
  });

  // logger().addLogEntry("ISMPC_State_ZMP_kinmes", [this]() -> const Eigen::Vector3d {

  //   return Eigen::Vector3d{0,0,1}.cross( robot().com().cross(robot().mass()*mc_rtc::constants::gravity) ) /
  //                     ( (robot().mass()*(mc_rtc::constants::gravity - robot().comAcceleration())).transpose() *
  //                     Eigen::Vector3d{0,0,1} );
  // });
  logger().addLogEntry("ISMPC_State_DCM", [this]() -> Eigen::Vector3d {
    if(MPC_thread_on)
    {
      return  (mpc_state_.getPuk());
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
  logger().addLogEntry("ISMPC_zmp_delay", [this]() -> double {
    if(MPC_thread_on)
    {
      return  MPCSolver.zmp_delay();
    }
    return 0;
  });
  logger().addLogEntry("ISMPC_Feasibility_min", [this]() -> const Eigen::Vector2d & { return mpc_state_.Pu_min; });
  logger().addLogEntry("ISMPC_Feasibility_max", [this]() -> const Eigen::Vector2d & { return mpc_state_.Pu_max; });
  logger().addLogEntry("ISMPC_ControllerStop", [this]() -> double {
    if(Stop)
    {
      return 1.0;
    }
    else
    {
      return 0.;
    }
  });
  logger().addLogEntry("ISMPC_Standing", [this]() -> double {
    if(mpc_state_.standing_mode)
    {
      return 1.0;
    }
    else
    {
      return 0.;
    }
  });
  logger().addLogEntry("ISMPC_Tail-used", [this]() -> double {
    if(mpc_state_.Tail)
    {
      return 1.;
    }
    else
    {
      return 0.;
    }
  });
  logger().addLogEntry("ISMPC_QPSuccess", [this]() -> double {
    if(mpc_state_.QPSuccess)
    {
      return 1.;
    }
    else
    {
      return 0.;
    }
  });
  logger().addLogEntry("ISMPC_active", [this]() -> double {
    if(active)
    {
      return 1.0;
    }
    else
    {
      return 0.;
    }
  });
  logger().addLogEntry("ISMPC_DoubleSupport", [this]() -> double {
    if(mpc_state_.doubleSupport)
    {
      return 1.0;
    }
    else
    {
      return 0.;
    }
  });
}