#include "../include/ismpc_walking/Walking_controller.h"

Walking_controller::~Walking_controller()
{
  MPC_thread_on = false;
  if(WalkingTrajectoryThread.joinable())
  {
    compute_trajectory_once.notify_all();
    WalkingTrajectoryThread.join();
  }
}

Walking_controller::Walking_controller(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config), filter_left_hand_wrench_(0.005, 0), filter_right_hand_wrench_(0.005, 0), filter_gamma_(0.005, 0), zmp_vel_(0.005,0.05,{0.,0.,0.})
{

  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration Stabiconfig(robot().module().defaultLIPMStabilizerConfiguration());
  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration Stabiconfig_standing(
      robot().module().defaultLIPMStabilizerConfiguration());
  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration Stabiconfig_sg_supp(
      robot().module().defaultLIPMStabilizerConfiguration());
  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration Stabiconfig_dbl_supp(
      robot().module().defaultLIPMStabilizerConfiguration());
  if(config("stabilizer")("robot").has(robot().name()))
  {
    const auto & s_config = config("stabilizer")("robot")(robot().name())("stabilizer");

    Stabiconfig_standing.load(s_config);
    Stabiconfig_sg_supp.load(s_config);
    Stabiconfig_dbl_supp.load(s_config);

    if(config("stabilizer")("robot")(robot().name()).has("stabilizer_sgsupp"))
    {
      const auto & s_config_sg = config("stabilizer")("robot")(robot().name())("stabilizer_sgsupp");
      Stabiconfig_sg_supp.load(s_config_sg);
    }
    if(config("stabilizer")("robot")(robot().name()).has("stabilizer_dblsupp"))   
    {
      const auto & s_config_dbl = config("stabilizer")("robot")(robot().name())("stabilizer_dblsupp");
      Stabiconfig_dbl_supp.load(s_config_dbl);
    }
  }

  if(config.has("footsteps_planner"))
  {
    planner_config_ = config("footsteps_planner");
  }

  controller_config_.Stab_config = Stabiconfig_standing;
  controller_config_.Stab_config_standing = Stabiconfig_standing;
  controller_config_.Stab_config_sg_supp = Stabiconfig_sg_supp;
  controller_config_.Stab_config_dbl_supp = Stabiconfig_dbl_supp;
  controller_config_.Controller_timestep = dt;

  MPCSolver = ISMPC_Solver(dt, controller_config_.delta, controller_config_.Tp, controller_config_.Tc);

  Configure(controller_config_);
  Configure(config);

  auto rConfig = config("walking_controller")("robot")(robot().name());
  rConfig("torsoBodyName", torsoBodyName_);
  rConfig("rightFootLink", RightFootLinkName_);
  rConfig("leftFootLink", LeftFootLinkName_);
  rConfig("left_foot_surface", leftFootName_);
  rConfig("right_foot_surface", rightFootName_);
  rConfig("left_hand_surface", leftHandName_);
  rConfig("right_hand_surface", rightHandName_);

  mc_rtc::log::info(robots().envIndex());
  controller_timestep = dt;
  // config_.load(config);
  // static auto constraint = mc_solver::ConstraintSetLoader::load(solver(), config("collisions")[0]);

  datastore().make_call("KinematicAnchorFrame::" + robot().name(), [this](const mc_rbdyn::Robot & robot) {
    return sva::interpolate(robot.surfacePose(leftFootName_), robot.surfacePose(rightFootName_), LeftFootRatio);
  });

  const auto oConfig = config("ObserverPipelines")("observers");
  for (auto conf : oConfig)
  {
    
    if (conf("type") == "KinematicInertial")
    {
      std::cout << (conf("config").dump()) << std::endl;
      const auto conf_obs = conf("config")("anchorFrame");
      conf_obs("maxAnchorFrameDiscontinuity",maxRatioDelta);
    }
  }
  

  // solver().addConstraintSet(*constraint);
  // solver().addConstraintSet(contactConstraint);
  // solver().addConstraintSet(kinematicsConstraint);
  // solver().addConstraintSet(dynamicsConstraint);

  footcontact_dof << 0, 0, 1, 0, 0, 0;
  addContact({robot().name(), "ground", rightFootName_, "AllGround", 0.7, footcontact_dof});
  addContact({robot().name(), "ground", leftFootName_, "AllGround", 0.7, footcontact_dof});

  leftSwingFootTask =
      std::make_shared<mc_tasks::SurfaceTransformTask>(leftFootName_, robots(), robots().robotIndex(), 10.0, 10.);

  rightSwingFootTask =
      std::make_shared<mc_tasks::SurfaceTransformTask>(rightFootName_, robots(), robots().robotIndex(), 10.0, 10.);

  sva::ForceVecd landingAdmittance = sva::ForceVecd(Eigen::Vector3d{0.03,0.03,0} , Eigen::Vector3d{0,0,0.003});
  leftLandingTask =
  std::make_shared<mc_tasks::force::CoPTask>(leftFootName_, robots(), robots().robotIndex(), 1, controller_config_.Stab_config.contactWeight);
  leftLandingTask->name("landingTask_left");
  leftLandingTask->admittance( landingAdmittance );
  leftLandingTask->damping(controller_config_.Stab_config.contactDamping);
  
  rightLandingTask =
  std::make_shared<mc_tasks::force::CoPTask>(rightFootName_, robots(), robots().robotIndex(), 1, controller_config_.Stab_config.contactWeight);
  rightLandingTask->name("landingTask_right");
  rightLandingTask->admittance( landingAdmittance );
  rightLandingTask->damping(controller_config_.Stab_config.contactDamping);

  swingFootName = leftFootName_;
  supportFootName = rightFootName_;

  MomentumTask =
      std::make_shared<mc_tasks::MomentumTask>(robots(),robot().robotIndex(),0,10);
  Eigen::Vector6d MomentumTask_dof;
  MomentumTask_dof << 1,1,1,0,0,0;
  MomentumTask->dimWeight(MomentumTask_dof);  

  comTask =
      std::make_shared<mc_tasks::CoMTask>(robots(),robot().robotIndex(),10,200);
   
  
  if(rConfig.has("momemtum_task_joints"))
  {
    MomentumTask->selectActiveJoints(solver(),rConfig("momemtum_task_joints"));
  }

  stabTask = std::make_shared<mc_tasks::lipm_stabilizer::StabilizerTask>(solver().robots(), solver().realRobots(),
                                                                         solver().robots().robotIndex(), solver().dt());

  mc_rtc::log::info("com stiff {}", controller_config_.Stab_config.comStiffness);


  SupportFootPose = robot().surfacePose(supportFootName).translation();
  SupportFootPose.z() = 0;

  SwingFootInitialPose = robot().surfacePose(swingFootName).translation();
  X_0_SwingFootInitial = SwingFootInitialPose;

  StaticPose =
      ((robot().surfacePose(leftFootName_).translation() + robot().surfacePose(rightFootName_).translation()) / 2);
  StaticPose.z() = controller_config_.Stab_config.comHeight;

  SwingFootAcc.setZero();
  SwingFootVel.setZero();

  filter_left_hand_wrench_ = mc_filter::LowPass<sva::ForceVecd>(solver().dt(),controller_config_.wrench_filter_cutoff);
  filter_right_hand_wrench_ = mc_filter::LowPass<sva::ForceVecd>(solver().dt(),controller_config_.wrench_filter_cutoff);
  filter_gamma_ = mc_filter::LowPass<Eigen::Vector3d>(solver().dt(),controller_config_.gamma_filter_cutoff);

  zmp_vel_ = mc_filter::ExponentialMovingAverage<Eigen::Vector3d>(solver().dt(),controller_config_.delta,Eigen::Vector3d::Zero());

  create_datastore();
  getTransformations();

  bool start_now = config("walking_controller")("auto_start")("activate");
  reference_velocity.setZero();
  if(start_now)
  {
    Stop = false;
    N_Steps_Desired = config("walking_controller")("auto_start")("steps");
    double t_step = config("walking_controller")("auto_start")("t_steps");
    ts(t_step);
    reference_velocity = config("walking_controller")("auto_start")("speed");
  }

  MPCSolver.Allow_none(controller_config_.MPC_allow_None);

  solver().addTask(stabTask);
  solver().addTask(comTask);
  solver().addTask(leftSwingFootTask);
  solver().addTask(rightSwingFootTask);
  solver().addTask(MomentumTask);
  updateTasks();
  deactivate();
  mc_rtc::log::success("ismpc_walking controller init done ");
}

bool Walking_controller::wait_for_mpc_thread()
{
  if(!MPC_thread_ready)
  {
    // if(!datastore().has("footstep_planner::configure"))
    // {
    //   mc_rtc::log::info("waiting for footsteps_planner plugin");
    //   return false;
    // }
    if(!MPC_thread_on)
    {
      mc_rtc::log::info("Start MPC thread");
      UpdateInitialVectors();
      UpdatePlanner_input();
      WalkingTrajectory_Computing = true;
      MPC_thread_on = true;
      WalkingTrajectoryThread = std::thread(&Walking_controller::WalkingTrajectoryLoop, this);
      auto th_handle = WalkingTrajectoryThread.native_handle();
      int policy = 0;
      sched_param param{};
      pthread_getschedparam(th_handle, &policy, &param);
      mc_rtc::log::info("MPC thread priority: {}", param.sched_priority);
      if(param.sched_priority > 90)
      {
        param.sched_priority = 80;
        pthread_setschedparam(th_handle, SCHED_RR, &param);
      }

      compute_trajectory_once.notify_all();
    }
    else
    {
      if(!WalkingTrajectory_Computing)
      {
        MPC_thread_ready = true;
        mc_rtc::log::success("MPC thread on");
        add_ISMPC_Config_GUI();
        addToGUI();
        add_FootSteps_GUI();
        Stabilizer_GUI(controller_config_.Stab_config_sg_supp, "single support");
        Stabilizer_GUI(controller_config_.Stab_config_dbl_supp, "double support");
        Stabilizer_GUI(controller_config_.Stab_config_standing, "standing");
        AddToLog();
      }
      else
      {
        mc_rtc::log::info("Waiting for first computation");
      }
    }
  }
  return MPC_thread_ready;
}

void Walking_controller::WalkingTrajectoryLoop()
{
  do
  {
    WalkingTrajectory_Computing = true;
    ComputeWalkingTrajectory();
    WalkingTrajectory_Computing = false;
    std::unique_lock<std::mutex> lock(compute_trajectory_once_mtx);
    compute_trajectory_once.wait(lock);
  } while(MPC_thread_on);
}

void Walking_controller::ComputeWalkingTrajectory()
{
  mc_rtc::clock::time_point t_clock = mc_rtc::clock::now();

  {
    std::lock_guard<std::mutex> lk_copy_state(mutex_mpc_);
    UpdateInitialVectors();
    mpc_thread_state = mpc_state_;
  }
  if(NewConfigState)
  {
    MPCSolver.configure(controller_config_);
    NewConfigState = false;
  }
  MPCSolver.AutoFootstepPlacement = AutoFootstepPlacement;
  MPCSolver.UsePendulumSolver = UsePendulumSolver;
  MPCSolver.UseAngularMomentumDot = UseAngularMomentum;

  if(mpc_thread_state.input_steps_.size() != 0)
  {
    N_Steps = 0;
    N_Steps_Desired = mpc_state_.input_steps_.size();
  }

  datastore().assign<std::vector<sva::MotionVecd>>("footsteps_planner::input_vel", mpc_thread_state.input_v_);
  datastore().assign<std::vector<sva::PTransformd>>("footsteps_planner::input_steps", mpc_thread_state.input_steps_);
  datastore().assign<std::string>("footsteps_planner::support_foot_name", mpc_thread_state.input_Support_FootName);
  datastore().assign<sva::PTransformd>("footsteps_planner::support_foot_pose", mpc_thread_state.X_0_SupportFoot);
  datastore().assign<std::vector<double>>("footsteps_planner::input_time_steps", mpc_thread_state.input_timesteps_);
  datastore().call("footstep_planner::compute_plan");

  mpc_thread_state.planned_steps_ =
        datastore().get<std::vector<sva::PTransformd>>("footsteps_planner::output_steps");
  // for (int k = 0 ; k < planned_steps_.size() ; k++)
  // {
  //   std::cout << "step " << k << ": " << planned_steps_[k].translation() << std::endl;
  // }
  mpc_thread_state.planned_timesteps_ = datastore().get<std::vector<double>>("footsteps_planner::output_time_steps");
  // mc_rtc::log::info("tds by ratio {}",Tds_by_ratio);
  double tds = controller_config_.Double_Step_Ratio * mpc_thread_state.planned_timesteps_[0];
  if(!Tds_by_ratio)
  {
    tds = mpc_thread_state.input_tds;
  }
  mpc_thread_state.tds = tds;
  int Steps = N_Steps;
  int Steps_Desired = N_Steps_Desired;

  if(Stop && !DoubleSupport_state)
  {
    Steps_Desired = Steps + 1;
  }

  // std::string tail = Tail;
  // if(mpc_thread_state.Index > 10. * MPCSolver.delta_mpc() / MPCSolver.delta_control())
  // {

  //   tail = "None";
  //   mc_rtc::log::warning("[ISMPC] Approaching Control Horizon, Tail temporary switched to None");
  // }

  MPCSolver.init_MPC(mpc_thread_state, Tail, Steps_Desired, Steps);
  // MPCSolver.Puk(mpc_state_.Pu);

  if(Use_w)
  {

    mc_filter::utils::clampInPlaceAndWarn(w_.x(),-0.05 , 0.05,"Perturbation (0)");
    mc_filter::utils::clampInPlaceAndWarn(w_.y(),-0.1 , 0.1,"Perturbation (1)");
    mc_filter::utils::clampInPlaceAndWarn(eta2_cstr,2, 20,"Omega Perturbation");
    double t_perturbation = 0.1;
    if(DoubleSupport_state){t_perturbation = 100;}
    MPCSolver.Disturbance(w_,sqrt(eta2_cstr),t_perturbation);

  }

  MPCSolver.GetWalkingParameters(mpc_thread_state.stop);

  std::chrono::duration<double, std::milli> time_span = mc_rtc::clock::now() - t_clock;
  mpc_thread_process_time = time_span.count();

  if(MPCSolver.QPsucceeded())
  {
    std::lock_guard<std::mutex> lk_copy_state(mutex_mpc_);
    mpc_thread_state.optimal_tds = MPCSolver.Tds();
    mpc_thread_state.optimal_timesteps_ = MPCSolver.timesteps();
    mpc_thread_state.optimal_steps_ = MPCSolver.optimal_steps();
    mpc_thread_state.QPSuccess = true;
    mpc_thread_state.FeasibilityPolygonStandingSwitch = MPCSolver.feasibility_region_switched();
    mpc_thread_state.X_MPC = MPCSolver.X_MPC();
    mpc_thread_state.Y_MPC = MPCSolver.Y_MPC();
    mpc_thread_state.Index = static_cast<int>(mpc_thread_process_time * 1e-3 / controller_timestep);
    mpc_thread_state.SupPolygon = MPCSolver.get_polynome_support();
    mpc_thread_state.Traj_ant = MPCSolver.GetAfterTc_ZMP_trajectory();
    mpc_thread_state.Tail = MPCSolver.Tail() != "None";
    mpc_thread_state.stab_error = MPCSolver.stability_error();
    mpc_thread_state.Pu_max = MPCSolver.Puk_max().segment(0, 2);
    mpc_thread_state.Pu_min = MPCSolver.Puk_min().segment(0, 2);
    mpc_thread_state.mpc_u_ = MPCSolver.ZMP_vel();
    mpc_thread_state.initial_zmp_ = MPCSolver.Initial_ZMP();
    mpc_thread_state.standing_mode = MPCSolver.stop();
    mpc_thread_state.FeasibilityPolygon = MPCSolver.feasibility_region();
    mpc_thread_state.alpha = MPCSolver.support_state();
    mpc_thread_state.ref_zmp_ = MPCSolver.zmp_ref().segment(0,2);
    mpc_thread_state.admittance_ref_ = MPCSolver.admittance_references();
    mpc_thread_state.QP_zmp = MPCSolver.QP_zmp();
    mpc_thread_state.eta = MPCSolver.eta();
    mpc_thread_state.mpc_Lc_dot_ = MPCSolver.Lc_dot();
    kfoot = 0;
    NewThreadState = true;
    
  }
  else
  {
    mpc_state_.Pu_max = MPCSolver.Puk_max().segment(0, 2);
    mpc_state_.Pu_min = MPCSolver.Puk_min().segment(0, 2);
    mpc_state_.SupPolygon = MPCSolver.get_polynome_support();
    mpc_state_.mpc_Lc_dot_.setZero();
    mpc_state_.QPSuccess = false;
  }
}

void Walking_controller::UpdatePlanner_input()
{
  mpc_state_.input_v_.clear();
  Eigen::Vector3d step_velocity = reference_velocity;
  double step_time = T_Steps;

  // if(supportFootName == leftFootName_)
  // {
  //   step_velocity.y() = mc_filter::utils::clamp(step_velocity.y(), -0.07, 0.0);
  // }
  // else
  // {
  //   step_velocity.y() = mc_filter::utils::clamp(step_velocity.y(), 0.0, 0.07);
  // }
  for(int k = 0; k < static_cast<int>(std::round(controller_config_.Tp / controller_config_.delta)); k++)
  {
    mpc_state_.input_v_.push_back(sva::MotionVecd(Eigen::Vector3d{0, 0, step_velocity.z()},
                                                  Eigen::Vector3d{step_velocity.x(), step_velocity.y(), 0}));
  }

  mpc_state_.input_timesteps_ = {step_time};
  while(mpc_state_.input_timesteps_.back() <= controller_config_.Tp)
  {
    mpc_state_.input_timesteps_.push_back( static_cast<double>(( static_cast<int>(mpc_state_.input_timesteps_.size()) + 1) * step_time ));
  }
  // std::cout << "//" << std::endl;
  // for (int k = 0 ; k < mpc_state_.input_timesteps_.size() ; k++ )
  // {
  //   std::cout << mpc_state_.input_timesteps_[k] << std::endl;
  // }
  // mpc_state_.input_v_.clear();
  //mpc_state_.input_timesteps_.clear();
  mpc_state_.set_input_tds(input_tds);
  mpc_state_.input_steps_.clear();
  // mpc_state_.input_steps_.push_back(target_pose_);
  mpc_state_.input_Support_FootName = "LeftFoot";
  if(supportFootName == rightFootName_)
  {
    mpc_state_.input_Support_FootName = "RightFoot";
  }
  mpc_state_.X_0_SupportFoot =
      sva::PTransformd(sva::RotZ(mc_rbdyn::rpyFromMat(robot().surfacePose(supportFootName).rotation()).z()),
                       robot().surfacePose(supportFootName).translation());
  mpc_state_.X_0_Initial_SwingFoot =
      sva::PTransformd(sva::RotZ(mc_rbdyn::rpyFromMat(robot().surfacePose(swingFootName).rotation()).z()),
                       robot().surfacePose(swingFootName).translation());
  mpc_state_.SupportFootPose = robot().surfacePose(supportFootName).translation();
  mpc_state_.SupportFootPose.z() = mc_rbdyn::rpyFromMat(robot().surfacePose(supportFootName).rotation()).z();

  Eigen::Vector3d Pf_m1(SwingFootInitialPose);
  Pf_m1.z() = SupportFootPose.z();
  mpc_state_.input_P_fm1 = Pf_m1;
  mpc_state_.stop = !Robot_Walking;
  if(DebugMode){mpc_state_.stop = false;}
}

void Walking_controller::CheckStepRecovery()
{
  if(MPC_thread_ready)
  {
    Eigen::MatrixX2d normals = MPCSolver.standing_feasibility_polygone().normals();
    Eigen::VectorXd offset = MPCSolver.standing_feasibility_polygone().offsets();
    Eigen::Vector2d dcm = (realRobot().com() + (realRobot().comVelocity()/mpc_state_.eta)).segment(0,2) + stabTask->biasDCM();
    Eigen::VectorXd stability_check = normals * dcm - offset;
    bool ok = true;
    for (int i = 0 ; i < stability_check.rows() ; i++ )
    {
      if(stability_check[i] > 1e-3)
      {
        ok = false;
        mc_rtc::log::info("break on cstr {}\nstabi check\n{}",i,stability_check);
        break;
      }
    }
    normals = mpc_state_.FeasibilityPolygonStandingSwitch.normals();
    offset = mpc_state_.FeasibilityPolygonStandingSwitch.offsets();
    stability_check = normals * dcm - offset;
    bool ok_switch = true;
    for (int i = 0 ; i < stability_check.rows() ; i++ )
    {
      if(stability_check[i] > 1e-3)
      {
        ok_switch = false;
        mc_rtc::log::info("[Support foot switch] break on cstr {}\nstabi check\n{}",i,stability_check);
        break;
      }
    }
    if(!ok || !ok_switch)
    {
      mc_rtc::log::warning("Can't Stop, stepping");
      if(ok_switch)
      {
        SwitchFootSupport_manual();
        return;
      }

      // mc_rtc::log::info("Pu {} ; Pu max {}",stabTask->measuredDCM(),MPCSolver.Puk_max());
      // mc_rtc::log::info("Pu {} ; Pu min {}",stabTask->measuredDCM(),MPCSolver.Puk_min());      
      N_Steps_Desired = 2;   
   
      sva::PTransformd ff = robot().posW();
      if( (ff.rotation() * stabTask->measuredCoMd()).x() < 0 && !StepRecoveryState)
      {
        if( (ff.rotation() * (robot().frame(supportFootName).position().translation() - robot().frame(swingFootName).position().translation())).x() > 0)
        {
          SwitchFootSupport_manual();
        }

      }
      if( (ff.rotation() * stabTask->measuredCoMd()).x() > 0 && !StepRecoveryState)
      {
        if( (ff.rotation() * (robot().frame(supportFootName).position().translation() - robot().frame(swingFootName).position().translation())).x() < 0)
        {
          SwitchFootSupport_manual();
        }

      }
      StepRecoveryState = true;
      Stop = false;
    }
    else
    {
      StepRecoveryState = false;
    }
  }
}

bool Walking_controller::run()
{
  JoystickInputs();
  if(!wait_for_mpc_thread())
  {
    return mc_control::fsm::Controller::run();
  }

  std::chrono::duration<double, std::milli> time_span = mc_rtc::clock::now() - t_clock;
  ControllerLoopTime = time_span.count();
  t_clock = mc_rtc::clock::now();

  if(emergencyFlag) return false;

  t = (count - countStart) * controller_timestep;

  planes_.clear();

  getTransformations();

  {
    std::lock_guard<std::mutex> lk_copy_state(mutex_mpc_);
    if(NewThreadState)
    {
      mpc_state_ = mpc_thread_state;
      NewThreadState = false;
    }
    MoveCoM();
    // mc_rtc::log::info("indx {} for N {}",mpc_state_.Index,static_cast<int>(controller_config_.delta/controller_timestep));
    UpdateInitialVectors();
    UpdatePlanner_input();
    mpc_state_.Index += 1;
  }


  if(!(Stop && DoubleSupport_state))
  {

    if(t - t_k >= controller_config_.delta || DoubleSupport_state )
    {
      t_k += t - t_k; 
      compute_trajectory_once.notify_all();
    }
    
    MoveFeet(t);

    Robot_Walking = true;
  }
  else
  {
    if(active)
    {
      MoveFeet(0);
    }
    updateTasks();

    t_stop = (count - count_stop) * controller_timestep;
    if(UseRealRobot && mpc_state_.standing_mode)
    {
      if(UseStepRecovery){CheckStepRecovery();}
    }
    if(t_stop > controller_config_.delta || StepRecoveryState )
    {
      count_stop = count;
    
      compute_trajectory_once.notify_all();
    }
    // compute_trajectory_once.notify_all();


    t_k = - controller_config_.delta;
    kfoot = 0;
    N_Steps = 0;
    countStart = count + 1;

    Robot_Walking = false;
  }
  

  if(!DoubleSupport_state && stabilizer_state_ != StabilizerState::SingleSupport && active)
  {
    stabilizer_state_ = StabilizerState::SingleSupport;
    mc_rbdyn::lipm_stabilizer::StabilizerConfiguration config = controller_config_.Stab_config_sg_supp;
    controller_config_.lambda_ = controller_config_.lambda_sg_supp;
    comTask->weight(config.comWeight);
    comTask->stiffness(config.comStiffness);
    comTask->selectActiveJoints(solver(),config.comActiveJoints);
    config.comWeight = 0.;
    stabTask->configure(config);
    Configure(controller_config_);
    mc_rtc::log::info("configure sg");
  }
  else if(Robot_Walking && DoubleSupport_state && stabilizer_state_ != StabilizerState::DoubleSupport && active)
  {
    stabilizer_state_ = StabilizerState::DoubleSupport;
    mc_rbdyn::lipm_stabilizer::StabilizerConfiguration config = controller_config_.Stab_config_dbl_supp;
    controller_config_.lambda_ = controller_config_.lambda_dbl_supp;
    comTask->weight(config.comWeight);
    comTask->stiffness(config.comStiffness);
    comTask->selectActiveJoints(solver(),config.comActiveJoints);
    config.comWeight = 0.;
    stabTask->configure(config);
    Configure(controller_config_);
    mc_rtc::log::info("configure dbl");
  }
  else if(!Robot_Walking && stabilizer_active_ && stabilizer_state_ != StabilizerState::Standing && active)
  {
    stabilizer_state_ = StabilizerState::Standing;
    mc_rbdyn::lipm_stabilizer::StabilizerConfiguration config = controller_config_.Stab_config_standing;
    controller_config_.lambda_ = controller_config_.lambda_dbl_supp;
    comTask->weight(config.comWeight);
    comTask->stiffness(config.comStiffness);
    comTask->selectActiveJoints(solver(),config.comActiveJoints);
    config.comWeight = 0.;
    stabTask->configure(config);
    Configure(controller_config_);
    mc_rtc::log::info("configure std");
  }
  controller_config_.Stab_config = stabTask->config();

  
  count += 1;

  bool ret = mc_control::fsm::Controller::run();

  return ret;
}

void Walking_controller::MoveCoM()
{


  if(mpc_state_.Index + 1 >= mpc_state_.X_MPC.size())
  {
    
    if(!Robot_Walking)
    {
      if(active)
      {
        mc_rtc::log::error("Control Horizon reached");
        deactivate();
      }
    }
    else
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("Control Horizon reached");
    }
  }

  Eigen::Vector3d Pcom(mpc_state_.Get_CoM_planarTarget(mpc_state_.Index));
  Pcom.z() = controller_config_.Stab_config.comHeight + 0*X_0_support.translation().z();
  Eigen::Vector3d Vc(mpc_state_.Get_CoMVel_planarTarget(mpc_state_.Index));
  Vc.z() = 0;
  zmpTarget = mpc_state_.Get_ZMP_planarTarget(mpc_state_.Index);
  
  LcDotTarget = mpc_state_.get_Lc_dot(mpc_state_.Index);

    
  // mc_rtc::log::info("//Index : {}, z_y {}",mpc_state_.Index,zmpTarget.y());


  Eigen::Vector3d Ac_com = std::pow(mpc_state_.eta, 2) * (Pcom - zmpTarget);

  Ac_com.z() = 0;
  admittanceTarget = mpc_state_.initial_zmp_;
  int n = static_cast<int>(controller_config_.delta/controller_timestep);
  for (int k = 0 ; k <= mpc_state_.Index/n ; k++)
  {
    // mc_rtc::log::info("index {} , k {}",mpc_state_.Index,k);
    admittanceTarget += mpc_state_.get_u(k);
  }
  // mc_rtc::log::info("//");

  admittanceTarget.z() = 0;

  if(DoubleSupport_state && mpc_state_.get_tds() - t_k > 0 && mpc_state_.zmp_references().size() != 0)
  {
    int n_indx = static_cast<int>((mpc_state_.get_tds() - t_k) / controller_config_.delta);
    n_indx = std::max(std::min(n_indx,20),1);
    std::vector<Eigen::Vector2d> zmp_ref = mpc_state_.zmp_references();
    auto start_zmp = zmp_ref.begin();
    auto end_zmp = zmp_ref.begin()  + n_indx + 1;
 
    std::vector<Eigen::Vector2d> result_zmp(n_indx + 1);
 
    // Copy vector using copy function()
    std::copy(start_zmp, end_zmp, result_zmp.begin());
    stabTask->horizonReference(result_zmp, controller_config_.delta);
  }


  Eigen::Vector3d Ac_wrench = std::pow(mpc_state_.eta , 2) * (Pcom - admittanceTarget);
  Ac_wrench.z() = 0;

  target_force_ = robot().mass() * (Ac_wrench + mc_rtc::constants::gravity);
  target_wrench_ = sva::ForceVecd{Pcom.cross(target_force_),target_force_};

  // mc_rtc::log::info("zmp diff {}", admittanceTarget -  mc_rbdyn::zmp(target_wrench_, sva::PTransformd::Identity()) );

  Ac_wrench.z() = 0;
  dcmTarget = Pcom + Vc / mpc_state_.eta;

  stabTask->target(Pcom, Vc, Ac_wrench, admittanceTarget);
  if(!active || DebugMode)
  {
    Pcom.segment(0,2) = sva::interpolate(robot().surfacePose(leftFootName_),robot().surfacePose(rightFootName_),0.5).translation().segment(0,2);
    Vc.setZero();
    Ac_com.setZero();
    if(!Stop)
    {
      Stop = true;
      mc_rtc::log::warning("[Walking Controller] MPC control is off, cannot walk");
    }
    LcDotTarget.setZero();
  }
  comTask->com(Pcom);
  comTask->refVel(Vc);
  comTask->refAccel(Ac_com);
  sva::ForceVecd RealRobot_LcDot = rbd::computeCentroidalMomentumDot(realRobot().mb(), realRobot().mbc(), realRobot().com(),realRobot().comVelocity());
  MomentumTask->weight(0);
  if(UseAngularMomentum)
  {
    if(LcDotTarget.norm() > 1e-6)
    {
      MomentumTask->weight(1000);
    }
    MomentumTask->refAccel(sva::MotionVecd(LcDotTarget,Eigen::Vector3d::Zero()).vector());
  }

  mc_tasks::lipm_stabilizer::ContactState supportFoot = supportFootName == leftFootName_ ? mc_tasks::lipm_stabilizer::ContactState::Left : mc_tasks::lipm_stabilizer::ContactState::Right;
  
  stabTask->supportFoot(supportFoot);

}

void Walking_controller::UpdateInitialVectors()
{

  mpc_state_.t_k = t_k;
  mpc_state_.t_lift = t_lift;
  mpc_state_.doubleSupport = DoubleSupport_state;
  mpc_state_.t = static_cast<double>(count) * controller_timestep;

  Eigen::Vector3d FilteredNetForce = stabTask->measuredFilteredNetForces();
  mpc_state_.input_mass = FilteredNetForce.z() / mc_rtc::constants::GRAVITY;
   
  // mpc_state_.Pzk = Eigen::Vector3d{0,0,1}.cross( robot().com().cross(robot().mass()*mc_rtc::constants::gravity) ) /
  //                       ( (robot().mass()*(mc_rtc::constants::gravity - robot().comAcceleration())).transpose() *
  //                       Eigen::Vector3d{0,0,1} );

  if(DebugMode)
  {
    debugCoM.z() = controller_config_.Stab_config.comHeight;
    debugZMP.z() = 0;
    mpc_state_.Vck = Eigen::Vector3d::Zero();
    mpc_state_.Pck = debugCoM;
    mpc_state_.Pzk = debugZMP;
    mpc_state_.Pu = mpc_state_.Pck + mpc_state_.Vck / mpc_state_.eta;
    mpc_state_.t_k = debugTk;
    mpc_state_.doubleSupport = debugDblSupp;
    return;
  }



  if(UseMPCState && mpc_state_.X_MPC.size() != 0)
  {
    mpc_state_.Pck = mpc_state_.Get_CoM_planarTarget(mpc_state_.Index);
    mpc_state_.Vck = mpc_state_.Get_CoMVel_planarTarget(mpc_state_.Index);
    mpc_state_.Pzk = mpc_state_.Get_ZMP_planarTarget(mpc_state_.Index);
    mpc_state_.Pu = mpc_state_.Pck + mpc_state_.Vck / mpc_state_.eta;
    // std::cout << "using MPC" << std::endl;
  }
  else
  {
    mpc_state_.Pck = robot().com();
    mpc_state_.Vck = robot().comVelocity();
    // mpc_state_.Pzk = mpc_state_.Get_ZMP_planarTarget(indx);
    mpc_state_.Pu = mpc_state_.Pck + mpc_state_.Vck / mpc_state_.eta;
  }
  if(UseRealRobot)
  {
    
    sva::PTransformd zmpFrame = robot().surfacePose(supportFootName);
    sva::ForceVecd measuredNetWrench_ = robot().netWrench({"LeftFootForceSensor"});
    if(supportFootName == "RightFootCenter")
    {
      measuredNetWrench_ = robot().netWrench({"RightFootForceSensor"});
    }
    if(DoubleSupport_state)
    {
      measuredNetWrench_ = robot().netWrench({"RightFootForceSensor","LeftFootForceSensor"});
      zmpFrame = sva::interpolate(robot().surfacePose(supportFootName),robot().surfacePose(swingFootName), 0.5);
    }
    Eigen::Vector3d zmp_vel = mpc_state_.Pzk;
    robot().zmp(mpc_state_.Pzk,measuredNetWrench_,zmpFrame);
    zmp_vel = (mpc_state_.Pzk - zmp_vel) / controller_timestep;
    zmp_vel_.append(zmp_vel);

    // mpc_state_.ComBias 
    
    mpc_state_.Vck = realRobot().comVelocity();
    mpc_state_.ComBias.segment(0,2) = stabTask->biasDCM();
    mpc_state_.Pck = realRobot().com() + mpc_state_.ComBias;

    mpc_state_.Pu = mpc_state_.Pck + mpc_state_.Vck/mpc_state_.eta;
    if(controller_config_.Stab_config.dcmBias.withDCMFilter)
    {
      mpc_state_.Pu.segment(0,2) = -stabTask->filteredDCM();
      mpc_state_.Pck = mpc_state_.Pu - (mpc_state_.Vck / stabTask->omega());
    }

    Ldot = rbd::computeCentroidalMomentumDot(realRobot().mb(), realRobot().mbc(), mpc_state_.Pck ,mpc_state_.Vck).moment(); 

  }

  if(mpc_state_.X_MPC.size() != 0 && !UseRealRobot)
  {

    mpc_state_.Pzk = mpc_state_.Get_ZMP_planarTarget(mpc_state_.Index);
  }
  else if(!UseRealRobot)
  {
    mpc_state_.Pzk = mpc_state_.Pck;
  }

  ComputeFeetPerturbances(w_,eta2_cstr);
  Ldot_offset = Eigen::Vector3d{-Ldot.y(),Ldot.x(),0.};
  Ldot_offset /= (robot().mass() * controller_config_.Stab_config.comHeight * eta2_cstr);
  w_ += Ldot_offset;
  

  // eta2_cstr = (mc_rtc::constants::GRAVITY/controller_config_.Stab_config.comHeight);

  mpc_state_.Pck.z() = controller_config_.Stab_config.comHeight;
  mpc_state_.Vck.z() = 0;
  mpc_state_.Pzk.z() = 0;

  mpc_state_.Uk.setZero();
  if(mpc_state_.X_MPC.size() != 0)
  {
    mpc_state_.Uk = stabTask->distribZMP();
    
  }

}

void Walking_controller::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);

  stabTask->reset();
  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration config_stab = controller_config_.Stab_config;
  config_stab.comWeight = 0;
  stabTask->configure(config_stab);

  if(config()("stabilizer")("robot")(robot().name())("stabilizer").has("external_wrench"))
  {
      Eigen::Vector3d ext_wrench_gain_v = config()("stabilizer")("robot")(robot().name())("stabilizer")("external_wrench")("ext_wrench_gain");
      sva::MotionVecd ext_wrench_gain{ext_wrench_gain_v, ext_wrench_gain_v};
      stabTask->setExternalWrenches({leftHandName_, rightHandName_}, {sva::ForceVecd::Zero(), sva::ForceVecd::Zero()},
                                    {ext_wrench_gain, ext_wrench_gain});
  }

  
  SwingFootTask.reset();
  SupportFootTask.reset();
  supportFootName = rightFootName_;
  swingFootName = leftFootName_;

  mpc_state_.input_v_.clear();
  mpc_state_.input_timesteps_.clear();
  mpc_state_.input_steps_.clear();

  SupportFootPose = robot().surfacePose(supportFootName).translation();
  SupportFootPose.z() = 0;

  mpc_state_.Pck = robot().com();
  mpc_state_.Pck.z() = controller_config_.Stab_config.comHeight;
  mpc_state_.Pzk = robot().surfacePose(swingFootName).translation();
  mpc_state_.Pzk.z() = 0;
  mpc_state_.Pu = mpc_state_.Pck;
  mpc_state_.Vck = robot().comVelocity();

  filter_left_hand_wrench_ = mc_filter::LowPass<sva::ForceVecd>(solver().dt(),controller_config_.wrench_filter_cutoff);
  filter_right_hand_wrench_ = mc_filter::LowPass<sva::ForceVecd>(solver().dt(),controller_config_.wrench_filter_cutoff);
  filter_gamma_ = mc_filter::LowPass<Eigen::Vector3d>(solver().dt(),controller_config_.gamma_filter_cutoff);

  SwingFootInitialPose = robot().surfacePose(swingFootName).translation();
  X_0_SwingFootInitial = SwingFootInitialPose;
  updateTasks();

  addContact({robot().name(), "ground", rightFootName_, "AllGround", 0.7, footcontact_dof});
  addContact({robot().name(), "ground", leftFootName_, "AllGround", 0.7, footcontact_dof});

  MPC_thread_on = false;
  MPC_thread_ready = false;
  if(WalkingTrajectoryThread.joinable())
  {
    compute_trajectory_once.notify_all();
    WalkingTrajectoryThread.join();
  }
  deactivate();
}
