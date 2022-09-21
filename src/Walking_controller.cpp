#include "../include/ismpc_walking/Walking_controller.h"

Walking_controller::~Walking_controller()
{
  MPC_thread_on = false;
  if(WalkingTrajectoryThread.joinable())
  {
    WalkingTrajectoryThread.join();
  }
}

Walking_controller::Walking_controller(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt,config)
{
  
  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration Stabiconfig(robot().module().defaultLIPMStabilizerConfiguration());
  if (config("stabilizer")("robot").has(robot().name()))
  {
    const auto & s_config = config("stabilizer")("robot")(robot().name())("stabilizer");
    Stabiconfig.load(s_config);
  }


  if(config.has("footsteps_planner"))
  {
    planner_config_ = config("footsteps_planner");
  }
  
  Controller_Config.Stab_config = Stabiconfig;
  Controller_Config.Controller_timestep = dt;

  MPCSolver = ISMPC_Solver(dt, Controller_Config.delta, Controller_Config.Tp, Controller_Config.Tc);

  Configure(Controller_Config);
  Configure(config);

  auto rConfig = config("walking_controller")("robot")(robot().name());
  rConfig("torsoBodyName", torsoBodyName_);
  rConfig("rightFootLink", RightFootLinkName_);
  rConfig("leftFootLink", LeftFootLinkName_);

  mc_rtc::log::info(robots().envIndex());
  controller_timestep = dt;
  // config_.load(config);
  // static auto constraint = mc_solver::ConstraintSetLoader::load(solver(), config("collisions")[0]);

  // datastore().make_call("KinematicAnchorFrame::" + robot().name(), [this] (const mc_rbdyn::Robot & robot) -> sva::PTransformd {
  //   sva::PTransformd X_0_leftfoot = robot.surfacePose("LeftFoot");
  //   sva::PTransformd X_0_rightfoot = robot.surfacePose("RightFoot");
  //   double height = robot.surfacePose(supportFootName).translation().z();
  //   Eigen::Vector3d T_0_leftfoot = X_0_leftfoot.translation();
  //   Eigen::Vector3d T_0_rightfoot = X_0_rightfoot.translation();
  //   sva::PTransformd X_0_leftfoot_floor(X_0_leftfoot.rotation(),Eigen::Vector3d{T_0_leftfoot.x(),T_0_leftfoot.y(),height});
  //   sva::PTransformd X_0_rightfoot_floor(X_0_rightfoot.rotation(),Eigen::Vector3d{T_0_rightfoot.x(),T_0_rightfoot.y(),height});

  //   return sva::interpolate(X_0_leftfoot_floor, X_0_rightfoot_floor, LeftFootRatio);
  
  // });

  datastore().make_call("KinematicAnchorFrame::" + robot().name(), [this](const mc_rbdyn::Robot & robot) {
    return sva::interpolate(robot.surfacePose("LeftFoot"), robot.surfacePose("RightFoot"), LeftFootRatio);
  });


  // solver().addConstraintSet(*constraint);
  // solver().addConstraintSet(contactConstraint);
  // solver().addConstraintSet(kinematicsConstraint);
  // solver().addConstraintSet(dynamicsConstraint);

  footcontact_dof << 0, 0, 1, 1, 1, 0;
  addContact({robot().name(), "ground", "RightFoot", "AllGround", 0.7, footcontact_dof});
  addContact({robot().name(), "ground", "LeftFoot", "AllGround", 0.7, footcontact_dof});

  rightShoulderIndex = robot().jointIndexByName("R_SHOULDER_P");
  rightLegIndex = robot().jointIndexByName("R_HIP_P");

  leftShoulderIndex = robot().jointIndexByName("L_SHOULDER_P");
  leftLegIndex = robot().jointIndexByName("L_HIP_P");

  leftSwingFootTask =
      std::make_shared<mc_tasks::SurfaceTransformTask>("LeftFootCenter", robots(), robots().robotIndex(), 10.0, 10.);

  rightSwingFootTask =
      std::make_shared<mc_tasks::SurfaceTransformTask>("RightFootCenter", robots(), robots().robotIndex(), 10.0, 10.);
 
  StabTask = std::make_shared<mc_tasks::lipm_stabilizer::StabilizerTask>(solver().robots(), solver().realRobots(),
                                                                         solver().robots().robotIndex(), solver().dt());

  StabTask->reset();
  StabTask->configure(Controller_Config.Stab_config);

  mc_rtc::log::info("com stiff {}",Controller_Config.Stab_config.comStiffness);

  armTask = std::make_shared<mc_tasks::PostureTask>(solver(), robots().robotIndex(), 100, 10);
  std::vector<std::string> armTask_Joints({"R_SHOULDER_P", "L_SHOULDER_P"});
  armTask->selectActiveJoints(solver(), armTask_Joints);
  armTask->name("ArmPosture");
  
  supportFootName = "RightFoot";
  swingFootName = "LeftFoot";

  SupportFootPose = robot().surfacePose(supportFootName+ "Center").translation();
  SupportFootPose.z() = 0;

  SwingFootInitialPose = robot().surfacePose(swingFootName+ "Center").translation();
  X_0_SwingFootInitial = SwingFootInitialPose;

  StaticPose = ((robot().surfacePose("LeftFoot").translation() + robot().surfacePose("RightFoot").translation()) / 2);
  StaticPose.z() = Controller_Config.Stab_config.comHeight;

  SwingFootAcc.setZero();
  SwingFootVel.setZero();

  create_datastore();
  getTransformations();

  bool start_now = config("walking_controller")("auto_start")("activate");
  if(start_now)
  {
    Stop = false;
    N_Steps_Desired = config("walking_controller")("auto_start")("steps");
    double t_step = config("walking_controller")("auto_start")("t_steps");
    ts(t_step);
    reference_velocity = config("walking_controller")("auto_start")("speed");
    
  }
  
  MPCSolver.Allow_none(Controller_Config.MPC_allow_None);



  solver().addTask(StabTask);
  solver().addTask(leftSwingFootTask);
  solver().addTask(rightSwingFootTask);
  solver().addTask(armTask);
  updateTasks();
  addToGUI();
  AddToLog();

  mc_rtc::log::success("ismpc_walking controller init done ");
}

void Walking_controller::wait_for_mpc_thread()
{
  if (!MPC_thread_on)
  {
    mc_rtc::log::info("waiting for footsteps_planner plugin");
    while (!datastore().has("footstep_planner::configure"))
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    // auto config_func = datastore().get<std::function<void(mc_rtc::Configuration const&)>>("footstep_planner::configure");
    // config_func(planner_config_);
    MPC_thread_on = true;
    ComputeTrajectoryOnce = true;
    WalkingTrajectory_Computing = true;
    WalkingTrajectoryThread = std::thread(&Walking_controller::WalkingTrajectoryLoop, this);
    WalkingTrajectoryThread.detach();

    mc_rtc::log::info("waiting for first computation");
    std::chrono::high_resolution_clock::time_point t_clock = std::chrono::high_resolution_clock::now();
    while(ComputeTrajectoryOnce)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      std::chrono::duration<double, std::milli> time_span = std::chrono::high_resolution_clock::now() - t_clock;
      if (time_span.count() > 5e3)
      {
        mc_rtc::log::error("Exiting waiting loop");
        WalkingTrajectory_Computing = false;
      }
    }
    mc_rtc::log::success("MPC thread on");
  }
}

void Walking_controller::WalkingTrajectoryLoop()
{
  while(MPC_thread_on)
  {

    ComputeWalkingTrajectory();
    
  }
}

void Walking_controller::ComputeWalkingTrajectory()
{
  
  WalkingTrajectory_Computing = true;
  if(ComputeTrajectoryOnce && datastore().has("footsteps_planner::planner_config"))
  {   
    std::chrono::high_resolution_clock::time_point t_clock = std::chrono::high_resolution_clock::now();

    {
      std::lock_guard<std::mutex> lk_copy_state(mutex_mpc_);
      UpdateInitialVectors();
      UpdatePlanner_input();
      mpc_thread_state = mpc_state_;
    }
    

    MPCSolver.AutoFootstepPlacement = AutoFootstepPlacement;


    if(mpc_thread_state.input_steps_.size() != 0)
    {
      N_Steps = 0;
      N_Steps_Desired = mpc_state_.input_steps_.size();
    }

    datastore().assign<std::vector<sva::MotionVecd>>("footsteps_planner::input_vel",mpc_thread_state.input_v_);
    datastore().assign<std::vector<sva::PTransformd>>("footsteps_planner::input_steps",mpc_thread_state.input_steps_);
    datastore().assign<std::string>("footsteps_planner::support_foot_name",mpc_thread_state.input_Support_FootName);
    datastore().assign<sva::PTransformd>("footsteps_planner::support_foot_pose",mpc_thread_state.X_0_SupportFoot);
    datastore().assign<std::vector<double>>("footsteps_planner::input_time_steps",mpc_thread_state.input_timesteps_);

    mc_rtc::DataStore & data_s(datastore());
    auto & lambda = datastore().get<std::function<void(mc_rtc::DataStore*)>>("footstep_planner::compute_plan");
    lambda(&data_s);

    std::vector<sva::PTransformd> & planned_steps_ = datastore().get<std::vector<sva::PTransformd>>("footsteps_planner::output_steps");
    // for (int k = 0 ; k < planned_steps_.size() ; k++)
    // {
    //   std::cout << "step " << k << ": " << planned_steps_[k].translation() << std::endl;
    // }
    std::vector<double> & timesteps = datastore().get<std::vector<double>>("footsteps_planner::output_time_steps");
    // mc_rtc::log::info("tds by ratio {}",Tds_by_ratio);
    double tds = Controller_Config.Double_Step_Ratio * timesteps[0];
    if ( !Tds_by_ratio)
    {
      tds = mpc_thread_state.input_tds;
    }
    int Steps = N_Steps;
    int Steps_Desired = N_Steps_Desired;
    if(Stop && !Swing_Foot_Contact)
    {
      Steps_Desired = Steps + 1;
    }

    std::string tail = Tail;
    if(mpc_thread_state.Index > 10. * MPCSolver.delta_mpc() / MPCSolver.delta_control())
    {

      tail = "None";
      mc_rtc::log::warning("[ISMPC] Approaching Control Horizon, Tail temporary switched to None");
    }

    MPCSolver.init_MPC(mpc_thread_state , planned_steps_, timesteps , Tail , Steps_Desired,Steps);
    // MPCSolver.Puk(mpc_state_.Pu);

  
    if (Use_w){
      MPCSolver.Disturbance(mpc_thread_state.w);
    }
    else{MPCSolver.Disturbance(Eigen::Vector3d::Zero());}

    MPCSolver.GetWalkingParameters(mpc_thread_state.t_k, tds,mpc_thread_state.stop);

    std::chrono::duration<double, std::milli> time_span = std::chrono::high_resolution_clock::now() - t_clock;
    mpc_thread_process_time = time_span.count();

    if(MPCSolver.QPsucceeded())
    {
      std::lock_guard<std::mutex> lk_copy_state(mutex_mpc_);
      mpc_thread_state.tds = tds;
      mpc_thread_state.TimeStamps = timesteps;
      mpc_thread_state.planned_steps_ = planned_steps_;
      mpc_thread_state.opti_steps = MPCSolver.optimal_steps();
      mpc_thread_state.QPSuccess = true;
      mpc_thread_state.X_MPC = MPCSolver.X_MPC();
      mpc_thread_state.Y_MPC = MPCSolver.Y_MPC();
      mpc_thread_state.Index = static_cast<int>(mpc_thread_process_time * 1e-3 / controller_timestep);
      mpc_thread_state.SupPolygon = MPCSolver.get_polynome_support();
      mpc_thread_state.Traj_ant = MPCSolver.GetAfterTc_ZMP_trajectory();
      mpc_thread_state.Tail = MPCSolver.Tail() != "None";
      mpc_thread_state.stab_error = MPCSolver.stability_error();
      mpc_thread_state.Pu_max = MPCSolver.Puk_max().segment(0, 2);
      mpc_thread_state.Pu_min = MPCSolver.Puk_min().segment(0, 2);


      kfoot = 0;
      NewThreadState = true;
    }
    else
    {
      mpc_state_.Pu_max = MPCSolver.Puk_max().segment(0, 2);
      mpc_state_.Pu_min = MPCSolver.Puk_min().segment(0, 2);
      mpc_state_.SupPolygon = MPCSolver.get_polynome_support();
      mpc_state_.QPSuccess = false;
    }

    ComputeTrajectoryOnce = false;

  }
  else
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(2)); 
  }
  
  WalkingTrajectory_Computing = false;
}

void Walking_controller::UpdatePlanner_input()
{
  mpc_state_.input_v_.clear();
  if(supportFootName == "LeftFoot")
  {
    reference_velocity.y() = mc_filter::utils::clamp(reference_velocity.y(), -0.07, 0.0);
  }
  else
  {
    reference_velocity.y() = mc_filter::utils::clamp(reference_velocity.y(), 0.0, 0.07);
  }
  for(int k = 0; k < (int)2 * std::round(Controller_Config.Tp / Controller_Config.delta); k++)
  {
    mpc_state_.input_v_.push_back(sva::MotionVecd(Eigen::Vector3d{0,0,reference_velocity.z()} , Eigen::Vector3d{reference_velocity.x(),reference_velocity.y(),0}));
  }
  
  mpc_state_.input_timesteps_ = {T_Steps , 2 * T_Steps};
  mpc_state_.set_input_tds(input_tds);
  mpc_state_.input_steps_.clear();
  // mpc._state_.input_Pf.push_back(Step_Target);
  mpc_state_.input_Support_FootName = supportFootName;
  mpc_state_.X_0_SupportFoot = sva::PTransformd(sva::RotZ(mc_rbdyn::rpyFromMat(robot().surfacePose(supportFootName+ "Center").rotation()).z()) ,
                                                robot().surfacePose(supportFootName+ "Center").translation());
  mpc_state_.X_0_Initial_SwingFoot = sva::PTransformd(sva::RotZ(mc_rbdyn::rpyFromMat(robot().surfacePose(swingFootName+ "Center").rotation()).z()) ,
                                                      robot().surfacePose(swingFootName+ "Center").translation());
  mpc_state_.SupportFootPose = robot().surfacePose(supportFootName+ "Center").translation();
  mpc_state_.SupportFootPose.z() = mc_rbdyn::rpyFromMat(robot().surfacePose(supportFootName+ "Center").rotation()).z();

  Eigen::Vector3d Pf_m1(SwingFootInitialPose);
  Pf_m1.z() = SupportFootPose.z();
  mpc_state_.input_P_fm1 = Pf_m1;
  mpc_state_.stop = !Robot_Walking;
}

bool Walking_controller::run()
{
  wait_for_mpc_thread();

  std::chrono::duration<double, std::milli> time_span = std::chrono::high_resolution_clock::now() - t_clock;
  ControllerLoopTime = time_span.count();
  t_clock = std::chrono::high_resolution_clock::now();
  if(NewThreadState)
  {
    std::lock_guard<std::mutex> lk_copy_state(mutex_mpc_);
    mpc_state_ = mpc_thread_state;
    NewThreadState = false;
  }

  if(emergencyFlag) return false;

  t = (count - countStart) * controller_timestep;

  getTransformations();

  if(!(Stop && Swing_Foot_Contact))
  {
    // if(!Robot_Walking)
    // {
    //   StabTask->comStiffness(Controller_Config.Stab_config.comStiffness);
    // }
    if(t - t_k >= Controller_Config.delta)
    {
      ComputeTrajectoryOnce = true;
      std::lock_guard<std::mutex> lk_copy_state(mutex_mpc_);
      t_k += Controller_Config.delta  ;
    }
    MoveCoM(t);
    MoveFeet(t);

    Robot_Walking = true;
  }
  else
  {
    // if (Robot_Walking)
    // {
    //   count = 0;
    //   StaticPose = ((robot().surfacePose("LeftFoot").translation() + robot().surfacePose("RightFoot").translation()) / 2);
    //   StaticPose.z() = Controller_Config.Stab_config.comHeight;
    //   StabTask->staticTarget(StaticPose);
    // }
    MoveFeet(0);
    MoveCoM(0);
    updateTasks();

    t_stop = (count - count_stop) * controller_timestep;
    if(t_stop > 1 * Controller_Config.delta)
    {
      ComputeTrajectoryOnce = true;
      count_stop = count - 1;
    }

    // double ratio = (count * controller_timestep - linearStiffTimeThreshold_)
    //                 / (maxStiffTimeThreshold_ - linearStiffTimeThreshold_);
    // mc_filter::utils::clampInPlace(ratio, 0, 1);
    // StabTask->comStiffness(Eigen::Vector3d::Ones() * (minStiffness_ + ratio * (maxStiffness_ - minStiffness_)));


    t_k = 0;
    kfoot = 0;
    N_Steps = 0;
    countStart = count - 1;

    Robot_Walking = false;
  }

  //{ ARM SWING

  currentLeftLeg = robot().mbc().q[leftLegIndex][0];
  currentRightLeg = robot().mbc().q[rightLegIndex][0];

  auto arm_posture = robot().mbc().q;
  arm_posture = armTask->posture();

  arm_posture[rightShoulderIndex][0] = 0.5 * currentLeftLeg - currentRightLeg;
  arm_posture[leftShoulderIndex][0] = 0.5 * currentRightLeg - currentLeftLeg;
  armTask->posture(arm_posture);

  //} ARM SWING

  count += 1;


  gui()->removeCategory({"Walking", "Visualization", "FootStep"});
  add_FootSteps_GUI();
  bool ret = mc_control::fsm::Controller::run();

  return ret;
}

void Walking_controller::MoveCoM(double t)
{

  // mc_rtc::log::info("//Index : " + std::to_string(Index));

  if(mpc_state_.Index + 1 >= mpc_state_.X_MPC.size())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Control Horizon reached");
  }

  Eigen::Vector3d Pcom(mpc_state_.Get_CoM_planarTarget(mpc_state_.Index));
  Pcom.z() = Controller_Config.Stab_config.comHeight + X_0_support.translation().z();
  zmpTarget = mpc_state_.Get_ZMP_planarTarget(mpc_state_.Index);

  Eigen::Vector3d Vc(mpc_state_.Get_CoMVel_planarTarget(mpc_state_.Index));

  const Eigen::Vector3d & robot_vc = robot().comVelocity();
  const Eigen::Vector3d & realrobot_vc = realRobot().comVelocity();

  double diffV_in = std::abs((Vc - robot_vc).y());


  Eigen::Vector3d Ac = std::pow(eta(), 2) * (Pcom - zmpTarget);

  Ac.z() = 0;
  dcmTarget = Pcom + Vc / eta();

  StabTask->target(Pcom, Vc, Ac, zmpTarget);

  {
    std::lock_guard<std::mutex> lk_copy_state(mutex_mpc_);
    mpc_state_.Index += 1;
  }
}

void Walking_controller::UpdateInitialVectors()
{
  
  mpc_state_.t_k = t_k;
  // mpc_state_.Pzk = Eigen::Vector3d{0,0,1}.cross( robot().com().cross(robot().mass()*mc_rtc::constants::gravity) ) /
  //                       ( (robot().mass()*(mc_rtc::constants::gravity - robot().comAcceleration())).transpose() *
  //                       Eigen::Vector3d{0,0,1} );

  if(UseMPCState && mpc_state_.X_MPC.size() != 0)
  {
    mpc_state_.Pck = mpc_state_.Get_CoM_planarTarget(mpc_state_.Index);
    mpc_state_.Vck = mpc_state_.Get_CoMVel_planarTarget(mpc_state_.Index);
    mpc_state_.Pzk = mpc_state_.Get_ZMP_planarTarget(mpc_state_.Index);
    mpc_state_.Pu = mpc_state_.Pck + mpc_state_.Vck/eta(); 
    // std::cout << "using MPC" << std::endl;
  }
  else if(UseRealRobot)
  {
    if (DoubleSupport_state && true)
    {
      double K = 0;
      // Pck = Pck * (1 - K) + K * computeInSupportFootFlat(realRobot().com());
      // Vck = Vck * (1 - K) + K * computeVelocityInSupportFoot(realRobot().comVelocity());
      mpc_state_.Pzk = mpc_state_.Get_ZMP_planarTarget(mpc_state_.Index) * (1 - K) + K * computeInSupportFootFlat(computeZMP());
      K = K_feedback;
      mpc_state_.Pck = robot().com() * (1 - K) + K * realRobot().com();
      mpc_state_.Vck = robot().comVelocity() * (1 - K) + K * realRobot().comVelocity();

      mpc_state_.Pu = StabTask->measuredDCM();
    }
    else
    {
      mpc_state_.Pck = mpc_state_.Get_CoM_planarTarget(mpc_state_.Index);
      mpc_state_.Vck = mpc_state_.Get_CoMVel_planarTarget(mpc_state_.Index);
      // mpc_state_.Pzk = mpc_state_.Get_ZMP_planarTarget(indx);      
    }


  }
  else
  {
    mpc_state_.Pck = robot().com();
    mpc_state_.Vck = robot().comVelocity();
    // mpc_state_.Pzk = mpc_state_.Get_ZMP_planarTarget(indx);
    mpc_state_.Pu = mpc_state_.Pck + mpc_state_.Vck/eta(); 
  }
  

  if(mpc_state_.X_MPC.size() != 0)
  {

    mpc_state_.Pzk = mpc_state_.Get_ZMP_planarTarget(mpc_state_.Index);
    
  }
  else
  {
    mpc_state_.Pzk = mpc_state_.Pck;
  }

  mpc_state_.w = - (StabTask->measuredDCM() - mpc_state_.Pu) + - (StabTask->measuredZMP() - mpc_state_.Pzk); 

  mpc_state_.Pck.z() = Controller_Config.Stab_config.comHeight;
  mpc_state_.Vck.z() = 0;
  mpc_state_.Pzk.z() = 0;
}

void Walking_controller::reset(const mc_control::ControllerResetData & reset_data)
{

  SwingFootTask.reset();
  SupportFootTask.reset();
  supportFootName = "RightFoot";
  swingFootName = "LeftFoot";

  mpc_state_.input_v_.clear();
  mpc_state_.input_timesteps_.clear();
  mpc_state_.input_steps_.clear();


  SupportFootPose = robot().surfacePose(supportFootName + "Center").translation();
  SupportFootPose.z() = 0;

  mpc_state_.Pck = robot().com();
  mpc_state_.Pck.z() = Controller_Config.Stab_config.comHeight;
  mpc_state_.Pzk = robot().surfacePose(swingFootName+ "Center").translation();
  mpc_state_.Pzk.z() = 0;
  mpc_state_.Pu = mpc_state_.Pck;
  mpc_state_.Vck = robot().comVelocity();

  SwingFootInitialPose = robot().surfacePose(swingFootName+ "Center").translation();
  X_0_SwingFootInitial = SwingFootInitialPose;
  updateTasks();

  addContact({robot().name(), "ground", "RightFoot", "AllGround", 0.7, footcontact_dof});
  addContact({robot().name(), "ground", "LeftFoot", "AllGround", 0.7, footcontact_dof});

  MPC_thread_on = false;
  if (WalkingTrajectoryThread.joinable())
  {
    WalkingTrajectoryThread.join();
  }

  mc_control::fsm::Controller::reset(reset_data);


};

