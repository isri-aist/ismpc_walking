#include "Walking_controller.h"
#include <mc_observers/KinematicInertialObserver.h>
#include <mc_observers/ObserverPipeline.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/logging.h>
#include <mc_solver/ConstraintSetLoader.h>
#include <chrono>
#include <unistd.h>

Walking_controller::Walking_controller(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{

  Controller_Config.Stab_config = robot().module().defaultLIPMStabilizerConfiguration();
  if(robot().name() == "hrp4cr")
  {
    Controller_Config.Stab_config.comActiveJoints.push_back("CHEST_R");
    Controller_Config.Stab_config.comActiveJoints.push_back("CHEST_P");
  }
  mc_rtc::log::info("Default CoM gain {}", Controller_Config.Stab_config.comStiffness);

  MPCSolver = ISMPC_Solver(dt, Controller_Config.delta, Controller_Config.Tp, Controller_Config.Tc);
  FootStpGen = FootStepGen();

  Configure(Controller_Config);
  Configure(config);

  auto rConfig = config("controller")("robot")(robot().name());
  rConfig("rightFootLink", RightFootLinkName_);
  rConfig("leftFootLink", LeftFootLinkName_);

  // auto rm0 = mc_rbdyn::RobotLoader::get_robot_module("HRP4");
  // for(const auto & j : rm0->mb.joints())
  // {
  //   if(j.dof() == 1 && !j.isMimic())
  //   {
  //     std::cout << "- " << j.name() << "\n";
  //   }
  // }

  mc_rtc::log::info(robots().envIndex());
  controller_timestep = dt;
  config_.load(config);
  static auto constraint = mc_solver::ConstraintSetLoader::load(solver(), config("collisions")[0]);

  datastore().make_call("KinematicAnchorFrame::" + robot().name(), [this](const mc_rbdyn::Robot & robot) {
    return sva::interpolate(robot.surfacePose("LeftFoot"), robot.surfacePose("RightFoot"), LeftFootRatio);
  });
  PrevLeftFootRatio = 0.5;

  solver().addConstraintSet(*constraint);
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);
  solver().addConstraintSet(dynamicsConstraint);

  solver().setContacts({{robots(), 0, 1, "LeftFoot", "AllGround"}, {robots(), 0, 1, "RightFoot", "AllGround"}});

  predictedZMPWorld.clear();
  predictedCoMWorld.clear();

  rightShoulderIndex = robot().jointIndexByName("R_SHOULDER_P");
  rightLegIndex = robot().jointIndexByName("R_HIP_P");

  mc_rtc::log::info("AnkleIndex : {} ", robot().jointIndexByName("L_ANKLE_P"));

  leftShoulderIndex = robot().jointIndexByName("L_SHOULDER_P");
  leftLegIndex = robot().jointIndexByName("L_HIP_P");

  leftSwingFootTask =
      std::make_shared<mc_tasks::SurfaceTransformTask>("LeftFoot", robots(), robots().robotIndex(), 10.0, 10.);

  rightSwingFootTask =
      std::make_shared<mc_tasks::SurfaceTransformTask>("RightFoot", robots(), robots().robotIndex(), 10.0, 10.);

  SwingFootContact = mc_tasks::lipm_stabilizer::ContactState::Left;
  SupportFootContact = mc_tasks::lipm_stabilizer::ContactState::Right;

  Tail = "Anticipative";

  for(int k = 0; k < Controller_Config.Stab_config.comActiveJoints.size(); k++)
  {
    mc_rtc::log::info(Controller_Config.Stab_config.comActiveJoints[k]);
  }

  mc_rtc::log::info("CoM Weight : {} ", Controller_Config.Stab_config.comWeight);
  mc_rtc::log::info("CoM Stiff : {}", Controller_Config.Stab_config.comStiffness.y());
  mc_rtc::log::info("DCM P : {}", Controller_Config.Stab_config.dcmPropGain);
  mc_rtc::log::info("DCM I : {}", Controller_Config.Stab_config.dcmIntegralGain);
  mc_rtc::log::info("Contact Weight : {}", Controller_Config.Stab_config.contactWeight);
  mc_rtc::log::info("Contact Stiffness : {}", Controller_Config.Stab_config.contactStiffness);

  // Stab_config.comStiffness = Eigen::Vector3d{1000,1000,1000};
  // Stab_config.comWeight = 200;
  // Stab_config.torsoStiffness = 5;
  // Stab_config.torsoWeight = 200;
  // Stab_config.pelvisStiffness = 5;
  // Stab_config.pelvisWeight = 200;
  // postureTask->stiffness(1);
  // postureTask->weight(5);
  // SwingFootWeight = 500;
  // SwingFootStiffness = 300;
  // SupportFootWeight = 20000;
  // SupportFootStiffness = 10000;

  StabTask = std::make_shared<mc_tasks::lipm_stabilizer::StabilizerTask>(solver().robots(), solver().realRobots(),
                                                                         solver().robots().robotIndex(), solver().dt());

  StabTask->configure(Controller_Config.Stab_config);

  mc_rtc::log::info("CoM Height : {} ", robot().com().z());

  StabTask->reset();

  Controller_Config.Stab_config.copMaxVel.angular() = Eigen::Vector3d{1.5, 1.5, 0.3};
  Controller_Config.Stab_config.copMaxVel.linear() = Eigen::Vector3d{.5, .5, 0.3};

  postureTask->stiffness(1);
  postureTask->weight(5);

  armTask = std::make_shared<mc_tasks::PostureTask>(solver(), robots().robotIndex(), 100, 10);
  std::vector<std::string> armTask_Joints({"R_SHOULDER_P", "L_SHOULDER_P", "R_SHOULDER_R", "R_ELBOW_P", "R_WRIST_Y",
                                           "R_WRIST_P", "R_WRIST_R", "L_SHOULDER_R", "R_SHOULDER_Y", "L_SHOULDER_Y",
                                           "L_ELBOW_P", "L_WRIST_Y", "L_WRIST_P", "L_WRIST_R"});
  armTask->selectActiveJoints(solver(), armTask_Joints);
  armTask->name("ArmPosture");

  supportFootName = "RightFoot";
  swingFootName = "LeftFoot";

  SupportFootPose = robot().surfacePose(supportFootName).translation();
  SupportFootPose.z() = 0;

  Pck = robot().com();
  Pck.z() = Controller_Config.CoMz0;
  Pzk = robot().surfacePose(swingFootName).translation();
  Pzk.z() = 0;
  Puk = Pck;
  Vck = robot().comVelocity();

  SwingFootInitialPose = robot().surfacePose(swingFootName).translation();
  X_0_SwingFootInitial = SwingFootInitialPose;

  SwingFootAcc.setZero();
  SwingFootVel.setZero();

  if(!joystick.isFound())
  {
    joystickConnected = false;
    mc_rtc::log::warning("WARNING: NO JOYPAD DETECTED");
  }

  create_datastore();
  getTransformations();

  bool start_now = config("controller")("start_at_launch");
  if(start_now)
  {
    Stop = false;
    N_Steps_Desired = config("controller")("steps");
    T_Steps = config("controller")("t_steps");
    Eigen::Vector3d v = config("controller")("speed");
    Vx_i = v.x();
    Vy_i = v.y();
    Omega_i = v.z();
  }
  V.clear();
  T.clear();
  Pf.clear();
  GenReferenceVelocity(Vx_i, Vy_i, Omega_i);
  ComputeTrajectoryOnce = true;
  WalkingTrajectoryThread = std::thread(&Walking_controller::WalkingTrajectoryLoop, this);
  // WalkingTrajectoryThread.detach();
  // WalkingTrajectoryLoop();
  mc_rtc::log::info("waiting for first computation");
  std::chrono::high_resolution_clock::time_point t_clock = std::chrono::high_resolution_clock::now();
  std::condition_variable cv;
  while(WalkingTrajectory_Computing)
  {
    sleep(1);
    std::chrono::duration<double, std::milli> time_span = std::chrono::high_resolution_clock::now() - t_clock;
    if (time_span.count() > 5e3)
    {
      mc_rtc::log::error("Exiting waiting loop");
      WalkingTrajectory_Computing = false;
    }
  }

  solver().addTask(StabTask);

  solver().addTask(leftSwingFootTask);
  solver().addTask(rightSwingFootTask);

  solver().addTask(postureTask);
  solver().addTask(armTask);
  updateTasks();
  addToGUI();

  mc_rtc::log::success("Controller init done ");
}

void Walking_controller::getTransformations()
{
  // X_world_floatingbase = robot().surfacePose("RightFoot");
  // X_world_floatingbase = robot().mbc().bodyPosW[robot().bodyIndexByName("base_link")];

  // floatingbaseWorldPos = X_world_floatingbase.translation();
  // floatingbaseWorldOri = X_world_floatingbase.rotation().transpose();
  // if(t == 0)
  // {
  //   R_body_world_Step = floatingbaseWorldOri;
  // }
  // floatingbaseWorldRPY << mc_rbdyn::rpyFromMat(floatingbaseWorldOri.inverse());

  X_0_leftFoot = robot().surfacePose("LeftFoot");
  X_0_rightFoot = robot().surfacePose("RightFoot");

  R_0_leftFoot = X_0_leftFoot.rotation();
  R_leftFoot_0 = X_0_leftFoot.rotation().transpose();
  T_leftFoot_0 = X_0_leftFoot.translation();

  R_0_rightFoot = X_0_rightFoot.rotation();
  R_rightFoot_0 = X_0_rightFoot.rotation().transpose();
  T_rightFoot_0 = X_0_rightFoot.translation();

  if(supportFootName == "LeftFoot")
  {
    X_0_support = X_0_leftFoot;
    X_0_swing = X_0_rightFoot;
    X_support_0 = X_0_leftFoot.inv();
    R_0_support = X_0_leftFoot.rotation();
    T_0_support = X_support_0.translation();
    R_support_0 = R_0_support.inverse();
    T_support_0 = X_0_support.translation();
    R_swing_0 = R_rightFoot_0;
    T_swing_0 = T_rightFoot_0;
  }
  else
  {
    X_0_support = X_0_rightFoot;
    X_0_swing = X_0_leftFoot;
    X_support_0 = X_0_rightFoot.inv();
    R_0_support = X_0_rightFoot.rotation();
    T_0_support = X_support_0.translation();
    R_support_0 = R_0_support.inverse();
    T_support_0 = X_0_support.translation();
    R_swing_0 = R_leftFoot_0;
    T_swing_0 = T_leftFoot_0;
  }
  R_0_swing = R_swing_0.inverse();
  X_0_support_real = realRobot().surfacePose(supportFootName);
  X_support_0_real = X_0_support_real.inv();
  R_support_0_real = X_0_support_real.rotation().inverse();
  T_support_0_real = X_0_support_real.translation();

  R_0_support_real = X_0_support_real.rotation();
  T_0_support_real = X_support_0_real.translation();

  // Compute support foot with flat floor assumption
  const auto & real_r = realRobot();
  const auto & X_world_real_support = real_r.surface(supportFootName).X_0_s(real_r);
  const Eigen::Vector3d & realRPY = mc_rbdyn::rpyFromMat(X_world_real_support.rotation().inverse());
  const Eigen::Matrix3d & orientationAroundZ = mc_rbdyn::rpyToMat(0, 0, realRPY(2));
  Eigen::Vector3d positionXY;
  positionXY << X_world_real_support.translation().x(), X_world_real_support.translation().y(), 0.;
  X_0_support_flat = sva::PTransformd(orientationAroundZ.inverse(), positionXY);

}

Eigen::Vector3d Walking_controller::computeInSupportFootFlat(const Eigen::Vector3d & t_world)
{
  // const auto & com = t_world.eval();
  // sva::PTransformd X_0_com(com);
  // const auto X_s_com = X_0_com * X_0_support_flat.inv();
  // return X_s_com.translation();

  return (t_world - T_support_0_real) - T_0_support;
}

Eigen::Vector3d Walking_controller::computeVelocityInSupportFoot(const Eigen::Vector3d & v_world)
{
  const auto & r = realRobot();
  const Eigen::Vector3d comV = v_world;
  const Eigen::Matrix3d R_0_support = r.surface(supportFootName).X_0_s(r).rotation();
  const Eigen::Vector3d comVInSupportFrame = R_0_support * comV;
  return comVInSupportFrame;
}

sva::ForceVecd Walking_controller::measuredContactWrench()
{
  sva::ForceVecd netWrench{Eigen::Vector6d::Zero()};
  for(std::string sensorName : {"LeftFootForceSensor", "RightFootForceSensor"})
  {
    const auto & sensor = robot().forceSensor(sensorName);
    if(sensor.force()[2] > 1.)
    {
      // XXX double check this
      netWrench += sensor.worldWrench(realRobot());
    }
  }
  return netWrench;
}

Eigen::Vector3d Walking_controller::computeZMP()
{
  // XXX double check which robot to use as reference
  const auto & measuredWrench = measuredContactWrench();
  const Eigen::Vector3d & force = measuredWrench.force();
  // We're walking, there should always be at least the weight of the robot
  if(force.z() < 100)
  {
    // mc_rtc::log::error_and_throw<std::runtime_error>("ZMP cannot be computed, force too small {}",
    // force.transpose());
    // mc_rtc::log::error("ZMP cannot be computed, force too small {}", force.transpose());
  }
  const Eigen::Vector3d & moment_0 = measuredWrench.couple();

  // Arbitrary frame on the floor
  const sva::PTransformd & floor = sva::PTransformd::Identity();
  const Eigen::Vector3d & floor_p = floor.translation();
  // normal vector to ZMP to ZMP plane
  const Eigen::Vector3d & floor_n = floor.rotation().row(2);

  Eigen::Vector3d moment_p = moment_0 - floor_p.cross(force);
  Eigen::Vector3d zmp = floor_p + floor_n.cross(moment_p) / floor_n.dot(force);
  return zmp;
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

  MPCSolver.AutoFootstepPlacement = AutoFootstepPlacement;
  WalkingTrajectory_Computing = true;
  if(ComputeTrajectoryOnce)
  {
    std::chrono::high_resolution_clock::time_point t_clock = std::chrono::high_resolution_clock::now();
    WalkingTrajectory_Computing = true;
    // if (V.size() != 0)
    // {
    //   if(Vx[0] != Vx_i || Vy[0] != Vy_i || Omega[0] != Omega_i)
    //   {
    //     GenReferenceVelocity(Vx_i, Vy_i, Omega_i);
    //   }
    // }
    // else
    // {
    //   GenReferenceVelocity(Vx_i, Vy_i, Omega_i);
    // }


    // T.clear();
    // T.push_back(T_Steps);
    // T.push_back(T[0] + T_Steps);
    T.clear();
    // V.clear();
    // double angle = z*M_PI/180.;
    // Eigen::Vector3d Step_Target = Eigen::Vector3d{x,y,angle};
    Pf.clear();
    // Pf.push_back(Step_Target);

    SupportFootPose = robot().surfacePose(supportFootName).translation();
    SupportFootPose.z() = mc_rbdyn::rpyFromMat(robot().surfacePose(supportFootName).rotation()).z();

    Eigen::Vector3d Pf_m1(SwingFootInitialPose);
    Pf_m1.z() = SupportFootPose.z();

    FootStpGen.Init(supportFootName, SupportFootPose, V, T, Pf);

    FootStpGen.GetFoosteps();

    if(Pf.size() != 0)
    {
      N_Steps = 0;
      N_Steps_Desired = FootStpGen.Get_Nsteps();
    }

    mpc_thread_state.P_traj = FootStpGen.Ref_Traj();
    mpc_thread_state.Xf = FootStpGen.Xf();
    mpc_thread_state.Yf = FootStpGen.Yf();
    mpc_thread_state.Thetaf = FootStpGen.Theta_f();

    mpc_thread_state.Xf_Corr = mpc_thread_state.Xf;
    mpc_thread_state.Yf_Corr = mpc_thread_state.Yf;
    if(mpc_thread_state.TimeStamps.size() != 0)
    {

      if(mpc_thread_state.TimeStamps[0] != FootStpGen.StepsTiming()[0])
      {
        t = FootStpGen.StepsTiming()[0] * t / mpc_thread_state.TimeStamps[0];
        countStart = count - std::round(t / controller_timestep);
      }
    }

    mpc_thread_state.TimeStamps = FootStpGen.StepsTiming();
    mpc_thread_state.TimeStampsIndex = FootStpGen.TimesIndex();
    if(Swing_Foot_Contact)
    {
      Tds = Controller_Config.Double_Step_Ratio * mpc_thread_state.TimeStamps[0];
    };

    MPCSolver.InitStepGen(mpc_thread_state.Xf, mpc_thread_state.Yf, mpc_thread_state.Thetaf);

    int Steps = N_Steps;
    int Steps_Desired = N_Steps_Desired;
    if(Stop && !Swing_Foot_Contact)
    {
      Steps_Desired = Steps + 1;
    }

    MPCSolver.SetWalkingParameters(Pck, Vck, Pzk, Pf_m1, FootStpGen.StepsTiming(), FootStpGen.TimesIndex(), Tail,
                                   N_Steps_Desired, N_Steps);

    MPCSolver.GetWalkingParameters(PrevStepTiming, t_k, Tds);

    if(MPCSolver.QPsucceeded())
    {
      mpc_thread_state.X_MPC = MPCSolver.X_MPC();
      mpc_thread_state.Y_MPC = MPCSolver.Y_MPC();
      mpc_thread_state.Xf_Corr = MPCSolver.Xf_Corr();
      mpc_thread_state.Yf_Corr = MPCSolver.Yf_Corr();
      Index = 0;
      mpc_thread_state.SupPolygon = MPCSolver.get_polynome_support();
      mpc_thread_state.Traj_ant = MPCSolver.GetAfterTc_ZMP_trajectory();
    
      std::lock_guard<std::mutex> lk(mutex_mpc_);
      mpc_state_ = mpc_thread_state;
      
    }

    ComputeTrajectoryOnce = false;

    std::chrono::duration<double, std::milli> time_span = std::chrono::high_resolution_clock::now() - t_clock;
    ProcessTime = time_span.count();
  }
  WalkingTrajectory_Computing = false;
  
}

bool Walking_controller::run()
{

  std::chrono::duration<double, std::milli> time_span = std::chrono::high_resolution_clock::now() - t_clock;
  ControllerLoopTime = time_span.count();
  t_clock = std::chrono::high_resolution_clock::now();

  if(emergencyFlag) return false;

  t = (count - countStart) * controller_timestep;

  // Update state variable
  UpdateInitialVectors();

  joypadLoop();
  getTransformations();
  // WalkingTrajectoryLoop();
  GenReferenceVelocity(Vx_i, Vy_i, Omega_i);
  StabTask->configure(Controller_Config.Stab_config);
  MPCSolver.configure(Controller_Config);

  // mc_rtc::log::info("// timing : {} ",t-t_k);

  if(!(Stop && Swing_Foot_Contact))
  {
    // StabTask->dcmGains(Controller_Config.Stab_P_gain,Stab_I_gain,0);
    MoveCoM(t);
    MoveFeet(t);
    if(t - t_k + controller_timestep > Controller_Config.delta)
    {
      ComputeTrajectoryOnce = true;
      t_k = t + controller_timestep;
    }
    if(mpc_state_.TimeStamps.size() != 0)
    {
      if(t > Tds && Swing_Foot_Contact)
      {
        N_Steps += 1;
        if(N_Steps == N_Steps_Desired)
        {
          Stop = true;
        }
        reset_MPC_states();
      }
    }

    Robot_Walking = true;
  }
  else
  {

    updateTasks();

    T_conv = 0.4;

    StabTask->copAdmittance(Eigen::Vector2d{0.01, 0.01});

    Eigen::Vector3d StaticPose(
        (robot().surfacePose("LeftFoot").translation() + robot().surfacePose("RightFoot").translation()) / 2);
    StaticPose.z() = Controller_Config.CoMz0;
    StabTask->staticTarget(StaticPose);
    StabTask->comStiffness(Eigen::Vector3d::Ones() * 10);
    t_k = 0;
    kfoot = 0;
    N_Steps = 0;
    countStart = count;

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

  predictedZMPWorld.clear();
  predictedCoMWorld.clear();
  for(int k = 0; k < mpc_state_.X_MPC.size(); k++)
  {
    predictedZMPWorld.push_back(mpc_state_.Get_ZMP_planarTarget(Index) );
    predictedCoMWorld.push_back(mpc_state_.Get_CoM_planarTarget(Index) );
  }

  count += 1;

  // StabTask->Set_Tconv(T_conv);

  gui()->removeCategory({"Walking", "Visualization", "FootStep"});
  add_FootSteps_GUI();
  bool ret = mc_control::MCController::run();

  UpdateFootRatio();
  update_datastore();
  return ret;
}

void Walking_controller::MoveCoM(double t)
{

  // mc_rtc::log::info("//Index : " + std::to_string(Index));

  if(Index + 1 >= mpc_state_.X_MPC.size())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Control Horizon reached");
  }

  
  Eigen::Vector3d Pcom(mpc_state_.Get_CoM_planarTarget(Index)); Pcom.z() = Controller_Config.CoMz0;
  zmpTarget = mpc_state_.Get_ZMP_planarTarget(Index);
  Eigen::Vector3d zmpdTarget;
  // zmpdTarget.setZero();
  // zmpdTarget.x() = MPCSolver.ZMP_vel()[Index % ((int)(Controller_Config.delta / controller_timestep))];
  // zmpdTarget.y() = MPCSolver.ZMP_vel()[(Index + ((int)MPCSolver.ZMP_vel().size() / 2))
  //                                     % ((int)(Controller_Config.delta / controller_timestep))];
  Eigen::Vector3d Vc(mpc_state_.Get_CoMVel_planarTarget(Index));
  
  Eigen::Vector3d robot_vc = robot().comVelocity();

  double diffV_in = (Vc - robot_vc).norm();

  if(diffV_in > 0.12)
  {
    // Vc = robot_vc - Eigen::Vector3d{0,0,robot_vc.z()};
    // mc_rtc::log::info("diff V input {}", diffV_in);
    StabTask->comStiffness(30 * Eigen::VectorXd::Ones(3));
  }
  else
  {
    StabTask->comStiffness(Controller_Config.CoM_Stiff * Eigen::VectorXd::Ones(3));
  }

  Eigen::Vector3d Ac = std::pow(eta(), 2) * (Pcom - zmpTarget);
  Ac.z() = 0;

  dcmTarget = Pcom + Vc / eta();

  Eigen::Vector3d dcm = Pck + Vck / eta();

  StabTask->target(Pcom, Vc, Ac, zmpTarget);

  Index += 1;
}

bool Walking_controller::MoveFeet(double t)
{

  if(mpc_state_.TimeStamps.size() == 0)
  {
    return 0;
  }

  PrevStepTiming = 0;
  double NextTimeStep(0);

  NextTimeStep = mpc_state_.get_Ts(kfoot);
  if(kfoot != 0)
  {
    PrevStepTiming = mpc_state_.get_Ts(kfoot-1);
  }

  X_0_SwingFootInitial.translation() = SwingFootInitialPose;
  double PrevSwingFootAngle = SwingFootInitialAngle;
  if(kfoot != 0)
  {
    X_0_SwingFootInitial.translation() = mpc_state_.Get_CorrectedFootstep(kfoot-1) ;
    PrevSwingFootAngle = mpc_state_.Thetaf(kfoot - 1);
  }
  X_0_SwingFootInitial.rotation() = sva::RotZ(PrevSwingFootAngle);

  sva::PTransformd X_0_SwingFootTarget;
  if(kfoot + 1 < mpc_state_.Xf_Corr.size())
  {
    X_0_SwingFootTarget.translation() = mpc_state_.Get_CorrectedFootstep(kfoot+1);
  }
  else
  {
    X_0_SwingFootTarget.translation() = mpc_state_.Get_PlannedFootstep(kfoot+1);
  }

  X_0_SwingFootTarget.rotation() = sva::RotZ(mpc_state_.Thetaf(kfoot + 1));

  if(Swing_Foot_Contact)
  {
    t_lift = PrevStepTiming + Tds;
  }

  Eigen::Vector3d SupportFootPose = robot().surfacePose(supportFootName).translation();
  SupportFootPose.z() = mc_rbdyn::rpyFromMat(robot().surfacePose(supportFootName).rotation()).z();

  std::pair<mc_tasks::lipm_stabilizer::ContactState, sva::PTransformd> SupportState(
      SupportFootContact, robot().surfacePose(supportFootName));

  std::string sensorName = swingFootName + "ForceSensor";
  const auto & sensor = robot().forceSensor(sensorName);

  if(Swing_Foot_Contact)
  {

    if(t > 0.1)
    {
      T_conv = 0.2;
    }

    if(t > (PrevStepTiming + Tds) - 0.15 && DoubleSupport_state) // t > (PrevStepTiming + Tds)  - 0.1 &&
    {

      // StabTask->setContacts({SupportState});

      // StabTask->copAdmittance(Controller_Config.Std_Admittance);
    }

    if(t > PrevStepTiming + Tds)
    {

      mc_rtc::log::success("lifting " + swingFootName);

      StabTask->setContacts({SupportState});
      DoubleSupport_state = false;

      solver().setContacts({{robots(), 0, 1, supportFootName, "AllGround"}});

      Swing_Foot_Contact = false;

      t_lift = t;

      std::string swingFootLinkName = LeftFootLinkName_;
      if(swingFootName == "RightFoot")
      {
        swingFootLinkName = RightFootLinkName_;
      }

      // t_lift += offset;

      T_conv = 0.2;
    }
  }

  double offset = 0;
  double SingleSupportDuration = (NextTimeStep - t_lift) + offset;
  double height_off = 0;
  SwingFootTrajectory.set_Z_ContactOffset(height_off);

  SwingFootTrajectory.getSwingFootTrajectory(X_0_SwingFootTarget, X_0_SwingFootInitial, t,
                                             Controller_Config.FootStepHeight, SingleSupportDuration, t_lift,
                                             controller_timestep);

  SwingFootVel = SwingFootTrajectory.GetVelocity().linear();
  SwingFootAcc = SwingFootTrajectory.GetAccel().linear();
  sva::PTransformd X_0_FootTask_Target = SwingFootTrajectory.GetTrajectory();

  Eigen::Matrix3d R_0_swing = X_0_FootTask_Target.rotation();
  sva::MotionVecd V_0_FootTask_Target = SwingFootTrajectory.GetVelocity();

  SwingFootTask->target(X_0_FootTask_Target);
  SwingFootTask->refVelB(sva::PTransformd(X_0_swing.rotation()) * V_0_FootTask_Target);

  if(!Swing_Foot_Contact)
  {

    double Step_Time = t - t_lift;
    double swing_foot_height = robot().surfacePose(swingFootName).translation().z();
    double support_foot_height = robot().surfacePose(supportFootName).translation().z();
    double foot_diff_height =
        (realRobot().surfacePose(swingFootName).translation() - realRobot().surfacePose(supportFootName).translation())
            .z();

    bool TouchDown = (sensor.force().norm() > 42 || swing_foot_height < 1e-3);

    if(((Step_Time > 0.25 && TouchDown) || Step_Time >= SingleSupportDuration) && !DoubleSupport_state)

    {

      T_conv = 0.05;
      StabTask->copAdmittance(Controller_Config.Impact_Admittance);

      t_contact = t;

      StabTask->setContacts(
          {mc_tasks::lipm_stabilizer::ContactState::Left, mc_tasks::lipm_stabilizer::ContactState::Right});
      DoubleSupport_state = true;

      mc_rtc::log::info("height : {} ", swing_foot_height);
      mc_rtc::log::info("touchdown : {} ", TouchDown);
      // mc_rtc::log::info("Locking " + swingFootName + "at t : " + std::to_string(t));
      mc_rtc::log::info("T_contact - T_steps : {}", t - NextTimeStep);

      // }
      // if( (Step_Time  >= SingleSupportDuration) &&
      //                                     !Swing_Foot_Contact &&
      //                                     DoubleSupport_state )
      // {

      solver().setContacts({{robots(), 0, 1, "LeftFoot", "AllGround"}, {robots(), 0, 1, "RightFoot", "AllGround"}});
      // StabTask->setContacts({mc_tasks::lipm_stabilizer::ContactState::Left,mc_tasks::lipm_stabilizer::ContactState::Right});
      StabTask->copAdmittance(Controller_Config.Std_Admittance);
      StabTask->contactStiffness(Controller_Config.Stab_config.contactStiffness);


      mc_rtc::log::success("Locked " + swingFootName);

      Swing_Foot_Contact = true;
      switchFootSupport();
      updateTasks();

      // PrevStepTiming = NextTimeStep;
      if(kfoot + 1 < mpc_state_.Xf_Corr.size())
      {
        kfoot += 1;
      }

      SwingFootInitialPose = robot().surfacePose(swingFootName).translation();
      SwingFootInitialAngle = mc_rbdyn::rpyFromMat(robot().surfacePose(swingFootName).rotation()).z();

      reset_MPC_states();
    }
  }

  if(DoubleSupport_state)
  {

    Eigen::Vector3d pos;
    double yaw;

    pos = robot().surfacePose(supportFootName).translation();
    pos.z() = 0.0;
    yaw = mc_rbdyn::rpyFromMat(robot().surfacePose(supportFootName).rotation()).z();
    SupportFootTask->target(sva::PTransformd(sva::RotZ(yaw), pos));
    SupportFootTask->refVelB(sva::MotionVecd::Zero());
    SupportFootTask->refAccel(sva::MotionVecd::Zero());

    pos = robot().surfacePose(swingFootName).translation();
    pos.z() = 0.0;
    yaw = mc_rbdyn::rpyFromMat(robot().surfacePose(swingFootName).rotation()).z();
    SwingFootTask->target(sva::PTransformd(sva::RotZ(yaw), pos));
    SwingFootTask->refVelB(sva::MotionVecd::Zero());
    SwingFootTask->refAccel(sva::MotionVecd::Zero());
  }

  return 0;
}

void Walking_controller::UpdateFootRatio()
{

  double ratio_obj = 0;
  if(supportFootName == "RightFoot")
  {
    ratio_obj = 1;
  }

  if(t > Tds || Stop)
  {

    double ratio = StabTask->leftFootRatio();

    // mc_rtc::log::info(ratio);
    sva::PTransformd Prev_Anchor =
        sva::interpolate(robot().surfacePose("LeftFoot"), robot().surfacePose("RightFoot"), LeftFootRatio);
    sva::PTransformd Anchor =
        sva::interpolate(robot().surfacePose("LeftFoot"), robot().surfacePose("RightFoot"), ratio);
    if((Prev_Anchor.translation() - Anchor.translation()).norm() > 0.01)
    {
      LeftFootRatio += 0.01 * (ratio - LeftFootRatio) / std::abs(ratio - LeftFootRatio);
    }
    else
    {
      LeftFootRatio = ratio;
    }
    PrevLeftFootRatio = LeftFootRatio;
  }
  else
  {
    LeftFootRatio = PrevLeftFootRatio + t * (ratio_obj - PrevLeftFootRatio) / (Tds);
  }
  // mc_rtc::log::info(LeftFootRatio);
  // mc_rtc::log::info("//");

  LeftFootRatio = std::min(1.0, std::max(0.0, LeftFootRatio));

  // LeftFootRatio = StabTask->leftFootRatio();
}

void Walking_controller::UpdateInitialVectors()
{

  Pck = robot().com();
  Vck = robot().comVelocity();

  // Pzk = Eigen::Vector3d{0,0,1}.cross( robot().com().cross(robot().mass()*mc_rtc::constants::gravity) ) /
  //                       ( (robot().mass()*(mc_rtc::constants::gravity - robot().comAcceleration())).transpose() *
  //                       Eigen::Vector3d{0,0,1} );

  // Pzk = Pck;

  dcmMeasured = realRobot().com() + realRobot().comVelocity() / eta();
  zmpMeasured = computeZMP();

  if(mpc_state_.X_MPC.size() != 0 && !(Stop && DoubleSupport_state))
  {
    Pzk = mpc_state_.Get_ZMP_planarTarget(Index);

  }
  if(UseRealRobot)
  {
    double K = 1;

    // Pck = Pck * (1 - K) + K * computeInSupportFootFlat(realRobot().com());
    // Vck = Vck * (1 - K) + K * computeVelocityInSupportFoot(realRobot().comVelocity());
    Pck = Pck * (1 - K) + K * realRobot().com();
    Vck = Vck * (1 - K) + K * realRobot().comVelocity();
    // Pzk = Pzk*(1-K) + K*computeInSupportFootFlat(computeZMP());
  }

  Pck.z() = Controller_Config.CoMz0;
  Vck.z() = 0;
  Pzk.z() = 0;
}

void Walking_controller::GenReferenceVelocity(double vx, double vy, double omega)
{
  Vx.clear();
  Vy.clear();
  Omega.clear();
  for(int k = 0; k < (int)2 * std::round(Controller_Config.Tp / Controller_Config.delta); k++)
  {
    Vx.push_back(vx);
    Vy.push_back(vy);
    Omega.push_back(omega);
  }
  V.clear();
  V.push_back(Vx);
  V.push_back(Vy);
  V.push_back(Omega);
}

void Walking_controller::updateTasks()
{
  std::vector<std::string> SupportFootJoints;
  if(supportFootName == "LeftFoot")
  {

    SupportFootTask = leftSwingFootTask;
    SwingFootTask = rightSwingFootTask;
  }
  else
  {

    SupportFootTask = rightSwingFootTask;
    SwingFootTask = leftSwingFootTask;
  }
  Eigen::MatrixXd dimW(Eigen::VectorXd::Zero(6));
  dimW(5) = 1;
  SwingFootTask->weight(Controller_Config.SwingFootWeight);
  SwingFootTask->dimWeight(Eigen::VectorXd::Ones(6));
  SwingFootTask->stiffness(Controller_Config.SwingFootStiffness);
  SupportFootTask->weight(Controller_Config.SupportFootWeight);
  SupportFootTask->stiffness(Controller_Config.SupportFootStiffness * dimW);

  // Eigen::VectorXd dimW_com(Eigen::VectorXd::Ones(3)); dimW_com(2) = 0.1;
  // CoMTask->dimWeight(dimW_com);
}

void Walking_controller::switchFootSupport()
{
  if(supportFootName == "LeftFoot")
  {
    supportFootName = "RightFoot";
    swingFootName = "LeftFoot";

    SwingFootContact = mc_tasks::lipm_stabilizer::ContactState::Left;
    SupportFootContact = mc_tasks::lipm_stabilizer::ContactState::Right;
  }

  else
  {

    supportFootName = "LeftFoot";
    swingFootName = "RightFoot";

    SwingFootContact = mc_tasks::lipm_stabilizer::ContactState::Right;
    SupportFootContact = mc_tasks::lipm_stabilizer::ContactState::Left;
  }
}

void Walking_controller::joypadLoop()
{
  if(joystickConnected)
  {
    // Stop = (vRefX==0 && vRefY == 0);

    Vx_i = vRefX;
    Vy_i = vRefY;
    Omega_i = omegaRef;

    if(joystick.sample(&event) || vRefX != 0 || vRefY != 0)
    {
      if(event.isButton())
      {
        if(event.number == 2 && event.value == 1)
        {
          std::cout << "UNUSED" << std::endl;
        }
        if(event.number == 1 && event.value == 1)
        {
          std::cout << "STOP" << std::endl;
          Stop = true;
        }
        if(event.number == 0 && event.value == 1)
        {
          std::cout << "START" << std::endl;
          if(Stop && Swing_Foot_Contact)
          {
            Stop = false;
            t_k = 0;
            ComputeTrajectoryOnce = true;
          }
        }

        if(event.number == 3 && event.value == 1)
        {
          std::cout << "EMERGENCY" << std::endl;
          emergencyFlag = true;
        }

        if(event.number == 5 && event.value == 1)
        {
          if(maxVelX <= 0.25)
          {
            maxVelX += 0.05;
            minVelX = -maxVelX;
          }
          // std::cout << "maxVelX = " << maxVelX << std::endl;
        }
        if(event.number == 4 && event.value == 1)
        {
          if(maxVelX >= 0.15)
          {
            maxVelX -= 0.05;
            minVelX = -maxVelX;
          }
          // std::cout << "maxVelX = " << maxVelX << std::endl;
        }
      }

      if(event.isAxis() || vRefX != 0 || vRefY != 0)
      {
        if(event.number == 1)
        {
          double value = -event.value;
          vRefY = (maxVelX - minVelX) * (value + 32767) / (32767 * 2) + minVelX;
          if(abs(vRefY) < 0.02)
          {
            vRefY = 0;
          }
          // std::cout << "vRefX = " << vRefX << std::endl;
          // if(vRefX<-0.2)
          //   vRefX=-0.2;
        }
        if(event.number == 3)
        {
          double value = -event.value;
          double maxVel = 0.1;
          double minVel = -0.1;
          omegaRef = (maxVel - minVel) * (value + 32767) / (32767 * 2) + minVel;
          if(abs(omegaRef) < 0.03)
          {
            omegaRef = 0;
          }
          // std::cout << "omegaRef = " << omegaRef << std::endl;
        }

        if(event.number == 2)
        {
          double value = -event.value;
          double maxVel = 0;
          double minVel = -0.15;
          vRefX = (maxVel - minVel) * (value + 32767) / (32767 * 2) + minVel;
          if(abs(vRefX) < 0.02)
          {
            vRefX = 0;
          }

          // std::cout << "vRefX = " << vRefY << std::endl;
        }

        if(event.number == 5)
        {
          double value = event.value;
          double maxVel = 0.15;
          double minVel = 0;
          vRefX = (maxVel - minVel) * (value + 32767) / (32767 * 2) + minVel;

          if(abs(vRefX) < 0.02)
          {
            vRefX = 0;
          }

          // std::cout << "vRefX = " << vRefY << std::endl;
        }
      }
    }
    PrevVrefX = vRefX;
  }
}

void Walking_controller::reset(const mc_control::ControllerResetData & reset_data)
{
  // Vx_i = 0. ; Vy_i = 0. ; Omega_i = 0.;
  SwingFootTask.reset();
  SupportFootTask.reset();
  supportFootName = "RightFoot";
  swingFootName = "LeftFoot";

  SupportFootPose = robot().surfacePose(supportFootName).translation();
  SupportFootPose.z() = 0;

  Pck = robot().com();
  Pck.z() = Controller_Config.CoMz0;
  Pzk = robot().surfacePose(swingFootName).translation();
  Pzk.z() = 0;
  Puk = Pck;
  Vck = robot().comVelocity();

  SwingFootInitialPose = robot().surfacePose(swingFootName).translation();
  X_0_SwingFootInitial = SwingFootInitialPose;
  updateTasks();

  Eigen::Matrix6d dof = Eigen::Matrix6d::Identity(6, 6);
  dof(0, 0) = 0;
  dof(1, 1) = 0;
  dof(5, 5) = 0;
  // dof = Eigen::Matrix6d::Zero(6,6);
  tasks::qp::ContactId ContactId_R;
  tasks::qp::ContactId ContactId_L;
  ContactId_R = mc_rbdyn::Contact(robots(), 0, 1, "RightFoot", "AllGround").contactId(robots());
  ContactId_L = mc_rbdyn::Contact(robots(), 0, 1, "LeftFoot", "AllGround").contactId(robots());
  contactConstraint.contactConstr->addDofContact(ContactId_L, dof);
  contactConstraint.contactConstr->addDofContact(ContactId_R, dof);
  contactConstraint.contactConstr->updateDofContacts();

  MPC_thread_on = false;
  WalkingTrajectoryThread.join();
  MPC_thread_on = true;

  V.clear();
  T.clear();
  Pf.clear();
  GenReferenceVelocity(Vx_i, Vy_i, Omega_i);
  ComputeTrajectoryOnce = true;
  
  WalkingTrajectoryThread = std::thread(&Walking_controller::WalkingTrajectoryLoop, this);

  mc_rtc::log::info("waiting for first computation after reset");
  std::chrono::high_resolution_clock::time_point t_clock = std::chrono::high_resolution_clock::now();
  std::condition_variable cv;
  while(WalkingTrajectory_Computing)
  {
    sleep(1);
    std::chrono::duration<double, std::milli> time_span = std::chrono::high_resolution_clock::now() - t_clock;
    if (time_span.count() > 5e3)
    {
      mc_rtc::log::error("Exiting waiting loop");
      WalkingTrajectory_Computing = false;
    }
  }

  mc_control::MCController::reset(reset_data);
};

