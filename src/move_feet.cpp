#include "Walking_controller.h"

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

  X_0_SwingFootInitial = mpc_state_.X_0_Initial_SwingFoot;
  if(kfoot != 0)
  {
    X_0_SwingFootInitial = mpc_state_.Get_CorrectedFootstep(kfoot-1) ;
  }

  sva::PTransformd X_0_SwingFootTarget;
  if(kfoot < mpc_state_.opti_steps.size())
  {
    X_0_SwingFootTarget = mpc_state_.Get_CorrectedFootstep(kfoot);
  }
  else
  {
    X_0_SwingFootTarget = mpc_state_.Get_PlannedFootstep(kfoot);
  }


  if(Swing_Foot_Contact)
  {
    t_lift = PrevStepTiming + Tds;
  }

  Eigen::Vector3d SupportFootPose = robot().surfacePose(supportFootName).translation();
  SupportFootPose.z() = mc_rbdyn::rpyFromMat(robot().surfacePose(supportFootName).rotation()).z();

  std::string sensorName = swingFootName + "ForceSensor";
  const auto & sensor = robot().forceSensor(sensorName);

  if(Swing_Foot_Contact)
  {

    if(t > PrevStepTiming + Tds)
    {

      mc_rtc::log::success("lifting " + swingFootName);
      auto contact_state = mc_tasks::lipm_stabilizer::ContactState::Right;
      if(swingFootName == "RightFoot"){contact_state = mc_tasks::lipm_stabilizer::ContactState::Left;}
      StabTask->setContacts({contact_state});

      removeContact({robot().name(), "ground", swingFootName, "AllGround", 0.7, footcontact_dof});

      t_lift = t;

      std::string swingFootLinkName = LeftFootLinkName_;
      if(swingFootName == "RightFoot")
      {
        swingFootLinkName = RightFootLinkName_;
      }
      DoubleSupport_state = false;
      Swing_Foot_Contact = false;
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

  sva::MotionVecd V_0_FootTask_Target = SwingFootTrajectory.GetVelocity();

  const Eigen::Vector3d & SwingFoot_rpy_real = mc_rbdyn::rpyFromMat(realRobot().surfacePose(swingFootName).rotation());
  Eigen::Vector3d SwingFoot_rpy_initial_real = mc_rbdyn::rpyFromMat(X_0_SwingFootInitial_real.rotation());
  const Eigen::Vector3d & SwingFoot_rpy_target = mc_rbdyn::rpyFromMat(X_0_FootTask_Target.rotation());
  Eigen::Vector3d SwingFoot_delta_rpy = SwingFoot_rpy_real - SwingFoot_rpy_initial_real;
  SwingFoot_delta_rpy.z() = 0.;
  SwingFoot_rpy_initial_real.z() = 0.;

  if(UseRealRobot)
  {
    X_0_FootTask_Target.rotation() =
        mc_rbdyn::rpyToMat(SwingFoot_rpy_initial_real - SwingFoot_delta_rpy + SwingFoot_rpy_target);
    // X_0_FootTask_Target.rotation() = mc_rbdyn::rpyToMat( -SwingFoot_delta_rpy + SwingFoot_rpy_target);
  }

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

      t_contact = t;

      StabTask->setContacts(
          {mc_tasks::lipm_stabilizer::ContactState::Left, mc_tasks::lipm_stabilizer::ContactState::Right});
      DoubleSupport_state = true;

      mc_rtc::log::info("height : {} ", swing_foot_height);
      mc_rtc::log::info("touchdown : {} ", TouchDown);
      // mc_rtc::log::info("Locking " + swingFootName + "at t : " + std::to_string(t));
      mc_rtc::log::info("T_contact - T_steps : {}", t - NextTimeStep);


      Eigen::Vector6d dof;
      dof << 0, 0, 1, 1, 1, 0;
      addContact({robot().name(), "ground", swingFootName, "AllGround", 0.7, dof});


      mc_rtc::log::success("Locked " + swingFootName);

      Swing_Foot_Contact = true;
      switchFootSupport();
      updateTasks();

      // PrevStepTiming = NextTimeStep;
      // if(kfoot + 1 < mpc_state_.planned_steps().size())
      // {
      //   kfoot += 1;
      // }

      N_Steps += 1;
      if(N_Steps == N_Steps_Desired)
      {
        Stop = true;
      }
      t_k = 0;
      countStart = count - 1;

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
  
    SwingFootInitialPose = robot().surfacePose(swingFootName).translation();
    SwingFootInitialAngle = mc_rbdyn::rpyFromMat(robot().surfacePose(swingFootName).rotation()).z();

  }

  return 0;
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

  }

  else
  {

    supportFootName = "LeftFoot";
    swingFootName = "RightFoot";

  }
}