#include "../include/ismpc_walking/Walking_controller.h"

bool Walking_controller::MoveFeet(double t)
{

  if(mpc_state_.TimeStamps.size() == 0)
  {
    return 0;
  }

  PrevStepTiming = 0;
  double NextTimeStep(0);

  NextTimeStep = mpc_state_.get_Ts(kfoot);

  // if(kfoot != 0)
  // {
  //   PrevStepTiming = mpc_state_.get_Ts(kfoot-1);
  //   X_0_SwingFootInitial = mpc_state_.Get_CorrectedFootstep(kfoot-1) ;
  // }

  sva::PTransformd X_0_SwingFootTarget;
  if(kfoot < mpc_state_.opti_steps.size())
  {
    X_0_SwingFootTarget = mpc_state_.Get_CorrectedFootstep(kfoot);
  }
  else
  {
    X_0_SwingFootTarget = mpc_state_.Get_PlannedFootstep(kfoot);
  }

  Eigen::Vector3d SupportFootPose = robot().surfacePose(supportFootName).translation();
  SupportFootPose.z() = mc_rbdyn::rpyFromMat(robot().surfacePose(supportFootName).rotation()).z();

  std::string swingFoot_sensorName = "RightFootForceSensor";
  std::string supportFoot_sensorName = "LeftFootForceSensor";
  if(swingFootName == leftFootName_)
  {
    swingFoot_sensorName = "LeftFootForceSensor";
    supportFoot_sensorName = "RightFootForceSensor";
  }
  const auto & sensor = robot().forceSensor(swingFoot_sensorName);
  const auto & sensor_support = robot().forceSensor(supportFoot_sensorName);

  if(Swing_Foot_Contact)
  {
    t_lift = PrevStepTiming + mpc_state_.get_tds();
    if(t - 0 * controller_config_.delta >= PrevStepTiming + mpc_state_.get_tds()
       && (std::abs(sensor_support.force().z()) > 50 || !force_contact_safety_))
    {

      mc_rtc::log::success("lifting " + swingFootName);
      solver().addTask(SwingFootTask);
      Eigen::Vector3d supp_pose;
      double supp_yaw;
      supp_pose = robot().surfacePose(supportFootName).translation();
      supp_pose.z() = 0.0;
      supp_yaw = mc_rbdyn::rpyFromMat(robot().surfacePose(supportFootName).rotation()).z();
      if(supportFootName == leftFootName_)
      {
        stabTask->setContacts(
            {{mc_tasks::lipm_stabilizer::ContactState::Left, sva::PTransformd(sva::RotZ(supp_yaw), supp_pose)}});
      }
      else
      {
        stabTask->setContacts(
            {{mc_tasks::lipm_stabilizer::ContactState::Right, sva::PTransformd(sva::RotZ(supp_yaw), supp_pose)}});
      }

      removeContact({robot().name(), "ground", swingFootName, "AllGround", 0.7, footcontact_dof});

      t_lift = t;

      std::string swingFootLinkName = LeftFootLinkName_;
      if(swingFootName == rightFootName_)
      {
        swingFootLinkName = RightFootLinkName_;
      }
      DoubleSupport_state = false;
      Swing_Foot_Contact = false;
    }
  }

  double offset = 0;
  double SingleSupportDuration = (NextTimeStep - t_lift) + offset;
  double height_off = X_0_support.translation().z();
  SwingFootTrajectory.set_Z_ContactOffset(height_off);
  SwingFootTrajectory.setZOffset(3e-3);

  SwingFootTrajectory.getSwingFootTrajectory(X_0_SwingFootTarget, X_0_SwingFootInitial, t,
                                             controller_config_.FootStepHeight, SingleSupportDuration, t_lift,
                                             controller_timestep);

  SwingFootVel = SwingFootTrajectory.GetVelocity().linear();
  SwingFootAcc = SwingFootTrajectory.GetAccel().linear();
  sva::PTransformd X_0_FootTask_Target = SwingFootTrajectory.GetTrajectory();

  sva::MotionVecd V_0_FootTask_Target = SwingFootTrajectory.GetVelocity();
  sva::MotionVecd A_0_FootTask_Target = SwingFootTrajectory.GetAccel();

  double swing_yaw = mc_rbdyn::rpyFromMat(X_0_swing.rotation()).z();

  SwingFootTask->target(X_0_FootTask_Target);
  SwingFootTask->refVelB(sva::PTransformd(sva::RotZ(swing_yaw)) * V_0_FootTask_Target);
  SwingFootTask->refAccel(
      sva::MotionVecd(Eigen::Vector3d::Zero(), sva::RotZ(swing_yaw) * A_0_FootTask_Target.linear()));
  // SwingFootTask->refAccel(sva::PTransformd(X_0_swing.rotation())
  // *sva::MotionVecd(Eigen::Vector3d::Zero(),A_0_FootTask_Target.linear()));

  if(!Swing_Foot_Contact)
  {
    if(controller_config_.FootStepHeight - X_0_FootTask_Target.translation().z() < 0.005)
    {
      vertical_force_measure_.push_back((float)SwingFootTask->frame().forceSensor().force().z());
      const int size = vertical_force_measure_.size();
      Eigen::VectorXd force_measure =
          Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vertical_force_measure_.data(), vertical_force_measure_.size());
      vertical_force_offset_ = force_measure.mean();
    }

    double Step_Time = t - t_lift;
    double swing_foot_height = robot().surfacePose(swingFootName).translation().z();
    double support_foot_height = robot().surfacePose(supportFootName).translation().z();
    double foot_diff_height =
        (realRobot().surfacePose(swingFootName).translation() - realRobot().surfacePose(supportFootName).translation())
            .z();

    bool TouchDown = (SwingFootTask->frame().forceSensor().force().z() - vertical_force_offset_
                      > controller_config_.impact_threshold);
    // TouchDown = false;

    if( ((Step_Time > SingleSupportDuration * 0.5 && TouchDown)
        || Step_Time >= SingleSupportDuration - 0*controller_config_.delta)
       && !DoubleSupport_state)

    {

      t_contact = t;

      Eigen::Vector3d supp_pose;
      double supp_yaw;
      supp_pose = robot().surfacePose(supportFootName).translation();
      supp_pose.z() = 0.0;
      supp_yaw = mc_rbdyn::rpyFromMat(robot().surfacePose(supportFootName).rotation()).z();
      Eigen::Vector3d swing_pose;
      double swing_yaw;
      swing_pose = robot().surfacePose(swingFootName).translation();
      swing_pose.z() = 0.0;
      swing_yaw = mc_rbdyn::rpyFromMat(robot().surfacePose(swingFootName).rotation()).z();
      if(supportFootName == leftFootName_)
      {
        stabTask->setContacts(
            {{mc_tasks::lipm_stabilizer::ContactState::Left, sva::PTransformd(sva::RotZ(supp_yaw), supp_pose)},
             {mc_tasks::lipm_stabilizer::ContactState::Right, sva::PTransformd(sva::RotZ(swing_yaw), swing_pose)}});
      }
      else
      {
        stabTask->setContacts(
            {{mc_tasks::lipm_stabilizer::ContactState::Left, sva::PTransformd(sva::RotZ(swing_yaw), swing_pose)},
             {mc_tasks::lipm_stabilizer::ContactState::Right, sva::PTransformd(sva::RotZ(supp_yaw), supp_pose)}});
      }

      DoubleSupport_state = true;
      mc_rtc::log::info("height : {} ", swing_foot_height);
      mc_rtc::log::info("touchdown : {} ", TouchDown);
      // mc_rtc::log::info("Locking " + swingFootName + "at t : " + std::to_string(t));
      mc_rtc::log::info("T_contact - T_steps : {}", t - NextTimeStep);

      mc_rtc::log::success("Locked " + swingFootName);

      // if( std::abs( mc_rbdyn::rpyFromMat(realRobot().surfacePose(swingFootName).rotation()).x() - mc_rbdyn::rpyFromMat(robot().surfacePose(swingFootName).rotation()).x() ) > controller_config_.safety_roll_error_ )
      // {
      //   mc_rtc::log::error("Robot is about to fall, stoping");
      //   Stop = true;
      // }

      // PrevStepTiming = NextTimeStep;
      // if(kfoot + 1 < mpc_state_.planned_steps().size())
      // {
      //   kfoot += 1;
      // }
    }
    if(DoubleSupport_state && t - t_contact >= 0.05)
    {
      addContact({robot().name(), "ground", swingFootName, "AllGround", 0.7, footcontact_dof});

      switchFootSupport();
      updateTasks();

      N_Steps += 1;

      Swing_Foot_Contact = true;
      if(N_Steps == N_Steps_Desired)
      {
        Stop = true;
      }
      t_k = -controller_config_.delta;
      countStart = count - 1;
    }
  }

  if(DoubleSupport_state)
  {

    SwingFootInitialPose = robot().surfacePose(swingFootName).translation();
    SwingFootInitialAngle = mc_rbdyn::rpyFromMat(robot().surfacePose(swingFootName).rotation()).z();

    X_0_SwingFootInitial =
        sva::PTransformd(sva::RotZ(mc_rbdyn::rpyFromMat(robot().surfacePose(swingFootName).rotation()).z()),
                         robot().surfacePose(swingFootName).translation());

    X_0_SwingFootInitial = robot().surfacePose(swingFootName);

    vertical_force_measure_.clear();

    solver().removeTask(leftSwingFootTask);
    solver().removeTask(rightSwingFootTask);

  }

  return 0;
}

void Walking_controller::updateTasks()
{
  if(supportFootName == leftFootName_)
  {
    SwingFootTask = rightSwingFootTask;
  }
  else
  {
    SwingFootTask = leftSwingFootTask;
  }
  Eigen::MatrixXd dimW(Eigen::VectorXd::Zero(6));
  dimW(5) = 1;
  SwingFootTask->weight(controller_config_.SwingFootWeight);
  SwingFootTask->dimWeight(Eigen::VectorXd::Ones(6));
  SwingFootTask->stiffness(controller_config_.SwingFootStiffness);
}

void Walking_controller::switchFootSupport()
{
  if(supportFootName == leftFootName_)
  {
    supportFootName = rightFootName_;
    swingFootName = leftFootName_;
  }

  else
  {

    supportFootName = leftFootName_;
    swingFootName = rightFootName_;
  }
}