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


  Eigen::Vector3d SupportFootPose = robot().surfacePose(supportFootName+ "Center").translation();
  SupportFootPose.z() = mc_rbdyn::rpyFromMat(robot().surfacePose(supportFootName+ "Center").rotation()).z();

  std::string sensorName = swingFootName + "ForceSensor";
  const auto & sensor = robot().forceSensor(sensorName);
  const auto & sensor_support = robot().forceSensor(supportFootName + "ForceSensor");

  if(Swing_Foot_Contact)
  {
    t_lift = PrevStepTiming + mpc_state_.get_tds();
    if(t - 0 * controller_config_.delta >= PrevStepTiming + mpc_state_.get_tds() && (std::abs(sensor_support.force().z()) > 50 || !force_contact_safety_ ))
    {

      mc_rtc::log::success("lifting " + swingFootName);
      solver().addTask(SwingFootTask);
      Eigen::Vector3d supp_pose;
      double supp_yaw;
      supp_pose = robot().surfacePose(supportFootName + "Center").translation();
      supp_pose.z() = 0.0;
      supp_yaw = mc_rbdyn::rpyFromMat(robot().surfacePose(supportFootName + "Center").rotation()).z();
      if(supportFootName == "LeftFoot")
      {
        StabTask->setContacts(
            {{mc_tasks::lipm_stabilizer::ContactState::Left, sva::PTransformd(sva::RotZ(supp_yaw), supp_pose)}});
      }
      else
      {
        StabTask->setContacts(
            { {mc_tasks::lipm_stabilizer::ContactState::Right, sva::PTransformd(sva::RotZ(supp_yaw), supp_pose)}});
      }

      removeContact({robot().name(), "ground", swingFootName+"Center", "AllGround", 0.7, footcontact_dof});

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
  double height_off = X_0_support.translation().z();
  SwingFootTrajectory.set_Z_ContactOffset(height_off);

  SwingFootTrajectory.getSwingFootTrajectory(X_0_SwingFootTarget, X_0_SwingFootInitial, t,
                                             controller_config_.FootStepHeight, SingleSupportDuration, t_lift,
                                             controller_timestep);

  SwingFootVel = SwingFootTrajectory.GetVelocity().linear();
  SwingFootAcc = SwingFootTrajectory.GetAccel().linear();
  sva::PTransformd X_0_FootTask_Target = SwingFootTrajectory.GetTrajectory();

  sva::MotionVecd V_0_FootTask_Target = SwingFootTrajectory.GetVelocity();
  sva::MotionVecd A_0_FootTask_Target = SwingFootTrajectory.GetAccel();

  const Eigen::Vector3d & SwingFoot_rpy_real = mc_rbdyn::rpyFromMat(realRobot().surfacePose(swingFootName+ "Center").rotation());
  Eigen::Vector3d SwingFoot_rpy_initial_real = mc_rbdyn::rpyFromMat(X_0_SwingFootInitial_real.rotation());
  const Eigen::Vector3d & SwingFoot_rpy_target = mc_rbdyn::rpyFromMat(X_0_FootTask_Target.rotation());
  Eigen::Vector3d SwingFoot_delta_rpy = Eigen::Vector3d{SwingFoot_rpy_real.x(),SwingFoot_rpy_real.y(),0} ;
  SwingFoot_delta_rpy.z() = 0.;
  SwingFoot_rpy_initial_real.z() = 0.;

  // if(UseRealRobot)
  // {
  //   X_0_FootTask_Target.rotation() =
  //       mc_rbdyn::rpyToMat(SwingFoot_rpy_initial_real - SwingFoot_delta_rpy + SwingFoot_rpy_target);
  //   // X_0_FootTask_Target.rotation() = mc_rbdyn::rpyToMat( -SwingFoot_delta_rpy + SwingFoot_rpy_target);
  // }

  X_0_FootTask_Target.rotation() =
      mc_rbdyn::rpyToMat( - SwingFoot_delta_rpy + SwingFoot_rpy_target);

  SwingFootTask->target(X_0_FootTask_Target);
  SwingFootTask->refVelB(sva::PTransformd(X_0_swing.rotation()) * V_0_FootTask_Target);
  // SwingFootTask->refAccel(sva::MotionVecd(Eigen::Vector3d::Zero(),A_0_FootTask_Target.linear()));
  SwingFootTask->refAccel(sva::PTransformd(X_0_swing.rotation()) *sva::MotionVecd(Eigen::Vector3d::Zero(),A_0_FootTask_Target.linear()));

  if(!Swing_Foot_Contact)
  {
    if(controller_config_.FootStepHeight - X_0_FootTask_Target.translation().z() < 0.005)
    {
      vertical_force_measure_.push_back((float) SwingFootTask->frame().forceSensor().force().z());
      const int size = vertical_force_measure_.size();
      Eigen::VectorXd force_measure = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vertical_force_measure_.data(), vertical_force_measure_.size());
      vertical_force_offset_ = force_measure.mean();
    }

    double Step_Time = t - t_lift;
    double swing_foot_height = robot().surfacePose(swingFootName+ "Center").translation().z();
    double support_foot_height = robot().surfacePose(supportFootName+ "Center").translation().z();
    double foot_diff_height =
        (realRobot().surfacePose(swingFootName+ "Center").translation() - realRobot().surfacePose(supportFootName+ "Center").translation())
            .z();

    bool TouchDown = (SwingFootTask->frame().forceSensor().force().z() - vertical_force_offset_ > controller_config_.impact_threshold);
    // TouchDown = false;

    if(((Step_Time > 0.25 && TouchDown) || Step_Time >= SingleSupportDuration) && !DoubleSupport_state)

    {

      t_contact = t;

      Eigen::Vector3d supp_pose;
      double supp_yaw;
      supp_pose = robot().surfacePose(supportFootName + "Center").translation();
      supp_pose.z() = 0.0;
      supp_yaw = mc_rbdyn::rpyFromMat(robot().surfacePose(supportFootName + "Center").rotation()).z();
      Eigen::Vector3d swing_pose;
      double swing_yaw;
      swing_pose = robot().surfacePose(swingFootName + "Center").translation();
      swing_pose.z() = 0.0;
      swing_yaw = mc_rbdyn::rpyFromMat(robot().surfacePose(swingFootName + "Center").rotation()).z();
      if(supportFootName == "LeftFoot")
      {
        StabTask->setContacts(
            {{mc_tasks::lipm_stabilizer::ContactState::Left, robot().surfacePose(supportFootName+"Center")}, {mc_tasks::lipm_stabilizer::ContactState::Right, sva::PTransformd(sva::RotZ(swing_yaw), swing_pose)}});
      }
      else
      {
        StabTask->setContacts(
            {{mc_tasks::lipm_stabilizer::ContactState::Left, sva::PTransformd(sva::RotZ(swing_yaw), swing_pose)}, {mc_tasks::lipm_stabilizer::ContactState::Right, robot().surfacePose(supportFootName+"Center")}});
      }
      DoubleSupport_state = true;

      mc_rtc::log::info("height : {} ", swing_foot_height);
      mc_rtc::log::info("touchdown : {} ", TouchDown);
      // mc_rtc::log::info("Locking " + swingFootName + "at t : " + std::to_string(t));
      mc_rtc::log::info("T_contact - T_steps : {}", t - NextTimeStep);

      addContact({robot().name(), "ground", swingFootName + "Center", "AllGround", 0.7, footcontact_dof});


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
      t_k = - controller_config_.delta;
      countStart = count - 1;

    }
  }

  if(DoubleSupport_state)
  {
  
    SwingFootInitialPose = robot().surfacePose(swingFootName+ "Center").translation();
    SwingFootInitialAngle = mc_rbdyn::rpyFromMat(robot().surfacePose(swingFootName+ "Center").rotation()).z();

    X_0_SwingFootInitial = sva::PTransformd(sva::RotZ(mc_rbdyn::rpyFromMat(robot().surfacePose(swingFootName+ "Center").rotation()).z()) ,
                                                          robot().surfacePose(swingFootName+ "Center").translation());
    
    X_0_SwingFootInitial = robot().surfacePose(swingFootName+ "Center");


    vertical_force_measure_.clear();

    solver().removeTask(leftSwingFootTask);
    solver().removeTask(rightSwingFootTask);

  }

  return 0;
}

void Walking_controller::updateTasks()
{
  if(supportFootName == "LeftFoot")
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