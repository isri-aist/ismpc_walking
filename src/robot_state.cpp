#include "Walking_controller.h"

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

  const auto & X_0_torso_link = robot().bodyPosW(torsoBodyName_);
  const auto & X_0_torso_reference = ReferenceFrame_Origin_Offset * X_0_torso_link;


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