#include "../include/ismpc_walking/FootTrajectory.h"

FootTrajectory::FootTrajectory(){};


// std::vector<Eigen::Vector3d> FootTrajectory::TrajGUI(){
//   int N = 100;
//   trajgui.clear();
//   for (int i = 0 ; i < N ; i++){

//     double t = ((double) (i/N))*duration;

//     double polyInT_X = m_Polynome_X(0) * pow(t, 5) + m_Polynome_X(1) * pow(t, 4) + m_Polynome_X(2) * pow(t, 3) +
//     m_Polynome_X(3) * pow(t, 2)
//                     + m_Polynome_X(4) * pow(t, 1) + m_Polynome_X(5);
//     double polyInT_Y = m_Polynome_Y(0) * pow(t, 5) + m_Polynome_Y(1) * pow(t, 4) + m_Polynome_Y(2) * pow(t, 3) +
//     m_Polynome_Y(3) * pow(t, 2)
//                     + m_Polynome_Y(4) * pow(t, 1) + m_Polynome_Y(5);

//     Eigen::Vector3d pos;
//     pos(0) = swingFootPosition_0(0) + (swingFootPosition_1(0) - swingFootPosition_0(0)) * polyInT_X;
//     pos(1) = swingFootPosition_0(1) + (swingFootPosition_1(1) - swingFootPosition_0(1)) * polyInT_Y;
//     pos(2) = 0;

//   }

// }

Eigen::VectorXd FootTrajectory::GetCoeffs(double t, double dur, Eigen::VectorXd boundaryConditions, bool IneqCstr_On)
{
  Eigen::VectorXd Output(3);

  int poly_dim = 6;

  Aeq = Eigen::MatrixXd::Zero(6, poly_dim);



  // Aeq << 0, 0, 0, 0, 0, 0, 1,
  //        pow(dur, 6), pow(dur, 5), pow(dur, 4), pow(dur, 3), pow(dur, 2), pow(dur, 1), 1,
  //        0, 0, 0, 0, 0, 1, 0,
  //        6 * pow(dur, 5), 5 * pow(dur, 4), 4 * pow(dur, 3), 3 * pow(dur, 2), 2 * pow(dur, 1), 1, 0,
  //        0, 0, 0, 0, 2, 0, 0,
  //        6 * 5 * pow(dur, 4) ,5 * 4 * pow(dur, 3), 4 * 3 * pow(dur, 2), 3 * 2 * pow(dur, 1), 2, 0, 0;

  Aeq << 0, 0, 0, 0, 0, 1, pow(dur, 5), pow(dur, 4), pow(dur, 3), pow(dur, 2), pow(dur, 1), 1, 0, 0, 0, 0, 1, 0,
      5 * pow(dur, 4), 4 * pow(dur, 3), 3 * pow(dur, 2), 2 * pow(dur, 1), 1, 0, 0, 0, 0, 2, 0, 0, 5 * 4 * pow(dur, 3),
      4 * 3 * pow(dur, 2), 3 * 2 * pow(dur, 1), 2, 0, 0;

  coeffs = Aeq.inverse() * boundaryConditions;



  double polyInT(0);
  double polyDotInT(0);
  double polyDotDotInT(0);

  for(int deg = 0; deg < poly_dim; deg++)
  {

    polyInT += coeffs(deg) * pow(t, poly_dim - 1 - deg);

    polyDotInT += (poly_dim - 1 - deg) * coeffs(deg) * pow(t, poly_dim - 1 - deg - 1);

    polyDotDotInT += (poly_dim - 1 - deg) * (poly_dim - 1 - deg - 1) * coeffs(deg) * pow(t, poly_dim - 1 - deg - 2);
  }

  Output << polyInT, polyDotInT, polyDotDotInT;

  return Output;
}

std::vector<Eigen::Vector3d> FootTrajectory::getSwingFootTrajectory(const sva::PTransformd & X_0_StepTarget,
                                                                    const sva::PTransformd & X_0_StartPose,
                                                                    double t,
                                                                    double stepHeight,
                                                                    double dur,
                                                                    double t0,
                                                                    double delta)
{

  t -= (t0);

  
  double tinit = 0;

  std::vector<Eigen::Vector3d> Output;

  if(t < 0)
  {
    duration = dur;
    prev_dur = dur;
    m_t = t;
    duration_Z = dur * 0.5;

    zf = stepHeight + X_0_StartPose.translation().z();
    Z_up = false;

    Eigen::MatrixXd R_0_swingFootStartPos = X_0_StartPose.rotation();
    double roll = atan2(R_0_swingFootStartPos(1, 2), R_0_swingFootStartPos(2, 2));
    double pitch = -asin(R_0_swingFootStartPos(0, 2));
    double yaw = atan2(R_0_swingFootStartPos(0, 1), R_0_swingFootStartPos(0, 0));

    swingFootPosition(0) = X_0_StartPose.translation().x();
    swingFootPosition(1) = X_0_StartPose.translation().y();
    swingFootPosition(2) = X_0_StartPose.translation().z();
    swingFootVelocity.setZero();
    swingFootAcc.setZero();

    swingFootOrientation(0) = roll;
    swingFootOrientation(1) = pitch;
    swingFootOrientation(2) = yaw;
    swingFootOrientationVelocity.setZero();
    swingFootOrientationAcc.setZero();

    swingFootPosition_0 = swingFootPosition;
    swingFootVelocity_0 = swingFootVelocity;
    swingFootAcc_0 = swingFootAcc;

    swingFootOrientation_0 = swingFootOrientation;
    swingFootOrientationVelocity_0 = swingFootOrientationVelocity;
    swingFootOrientationAcc_0 = swingFootOrientationAcc;

    swingFootPosition_1 = X_0_StepTarget.translation();
    swingFootPosition_1.z() = 0;
    swingFootOrientation_1.z() = atan2(X_0_StepTarget.rotation()(0, 1), X_0_StepTarget.rotation()(0, 0));
  }

  else if(t >= dur)
  {
    // swingFootPosition(0) = X_0_StepTarget(0);
    // swingFootPosition(1) = X_0_StepTarget(1);
    // swingFootPosition(2) = 0;
    swingFootVelocity.setZero();
    swingFootAcc.setZero();

    swingFootOrientationVelocity.setZero();
    swingFootOrientationAcc.setZero();
  }

  else
  {
    // Generate Spline_XY polynomial for single support

    Eigen::VectorXd boundaryConditions_X(6);
    Eigen::VectorXd boundaryConditions_Y(6);
    Eigen::VectorXd boundaryConditions_Z(6);
    Eigen::VectorXd boundaryConditions_w(6);
    Eigen::VectorXd poly_X;
    Eigen::VectorXd poly_Y;
    Eigen::VectorXd poly_Z;
    Eigen::VectorXd poly_w;

    Eigen::VectorXd boundaryConditions_pitch(6);
    Eigen::VectorXd poly_pitch(6);
    poly_pitch.setZero();

    // if (prev_dur != dur){mc_rtc::log::info("diif t");}
    double d_footpose = Eigen::Vector2d{(X_0_StepTarget.translation() - swingFootPosition_1)(0),
                                        (X_0_StepTarget.translation() - swingFootPosition_1)(1)}
                            .norm();

    // std::cout << "ss d_dur" << std::abs(prev_dur - dur) << std::endl;
    if(d_footpose > 1e-2 || ( std::abs(prev_dur - dur) > 0.05) && dur > 0.2)
    {
      New_traj = true;
    }

    if((m_t < duration_Z + delta && m_t > duration_Z) && !Z_up)
    {
      New_traj = true;
      Z_up = true;
    }

    if((New_traj))
    {

      duration = dur - t;
      prev_dur = dur;
      m_t = delta;

      if(!Z_up && dur * 0.5 - t > 0.1)
      {
        duration_Z = dur * 0.5 - t;
        zf = stepHeight + X_0_StartPose.translation().z();

        pitch_f = 0;
      }
      else
      {
        Z_up = true;
      }
      if(Z_up || duration_Z <= 0.05)
      {
        duration_Z = dur - t;
        zf = 0.;
        Z_up = true;

        pitch_f = 0;
      }

      std::cout << "change traj, new duration Z: " << duration_Z << std::endl;

      swingFootPosition_1 = X_0_StepTarget.translation();
      swingFootPosition_1.z() = 0;
      swingFootOrientation_1.z() = atan2(X_0_StepTarget.rotation()(0, 1), X_0_StepTarget.rotation()(0, 0));

      swingFootPosition_0 = swingFootPosition;

      swingFootVelocity_0 = swingFootVelocity;
      swingFootAcc_0 = swingFootAcc;

      swingFootOrientation_0 = swingFootOrientation;
      swingFootOrientationVelocity_0 = swingFootOrientationVelocity;
      swingFootOrientationAcc_0 = swingFootOrientationAcc;

      New_traj = false;
    }
    else
    {
      m_t += delta;
    }

    if(std::abs((swingFootOrientation_0 - swingFootOrientation_1).z()) > M_PI)
    {
      swingFootOrientation_1.z() -= (swingFootOrientation_1.z() / std::abs(swingFootOrientation_1.z())) * 2 * M_PI;
    }

    boundaryConditions_X << swingFootPosition_0(0), swingFootPosition_1(0), swingFootVelocity_0(0), 0,
        swingFootAcc_0(0), 0;
    boundaryConditions_Y << swingFootPosition_0(1), swingFootPosition_1(1), swingFootVelocity_0(1), 0,
        swingFootAcc_0(1), 0;
    boundaryConditions_w << swingFootOrientation_0(2), swingFootOrientation_1(2), swingFootOrientationVelocity_0(2), 0,
        swingFootOrientationAcc_0(2), 0;

    poly_X = GetCoeffs(m_t, duration, boundaryConditions_X, false);

    prev_poly_X = poly_X;

    poly_Y = GetCoeffs(m_t, duration, boundaryConditions_Y, false);
 
    prev_poly_Y = poly_Y;

    poly_w = GetCoeffs(m_t, duration, boundaryConditions_w, false);
 
    prev_poly_w = poly_w;

    boundaryConditions_Z << swingFootPosition_0(2), zf, swingFootVelocity_0(2), 0, swingFootAcc_0(2), 0;

    poly_Z = GetCoeffs(m_t, duration_Z, boundaryConditions_Z, true);
 
    prev_poly_Z = poly_Z;

    // boundaryConditions_pitch << swingFootOrientation_0(1) , pitch_f ,
    //                         swingFootOrientationVelocity_0(1) , 0  ,
    //                         swingFootOrientationAcc_0(1) , 0;
    // poly_pitch = GetCoeffs(m_t,duration_Z,boundaryConditions_pitch,true);
    // if(!QPsuccess){
    //   poly_pitch = prev_poly_pitch;
    //   // m_Polynome_Z = coeffs;
    // }
    // prev_poly_pitch = poly_pitch;

    swingFootPosition(0) = poly_X(0);
    swingFootPosition(1) = poly_Y(0);
    swingFootPosition(2) = poly_Z(0);

    swingFootOrientation(0) = 0;
    swingFootOrientation(1) = 0;
    swingFootOrientation(2) = poly_w(0);

    if(std::abs(swingFootOrientation(2)) > M_PI)
    {
      swingFootOrientation(2) -= (swingFootOrientation(2) / std::abs(swingFootOrientation(2))) * 2 * M_PI;
    }

    swingFootVelocity(0) = poly_X(1);
    swingFootVelocity(1) = poly_Y(1);
    swingFootVelocity(2) = poly_Z(1);

    swingFootOrientationVelocity(0) = 0;
    swingFootOrientationVelocity(1) = 0;
    swingFootOrientationVelocity(2) = poly_w(1);

    swingFootAcc(0) = poly_X(2);
    swingFootAcc(1) = poly_Y(2);
    swingFootAcc(2) = poly_Z(2);
  }

  Output.push_back(swingFootPosition);
  Output.push_back(swingFootOrientation);
  Output.push_back(swingFootVelocity);
  Output.push_back(swingFootOrientationVelocity);
  Output.push_back(swingFootAcc);
  Output.push_back(swingFootOrientationAcc);

  

  return Output;
}


