#include "ISMPC_Solver.h"

ISMPC_Solver::ISMPC_Solver() {}

ISMPC_Solver::ISMPC_Solver(double delta_controller, double delta, double Tp, double Tc)
{

  m_Tc = Tc;
  m_Tp = Tp;
  m_delta = delta;
  m_delta_control = delta_controller;

  m_eta = sqrt(9.8 / 0.78);
  count_Dstep = 0;
  m_C = (int)std::round((m_Tc) / m_delta);
  m_P = (int)std::round((m_Tp) / m_delta);
  QPsuccess = false;

  ZMP_Lower_Limits_X.resize(m_C, 1);
  ZMP_Upper_Limits_X.resize(m_C, 1);
  ZMP_Lower_Limits_Y.resize(m_C, 1);
  ZMP_Upper_Limits_Y.resize(m_C, 1);
  ZMP_Lower_Limits_X.setZero();
  ZMP_Lower_Limits_Y.setZero();
  ZMP_Upper_Limits_X.setZero();
  ZMP_Upper_Limits_Y.setZero();

  Integration_Mat.setZero();
  Integration_Mat(0, 0) = std::cosh(m_eta * m_delta_control);
  Integration_Mat(0, 1) = std::sinh(m_eta * m_delta_control) / m_eta;
  Integration_Mat(0, 2) = 1 - std::cosh(m_eta * m_delta_control);
  Integration_Mat(1, 0) = m_eta * std::sinh(m_eta * m_delta_control);
  Integration_Mat(1, 1) = std::cosh(m_eta * m_delta_control);
  Integration_Mat(1, 2) = -m_eta * std::sinh(m_eta * m_delta_control);
  Integration_Mat(2, 0) = 0;
  Integration_Mat(2, 1) = 0;
  Integration_Mat(2, 2) = 1;

  Integration_Vec = Eigen::Vector3d{m_delta_control - (std::sinh(m_eta * m_delta_control) / m_eta),
                                    1 - std::cosh(m_eta * m_delta_control), m_delta_control};
}

void ISMPC_Solver::InitStepGen(const Eigen::VectorXd & Xf, const Eigen::VectorXd & Yf, const Eigen::VectorXd & Thetaf)
{
  m_Xf = Xf;
  m_Yf = Yf;
  Offset.setZero();
  Offset = Eigen::Vector3d{m_Xf(0), m_Yf(0), 0};

  for(int i = 0; i < m_Xf.size(); i++)
  {
    m_Xf(i) -= Offset.x();
    m_Yf(i) -= Offset.y();
  }
  m_Theta_f = Thetaf;
}

void ISMPC_Solver::SetWalkingParameters(const Eigen::Vector3d & Pck,
                                        const Eigen::Vector3d & Vck,
                                        const Eigen::Vector3d & Pzk,
                                        const Eigen::Vector3d & Pfm1,
                                        const std::vector<double> & timesstp,
                                        const std::vector<int> & timesindx,
                                        std::string Tail,
                                        int Steps_Desired,
                                        int Steps)
{
  P_c_k = Pck - Offset;
  V_c_k = Vck;
  P_z_k = Pzk - Offset;
  m_Tail = Tail;
  m_Tail_save = Tail;
  m_eta = sqrt(g / P_c_k.z());
  P_u_k = P_c_k + (V_c_k / m_eta);
  m_Pfm1 = Pfm1 - Offset;
  m_timestamp = timesstp;
  m_timesIndex = timesindx;
  N_Steps = Steps;
  N_Steps_Desired = Steps_Desired;
}

void ISMPC_Solver::ZMP_Constraints()
{

  std::vector<Eigen::VectorXd> b_zmp_ineq;
  std::vector<Eigen::MatrixXd> Normal_zmp_Vec;

  Rectangle Sliding_rect = Rectangle(Eigen::Vector3d{0, 0, m_Theta_f[j_f]}, Eigen::Vector3d{m_dx, m_dy, 0});
  SupportPolygon Poly_Rect(Sliding_rect);

  Rectangle Rect_jm1 = Rectangle(m_Pfm1, Eigen::Vector3d{m_dx, m_dy, 0});
  Rectangle Rect_j = Rectangle(Eigen::Vector3d{m_Xf(0), m_Yf(0), m_Theta_f(0)}, Eigen::Vector3d{m_dx, m_dy, 0});
  SupportPolygon SuppPoly(Rect_jm1, Rect_j);
  SupportPolygon S_Support_Poly(
      Rectangle(Eigen::Vector3d{m_Xf(0), m_Yf(0), m_Theta_f(0)}, Eigen::Vector3d{m_dx, 0.5 * m_dy, 0}));

  Eigen::MatrixXd Delta; // Matrix to derive the ZMP position to ZMP velocity
  Delta = Eigen::MatrixXd::Identity(2 * (m_C + j_Max_C), 2 * (m_C + j_Max_C));
  P_u_k_min.setZero();
  P_u_k_max.setZero();
  P_u_k_max = m_eta * m_delta * P_z_k;
  P_u_k_min = m_eta * m_delta * P_z_k;
  double NextStepTiming(0);
  if(m_timestamp.size() != 0)
  {
    NextStepTiming = m_timestamp[j_f];
  }
  double PrevStepTime = 0;

  for(int i = 0; i < m_C; i++)
  {

    double DD = (double)m_D;
    for(int k = 0; k <= i; k++)
    {
      Delta(2 * i, 2 * k) = m_delta;
      Delta(2 * i + 1, 2 * k + 1) = m_delta;
    }

    double nn = std::max(0.0, std::min((double)count_Dstep, DD + 1));

    // mc_rtc::log::info("i : {} ; DD : {} ; nn : {} ; C : {} ",i,j_f,DD,nn,m_C);

    Eigen::Vector3d ZMP_Zone;

    if(j_f == 0 || !AutoFootstepPlacement)
    {
      Eigen::Vector3d P_f_j{m_Xf(j_f), m_Yf(j_f), 0};
      Eigen::Vector3d P_f_jm1 = m_Pfm1;
      P_f_jm1 = P_z_k;
      if(j_f > 0)
      {
        P_f_jm1 = Eigen::Vector3d{m_Xf(j_fm1), m_Yf(j_fm1), 0.};
      }
      if(N_Steps == N_Steps_Desired)
      {
        P_f_j = (P_f_j + P_f_jm1) / 2;
        Rect_j = Rectangle(P_f_j, Eigen::Vector3d{m_dx, m_dy, 0});
        SuppPoly = SupportPolygon(Rect_jm1, Rect_j);
      }

      ZMP_Zone = (P_f_j * nn + P_f_jm1 * (DD + 1 - nn)) / (DD + 1);
      ZMP_Zone.z() = 0;

      P_u_k_min += m_eta * std::exp(-m_eta * m_delta * (i + 1))
                   * (ZMP_Zone - sva::RotZ(-m_Theta_f(j_f)) * Eigen::Vector3d{m_dx / 2, m_dy / 2, 0}) * m_delta;
      P_u_k_max += m_eta * std::exp(-m_eta * m_delta * (i + 1))
                   * (ZMP_Zone + sva::RotZ(-m_Theta_f(j_f)) * Eigen::Vector3d{m_dx / 2, m_dy / 2, 0}) * m_delta;

      ZMP_Zone.z() = m_Theta_f(0);

      SupportPolygon Polygon(SuppPoly);
      if(Slide_ZMP_region)
      {
        Polygon = SupportPolygon(Rectangle(ZMP_Zone, Eigen::Vector3d{m_dx, m_dy, 0}));
      }
      if(nn == DD + 1)
      {
        Polygon = S_Support_Poly;
      }
      if(i == 0)
      {
        SuppPolyCorners = Polygon.Get_Polygone_Corners();
      }

      Normal_zmp_Vec.push_back(Polygon.SupportPolygone_Normals.transpose());
      b_zmp_ineq.push_back(Polygon.Offset
                           - Polygon.SupportPolygone_Normals.transpose() * Eigen::Vector2d{P_z_k.x(), P_z_k.y()});
    }

    else if(j_f == 1)
    {

      Eigen::Vector3d P_f_jm1{m_Xf(0), m_Yf(0), 0};
      Eigen::Vector3d P_f_j_min = Eigen::Vector3d{m_Xf(j_f), m_Yf(j_f), 0}
                                  - (sva::RotZ(-m_Theta_f(j_f)) * Eigen::Vector3d{m_dx_f / 2, m_dy_f / 2, 0});
      Eigen::Vector3d P_f_j_max = Eigen::Vector3d{m_Xf(j_f), m_Yf(j_f), 0}
                                  + (sva::RotZ(-m_Theta_f(j_f)) * Eigen::Vector3d{m_dx_f / 2, m_dy_f / 2, 0});
      Eigen::Vector3d P_f_j_med{m_Xf(j_f), m_Yf(j_f), 0};

      Eigen::Vector3d ZMP_Zone_min = (P_f_j_min * nn + P_f_jm1 * (DD + 1 - nn)) / (DD + 1);
      Eigen::Vector3d ZMP_Zone_med = (P_f_j_med * nn + P_f_jm1 * (DD + 1 - nn)) / (DD + 1);
      Eigen::Vector3d ZMP_Zone_max = (P_f_j_max * nn + P_f_jm1 * (DD + 1 - nn)) / (DD + 1);

      ZMP_Zone = P_f_jm1 * (DD + 1 - nn) / (DD + 1);
      Eigen::Vector2d Dstep_state = Eigen::Vector2d{nn / (DD + 1), nn / (DD + 1)};

      if(N_Steps + j_f == N_Steps_Desired)
      {
        ZMP_Zone = P_f_jm1 * (DD + 1 - 0.5 * nn) / (DD + 1);
        Dstep_state *= 0.5;
      }
      ZMP_Zone.z() = 0;

      P_u_k_min += m_eta * std::exp(-m_eta * m_delta * (i + 1))
                   * (ZMP_Zone_min - sva::RotZ(-m_Theta_f(j_f)) * Eigen::Vector3d{m_dx / 2, m_dy / 2, 0}) * m_delta;
      P_u_k_max += m_eta * std::exp(-m_eta * m_delta * (i + 1))
                   * (ZMP_Zone_max + sva::RotZ(-m_Theta_f(j_f)) * Eigen::Vector3d{m_dx / 2, m_dy / 2, 0}) * m_delta;

      Eigen::VectorXd bcstr =
          Poly_Rect.Offset - Poly_Rect.SupportPolygone_Normals.transpose() * Eigen::Vector2d{P_z_k.x(), P_z_k.y()}
          + Poly_Rect.SupportPolygone_Normals.transpose() * Eigen::Vector2d{ZMP_Zone.x(), ZMP_Zone.y()};

      Normal_zmp_Vec.push_back(Poly_Rect.SupportPolygone_Normals.transpose());
      b_zmp_ineq.push_back(bcstr);

      Delta(2 * i, 2 * m_C + 2 * (j_f - 1)) = -Dstep_state.x();
      Delta(2 * i + 1, 2 * m_C + 2 * (j_f - 1) + 1) = -Dstep_state.y();
    }

    else
    {
      Eigen::Vector2d Dstep_state_jf = Eigen::Vector2d{nn / (DD + 1), nn / (DD + 1)};
      Eigen::Vector2d Dstep_state_jfm1 = Eigen::Vector2d{(DD + 1 - nn) / (DD + 1), (DD + 1 - nn) / (DD + 1)};

      if(N_Steps + j_f == N_Steps_Desired)
      {
        Dstep_state_jf = 0.5 * Dstep_state_jf;
        Dstep_state_jfm1 = Eigen::Vector2d{(DD + 1 - nn) / (DD + 1), (DD + 1 - nn) / (DD + 1)};
      }

      Normal_zmp_Vec.push_back(Poly_Rect.SupportPolygone_Normals.transpose());
      b_zmp_ineq.push_back(Poly_Rect.Offset
                           - Poly_Rect.SupportPolygone_Normals.transpose() * Eigen::Vector2d{P_z_k.x(), P_z_k.y()});

      Delta(2 * i, 2 * m_C + 2 * (j_f - 1)) = -Dstep_state_jf.x();
      Delta(2 * i + 1, 2 * m_C + 2 * (j_f - 1) + 1) = -Dstep_state_jf.y();
      Delta(2 * i, 2 * m_C + 2 * (j_fm1 - 1)) = -Dstep_state_jfm1.x();
      Delta(2 * i + 1, 2 * m_C + 2 * (j_fm1 - 1) + 1) = -Dstep_state_jfm1.y();
    }

    count_Dstep += 1;

    if(m_tk + ((double)i + 1) * m_delta >= NextStepTiming && j_f + 1 < m_timestamp.size())
    {
      m_D = (int)std::round(m_Tds / m_delta);
      if(N_Steps + j_f + 1 <= N_Steps_Desired || N_Steps_Desired < 0)
      {
        j_f = std::min(j_f + 1, j_Max_C);
        j_fm1 = j_f - 1;
        count_Dstep = 1;

        Sliding_rect = Rectangle(Eigen::Vector3d{0, 0, m_Theta_f[j_f]}, Eigen::Vector3d{m_dx, m_dy, 0});
        Poly_Rect = SupportPolygon(Sliding_rect);
      }
      else
      {
        count_Dstep = (double)(m_D + 1);
      }

      NextStepTiming = m_timestamp[j_f];
      PrevStepTime = m_timestamp[j_fm1];
    }
  }

  int N_zmp_cstr = 0;
  for(int k = 0; k < Normal_zmp_Vec.size(); k++)
  {
    N_zmp_cstr += (int)Normal_zmp_Vec[k].rows();
  }
  // mc_rtc::log::success("ZMP cstr computed, Ncstr = {}", N_zmp_cstr);

  Eigen::MatrixXd ZMP_Cstr;
  ZMP_Cstr.resize(N_zmp_cstr, 2 * (m_C + j_Max_C));
  ZMP_Cstr.setZero();

  Eigen::VectorXd b_zmp(N_zmp_cstr);

  int zk = 0;
  int cstr_index = 0;
  for(int i_ineq = 0; i_ineq < Normal_zmp_Vec.size(); i_ineq++)
  {

    Eigen::MatrixXd n_vec = Normal_zmp_Vec[i_ineq];
    Eigen::VectorXd ineq = b_zmp_ineq[i_ineq];

    ZMP_Cstr.block(cstr_index, zk, n_vec.rows(), 2) = n_vec.block(0, 0, n_vec.rows(), 2);
    b_zmp.segment(cstr_index, n_vec.rows()) = b_zmp_ineq[i_ineq].segment(0, n_vec.rows());

    zk += 2;
    cstr_index += n_vec.rows();
  }
  Aineq_zmp = ZMP_Cstr * Delta;
  bineq_zmp = b_zmp;
}

void ISMPC_Solver::FootSteps_Constraints()
{
  std::vector<Eigen::VectorXd> b_step_ineq;
  std::vector<Eigen::MatrixXd> Normal_step_Vec;
  Eigen::MatrixXd Delta = Eigen::MatrixXd::Identity(2 * j_Max_C, 2 * j_Max_C); // Matrix to differentiate two footsteps

  double l = 0;
  for(int i = 0; i < j_Max_C; i++)
  {
    const double & theta_i = m_Theta_f(i);
    Eigen::Matrix2d R_Theta_i_0 = sva::RotZ(-theta_i).block(0, 0, 2, 2);

    if(i + 1 < m_Xf.size())
    {
      l = (R_Theta_i_0.transpose() * Eigen::Vector2d{m_Xf(i + 1) - m_Xf(i), m_Yf(i + 1) - m_Yf(i)}).y();
    }
    else
    {
      l *= -1;
    }

    Rectangle Kinematic_Rectangle = Rectangle(Eigen::Vector3d{0, 0, theta_i}, Eigen::Vector3d{m_dx_f, m_dy_f, 0});
    SupportPolygon Kinematic_Poly = SupportPolygon(Kinematic_Rectangle);
    Normal_step_Vec.push_back(Kinematic_Poly.SupportPolygone_Normals.transpose());
    if(i > 0)
    {
      Delta(2 * i, 2 * (i - 1)) = -1;
      Delta(2 * i + 1, 2 * (i - 1) + 1) = -1;
      b_step_ineq.push_back(Kinematic_Poly.Offset
                            + Kinematic_Poly.SupportPolygone_Normals.transpose() * R_Theta_i_0 * Eigen::Vector2d{0, l});
    }
    else
    {
      b_step_ineq.push_back(Kinematic_Poly.Offset
                            + Kinematic_Poly.SupportPolygone_Normals.transpose() * Eigen::Vector2d{m_Xf(0), m_Yf(0)}
                            + Kinematic_Poly.SupportPolygone_Normals.transpose() * R_Theta_i_0 * Eigen::Vector2d{0, l});
    }
  }

  int N_Footsteps_cstr = 0;
  for(int k = 0; k < Normal_step_Vec.size(); k++)
  {
    N_Footsteps_cstr += (int)Normal_step_Vec[k].rows();
  }

  Eigen::MatrixXd Foosteps_Cstr = Eigen::MatrixXd::Zero(N_Footsteps_cstr, 2 * (j_Max_C));
  Eigen::VectorXd b_steps(N_Footsteps_cstr);
  int step = 0;
  int cstr_index = 0;

  for(int i_ineq = 0; i_ineq < Normal_step_Vec.size(); i_ineq++)
  {

    Eigen::MatrixXd ineq = Normal_step_Vec[i_ineq];

    Foosteps_Cstr.block(cstr_index, step, ineq.rows(), 2) = ineq.block(0, 0, ineq.rows(), 2);
    b_steps.segment(cstr_index, ineq.rows()) = b_step_ineq[i_ineq].segment(0, ineq.rows());

    step += 2;
    cstr_index += ineq.rows();
  }

  Aineq_steps = Eigen::MatrixXd::Zero(N_Footsteps_cstr, 2 * (m_C + j_Max_C));
  Aineq_steps.block(0, 2 * m_C, N_Footsteps_cstr, 2 * j_Max_C) = Foosteps_Cstr * Delta;
  bineq_steps = b_steps;
}

void ISMPC_Solver::AntTailTrajectory()
{
  int PreviewSize = m_P - m_C;
  if(m_timestamp.size() != 0)
  {
    PreviewSize = m_timesIndex[m_timesIndex.size() - 1] - m_C;
  }
  AfterTc_ZMP_trajectory;
  AfterTc_ZMP_trajectory.resize(2 * PreviewSize, 1);
  AfterTc_ZMP_trajectory.setZero();
  double DD = (double)m_D;
  for(int i = 0; i < PreviewSize; i++)
  {

    double PrevStepTiming(0);
    double NextStepTiming(0);
    if(m_timestamp.size() != 0)
    {
      NextStepTiming = m_timestamp[j_f];
    }
    if(j_fm1 != -1)
    {
      PrevStepTiming = m_timestamp[j_fm1];
    }

    Eigen::Vector3d P_f_jm1 = m_Pfm1;
    if(j_fm1 != -1)
    {
      P_f_jm1 = Eigen::Vector3d{m_Xf(j_fm1), m_Yf(j_fm1), 0};
    }
    Eigen::Vector3d P_f_j{m_Xf(j_f), m_Yf(j_f), 0};
    if(N_Steps + j_f == N_Steps_Desired)
    {
      P_f_j = (P_f_j + P_f_jm1) / 2;
    }

    double nn = std::max(1.0, std::min((double)count_Dstep, DD + 1));

    Eigen::Vector3d StepZone = (P_f_j * nn + P_f_jm1 * (DD + 1 - nn)) / (DD + 1);

    AfterTc_ZMP_trajectory(i) = StepZone.x();
    AfterTc_ZMP_trajectory(i + PreviewSize) = StepZone.y();

    count_Dstep += 1;

    if(m_tk + (m_C + (double)i + 1) * m_delta > NextStepTiming && j_f + 1 < m_timestamp.size())
    {
      if(N_Steps + j_f + 1 <= N_Steps_Desired || N_Steps_Desired < 0)
      {
        j_f = std::min(j_f + 1, (int)m_timestamp.size() - 1);
        j_fm1 = j_f - 1;
        count_Dstep = 1;
      }
      else
      {
        count_Dstep = DD + 1;
      }
    }
  }

  AfterTc_ZMP_velocity.resize(2 * (PreviewSize - 1));

  for(int k = 0; k < PreviewSize - 1; k++)
  {
    AfterTc_ZMP_velocity(k) = (AfterTc_ZMP_trajectory(k + 1) - AfterTc_ZMP_trajectory(k)) / m_delta;
    AfterTc_ZMP_velocity(k + PreviewSize - 1) =
        (AfterTc_ZMP_trajectory(k + 1 + PreviewSize) - AfterTc_ZMP_trajectory(k + PreviewSize)) / m_delta;
  }
}

void ISMPC_Solver::Stability_Constraints()
{
  std::chrono::high_resolution_clock::time_point t_clock = std::chrono::high_resolution_clock::now();

  Eigen::Vector3d c_k;
  c_k.setZero();

  Aeq = Eigen::MatrixXd::Zero(2, 2 * (m_C + j_Max_C));
  beq.resize(2, 1);
  beq.setZero();
  for(int j = 0; j < m_C; j++)
  {
    Aeq(0, 2 * j) = exp(-j * m_eta * m_delta);
    Aeq(1, 2 * j + 1) = exp(-j * m_eta * m_delta);
  }
  if(m_Tail == "Periodic")
  {
    beq(0) = m_eta * ((1 - exp(-m_eta * m_delta * m_C)) / (1 - exp(-m_eta * m_delta))) * (P_u_k.x() - P_z_k.x());
    beq(1) = m_eta * ((1 - exp(-m_eta * m_delta * m_C)) / (1 - exp(-m_eta * m_delta))) * (P_u_k.y() - P_z_k.y());
  }
  else if(m_Tail == "Truncated")
  {
    beq(0) = (m_eta / (1 - exp(-m_eta * m_delta))) * (P_u_k.x() - P_z_k.x());
    beq(1) = (m_eta / (1 - exp(-m_eta * m_delta))) * (P_u_k.y() - P_z_k.y());
  }
  else if(m_Tail == "Anticipative")
  {
    AntTailTrajectory();
    int PreviewSize = (int)std::round(AfterTc_ZMP_trajectory.size() / 2);
    Ant_Tail_X = 0;
    Ant_Tail_Y = 0;
    for(int k = 0; k < PreviewSize; k++)
    {
      if(k < PreviewSize - 1)
      {
        Ant_Tail_X += exp(-(k + m_C) * m_eta * m_delta) * AfterTc_ZMP_velocity(k);
        Ant_Tail_Y += exp(-(k + m_C) * m_eta * m_delta) * AfterTc_ZMP_velocity(k + PreviewSize - 1);
      }
      c_k += m_eta * exp(-m_eta * m_delta * (k + m_C))
             * Eigen::Vector3d{AfterTc_ZMP_trajectory(k), AfterTc_ZMP_trajectory(k + PreviewSize), 0} * m_delta;
    }
    beq(0) = m_eta * ((1 - exp(-m_eta * m_delta * m_P)) / (1 - exp(-m_eta * m_delta))) * (P_u_k.x() - P_z_k.x())
             - Ant_Tail_X;
    beq(1) = m_eta * ((1 - exp(-m_eta * m_delta * m_P)) / (1 - exp(-m_eta * m_delta))) * (P_u_k.y() - P_z_k.y())
             - Ant_Tail_Y;
  }
  else
  {
    Aeq.setZero();
    beq(0) = 0;
    beq(1) = 0;
  }

  P_u_k_max += c_k;
  P_u_k_min += c_k;

  InStabilityRange = P_u_k_min.x() <= P_u_k.x() && P_u_k_min.y() <= P_u_k.y() && P_u_k_max.x() >= P_u_k.x()
                     && P_u_k_max.y() >= P_u_k.y();

  std::chrono::duration<double, std::milli> time_span = std::chrono::high_resolution_clock::now() - t_clock;
  double ProcessTime = time_span.count();
  // mc_rtc::log::success("Stability Constraints computed in : " + std::to_string(ProcessTime) + " ms");
}

void ISMPC_Solver::IntegrateZMPVel()
{

  Eigen::Vector3d v_z_i = Eigen::Vector3d{m_ZMP_vel[0], m_ZMP_vel[m_C], 0};

  int N = (int)(m_delta / m_delta_control);

  m_X_MPC.clear();
  m_Y_MPC.clear();
  m_X_MPC.push_back(Integration_Mat * Eigen::Vector3d{P_c_k.x() + Offset.x(), V_c_k.x(), P_z_k.x() + Offset.x()}
                    + Integration_Vec * v_z_i.x());
  m_Y_MPC.push_back(Integration_Mat * Eigen::Vector3d{P_c_k.y() + Offset.y(), V_c_k.y(), P_z_k.y() + Offset.y()}
                    + Integration_Vec * v_z_i.y());

  for(int k = 1; k < N; k++)
  {

    m_X_MPC.push_back(Integration_Mat * m_X_MPC[m_X_MPC.size() - 1] + Integration_Vec * v_z_i.x());

    m_Y_MPC.push_back(Integration_Mat * m_Y_MPC[m_Y_MPC.size() - 1] + Integration_Vec * v_z_i.y());
  }
  for(int i = 1; i < m_C; i++)
  {

    v_z_i = Eigen::Vector3d{m_ZMP_vel[i], m_ZMP_vel[m_C + i], 0};

    for(int k = 0; k < N; k++)
    {

      m_X_MPC.push_back(Integration_Mat * m_X_MPC[m_X_MPC.size() - 1] + Integration_Vec * v_z_i.x());

      m_Y_MPC.push_back(Integration_Mat * m_Y_MPC[m_Y_MPC.size() - 1] + Integration_Vec * v_z_i.y());
    }
  }
}

void ISMPC_Solver::GetWalkingParameters(double PrevStepTime, double t_k, double Tds)
{

  m_tk = t_k;
  m_Tds = Tds;
  QPsuccess = false;
  InStabilityRange = false;

  int t(0);
  double ts = 0;
  j_f = 0;
  kfoot = 1;
  j_Max_C = 0;
  if(m_timestamp.size() != 0)
  {
    while(ts + m_Tds < m_Tc)
    {
      t += 1;
      ts = m_timestamp[t - 1];
      if(t > m_timestamp.size())
      {
        break;
      }
      if(m_tk > m_timestamp[t])
      {
        kfoot += 1;
      }
    }
    j_Max_C = t + 1;
  }
  j_fm1 = j_f - 1;

  // mc_rtc::log::info("t_k : {}; Tc : {} ; Tds {} ; j_f_max : {}",m_tk, m_Tc ,m_Tds ,j_Max_C);

  m_D = (int)std::round(m_Tds / m_delta);
  count_Dstep = (int)(std::round(std::min(m_tk / m_delta, (double)m_D))) + 1;

  // m_D = std::max(0,(int) std::round((m_Tds + PrevStepTime - m_tk)/m_delta));
  // count_Dstep = 1;

  Eigen::MatrixXd M_zmp = Eigen::MatrixXd::Zero(2 * (m_C + j_Max_C), 2 * (m_C + j_Max_C));
  Eigen::MatrixXd M_steps = Eigen::MatrixXd::Zero(2 * (m_C + j_Max_C), 2 * (m_C + j_Max_C));
  Eigen::MatrixXd m = Eigen::MatrixXd::Zero(2 * m_C, 2 * (m_C + j_Max_C));
  M_zmp.block(0, 0, 2 * m_C, 2 * m_C) = Eigen::MatrixXd::Identity(2 * m_C, 2 * m_C);
  M_steps.block(2 * m_C, 2 * m_C, 2 * j_Max_C, 2 * j_Max_C) = Eigen::MatrixXd::Identity(2 * j_Max_C, 2 * j_Max_C);
  Eigen::VectorXd b_zmp = Eigen::VectorXd::Zero(2 * (m_C + j_Max_C));
  Eigen::VectorXd b_steps = Eigen::VectorXd::Zero(2 * (m_C + j_Max_C));
  for(int i = 0; i < j_Max_C; i++)
  {
    b_steps(2 * m_C + 2 * i) = m_Xf(i + 1);
    b_steps(2 * m_C + 2 * i + 1) = m_Yf(i + 1);
  }

  ZMP_Constraints();
  FootSteps_Constraints();

  Stability_Constraints();

  // if(!InStabilityRange){
  //   mc_rtc::log::warning("Pu Outside Stability Region");
  //   mc_rtc::log::info("X : " + std::to_string(P_u_k_min.x()) + " <= " + std::to_string(P_u_k.x()) + " <= " +
  //   std::to_string(P_u_k_max.x())); mc_rtc::log::info("Y : " + std::to_string(P_u_k_min.y()) + " <= " +
  //   std::to_string(P_u_k.y()) + " <= " + std::to_string(P_u_k_max.y())); double err = (P_u_k -
  //   (P_u_k_max+P_u_k_min)/2 - Eigen::Vector3d{0,0,P_u_k.z()}).norm(); mc_rtc::log::info(err); m_Tail = "None";
  //   Stability_Constraints();
  // }
  // else{
  //   mc_rtc::log::success("Pu Inside Stability Region");
  // }
  // mc_rtc::log::info((P_u_k_max - P_u_k_min).x());
  // mc_rtc::log::info(m_dx*(1-exp(-m_eta*m_Tc)));
  std::chrono::high_resolution_clock::time_point t_clock = std::chrono::high_resolution_clock::now();

  // for (int k = 0 ; k < m_C ; k++){

  //   if (bineq(k) + bineq(k+2*(m_C+j_Max_C)) < 0 ){
  //     mc_rtc::log::error("Unfeasible constraints X at k : {} ",k);
  //   }
  //   if (bineq(k+m_C) + bineq(k+2*(m_C+j_Max_C) + m_C) < 0 ){
  //     mc_rtc::log::error("Unfeasible constraints Y at k : {}",k);
  //   }

  // }

  m_Q = Eigen::MatrixXd::Identity(M_zmp.rows(), M_zmp.cols()) * 1e-12 + (M_zmp.transpose() * M_zmp)
        + m_Beta * (M_steps.transpose() * M_steps);
  m_p = (-M_zmp.transpose() * b_zmp) + m_Beta * (-M_steps.transpose() * b_steps);

  Aineq = Eigen::MatrixXd::Zero(Aineq_steps.rows() + Aineq_zmp.rows(), 2 * (m_C + j_Max_C));
  bineq = Eigen::VectorXd::Zero(bineq_steps.rows() + bineq_zmp.rows());
  Aineq << Aineq_zmp, Aineq_steps;
  bineq << bineq_zmp, bineq_steps;

  QP_Output = solveQP();
  // std::cout << "QP out " << QP_Output << std::endl;

  Eigen::VectorXd zmp_vel_ = QP_Output.segment(0, 2 * m_C);
  if(!(((zmp_vel_ - zmp_vel_).array() == (zmp_vel_ - zmp_vel_).array()).all()))
  {

    mc_rtc::log::warning("[ISMPC] nan");
    QPsuccess = false;
  }

  std::chrono::duration<double, std::milli> time_span = std::chrono::high_resolution_clock::now() - t_clock;
  double ProcessTime = time_span.count();
  // mc_rtc::log::success("ZMPvel QP computed in : " + std::to_string(ProcessTime));

  if(!QPsuccess && m_Tail != "None")
  {

    // mc_rtc::log::error("Equality contraints error : ");
    // mc_rtc::log::error(Aeq*QP_Output - beq);
    // mc_rtc::log::error("Eq Cstr Norm error : " + std::to_string((Aeq*QP_Output - beq).norm()));
    Eigen::VectorXd b = Aineq * QP_Output;
    // mc_rtc::log::error("Ineq Cstr Norm error : " + std::to_string((b - bineq).norm()));

    // bool Ignorable = false;
    // for (int k = 0 ; k < (int) (m_delta/m_delta_control) ; k++){ Ignorable = (b - bineq)(k) <= 0 && (b - bineq)(k +
    // m_C) <= 0  &&
    //                                                                          (b - bineq)( 2*m_C ) <= 0 && (b -
    //                                                                          bineq)( 2*m_C+j_Max_C) <= 0 && (b -
    //                                                                          bineq)(k + 2*(m_C+j_Max_C)) <= 0 && (b -
    //                                                                          bineq)(k + m_C + 2*(m_C+j_Max_C)) <= 0
    //                                                                          && (b - bineq)(2*(m_C + j_Max_C) +
    //                                                                          2*m_C) <= 0 && (b - bineq)(2*(m_C +
    //                                                                          j_Max_C) + 2*m_C + j_Max_C) <= 0; }
    // if(Ignorable){ Ignorable   =  (Aeq*QP_Output - beq).norm() < 1e-3 ;}
    // Ignorable = false;
    // if(!AutoFootstepPlacement){
    //     for (int k = 0 ; k < m_C ; k++){
    //       if (b(k) - bineq(k) > 0){
    //         Ignorable = false;
    //         mc_rtc::log::error("Upper Geom Constraints broken on x at k = " + std::to_string(k) + " :" +
    //         std::to_string(b(k)) + " <= " + std::to_string(bineq(k))); mc_rtc::log::error(b(k) - bineq(k));
    //       }
    //       if(b(k + 2*m_C) - bineq(k + 2*m_C) > 0 ){
    //         Ignorable = false;
    //         mc_rtc::log::error("Lower Geom Constraints broken on x at k = " + std::to_string(k) + " :" +
    //         std::to_string(b(k+2*m_C)) + " <= " + std::to_string(bineq(k + 2*m_C))); mc_rtc::log::error(b(k + 2*m_C)
    //         - bineq(k + 2*m_C ));
    //       }
    //       if (b(k+m_C) - bineq(k+m_C) > 0){
    //         Ignorable = false;
    //         mc_rtc::log::error("Upper Geom Constraints broken on y at k = " + std::to_string(k) + " :" +
    //         std::to_string(b(k+m_C)) + " <= " + std::to_string(bineq(k+m_C))); mc_rtc::log::error(b(k + m_C) -
    //         bineq(k + m_C ));
    //       }
    //       if( b(k + 3*m_C) - bineq(k + 3*m_C) > 0 ){
    //         Ignorable = false;
    //         mc_rtc::log::error("Lower Geom Constraints broken on y at k = " + std::to_string(k) + " :" +
    //         std::to_string(b(k+3*m_C)) + " <= " +  std::to_string(bineq(k + 3*m_C))); mc_rtc::log::error(b(k + 3*m_C)
    //         - bineq(k + 3*m_C ));
    //       }
    //     }
    // }
    // else{
    //   for (int k = 0 ; k < 1 ; k++){
    //     if (b(k) - bineq(k) > 0){
    //       Ignorable = false;
    //       mc_rtc::log::error("Upper Geom Constraints broken on x at k = " + std::to_string(k) + " :" +
    //       std::to_string(b(k)) + " <= " + std::to_string(bineq(k))); mc_rtc::log::error(b(k) - bineq(k));
    //     }
    //     if(b(k + 2*m_C) - bineq(k + 2*m_C) > 0 ){
    //       Ignorable = false;
    //       mc_rtc::log::error("Lower Geom Constraints broken on x at k = " + std::to_string(k) + " :" +
    //       std::to_string(-b(k+2*(m_C+j_Max_C))) + " >= " + std::to_string(-bineq(k + 2*(m_C+j_Max_C))));
    //       mc_rtc::log::error(b(k + 2*m_C) - bineq(k + 2*m_C ));
    //     }
    //     if (b(k+m_C) - bineq(k+m_C) > 0){
    //       Ignorable = false;
    //       mc_rtc::log::error("Upper Geom Constraints broken on y at k = " + std::to_string(k) + " :" +
    //       std::to_string(b(k+m_C)) + " <= " + std::to_string(bineq(k+m_C))); mc_rtc::log::error(b(k + m_C) - bineq(k
    //       + m_C ));
    //     }
    //     if( b(k + 3*m_C) - bineq(k + 3*m_C) > 0 ){
    //       Ignorable = false;
    //       mc_rtc::log::error("Lower Geom Constraints broken on y at k = " + std::to_string(k) + " :" +
    //       std::to_string(-b(k+2*(m_C+j_Max_C) + m_C)) + " >= " +  std::to_string(-bineq(k + 2*(m_C+j_Max_C) + m_C)));
    //       mc_rtc::log::error(b(k + 3*m_C) - bineq(k + 3*m_C ));
    //     }
    //   }
    // }

    QPsuccess = false;

    mc_rtc::log::warning("[ISMPC] Ignoring Stability cstr");
    m_Tail = "None";
    Stability_Constraints();
    QP_Output = solveQP();
  }

  if(!QPsuccess)
  {
    // mc_rtc::log::error_and_throw<std::runtime_error>("QP Failed");
    mc_rtc::log::warning("[ISMPC] Ignoring QP");
  }

  else
  {

    m_Xf_Corr.resize(j_Max_C + 1, 1);
    m_Xf_Corr.setZero();
    m_Yf_Corr.resize(j_Max_C + 1, 1);
    m_Yf_Corr.setZero();

    m_ZMP_vel.resize(2 * m_C, 1);
    for(int k = 0; k < m_C; k++)
    {
      m_ZMP_vel(k) = QP_Output(2 * k);
      m_ZMP_vel(k + m_C) = QP_Output(2 * k + 1);
    }

    m_Yf_Corr(0) = m_Yf(0);
    m_Xf_Corr(0) = m_Xf(0);
    for(int k = 0; k < j_Max_C; k++)
    {
      m_Xf_Corr(k + 1) = QP_Output(2 * m_C + 2 * k);
      m_Yf_Corr(k + 1) = QP_Output(2 * m_C + 2 * k + 1);
    }

    // mc_rtc::log::info("zmp vel x {}",m_ZMP_vel.segment(0,m_C));
    // mc_rtc::log::info("zmp vel y {}",m_ZMP_vel.segment(m_C,m_C));
    if(m_Tail == "None")
    {
      Eigen::Vector3d P_u_k_2 = P_z_k;
      if(m_Tail_save == "Periodic")
        for(int k = 0; k < m_C; k++)
        {
          P_u_k_2 += ((1 - exp(-m_eta * m_delta)) / (m_eta * (1 - exp(-m_eta * m_Tc)))) * exp(-k * m_eta * m_delta)
                     * Eigen::Vector3d{m_ZMP_vel[k], m_ZMP_vel[k + m_C], 0};
        }
      else if(m_Tail_save == "Truncated")
      {
        for(int k = 0; k < m_C; k++)
        {
          P_u_k_2 += ((1 - exp(-m_eta * m_delta)) / (m_eta)) * exp(-k * m_eta * m_delta)
                     * Eigen::Vector3d{m_ZMP_vel[k], m_ZMP_vel[k + m_C], 0};
        }
      }
      else
      {
        for(int k = 0; k < m_C; k++)
        {
          P_u_k_2 += ((1 - exp(-m_eta * m_delta)) / (m_eta * (1 - exp(-m_eta * m_Tp)))) * exp(-k * m_eta * m_delta)
                     * Eigen::Vector3d{m_ZMP_vel[k], m_ZMP_vel[k + m_C], 0};
        }
        P_u_k_2 += ((1 - exp(-m_eta * m_delta)) / (m_eta * (1 - exp(-m_eta * m_Tp))))
                   * Eigen::Vector3d{Ant_Tail_X, Ant_Tail_Y, 0};
      }
      V_c_k = m_eta * (P_u_k_2 - P_c_k);
      // P_c_k = P_u_k_2 - V_c_k/m_eta;
    }

    // std::vector<double> ZMPvelx;
    // std::vector<double> ZMPvely;
    // for (int k = 0; k<m_ZMP_vel.size()/2 ; k++){
    //   ZMPvelx.push_back(m_ZMP_vel[k]);
    //   ZMPvely.push_back(m_ZMP_vel[k + m_C]);
    // }
    // double V_z_Max_X = std::abs(*std::max_element(ZMPvelx.begin(), ZMPvelx.end()));
    // double V_z_Max_Y = std::abs(*std::max_element(ZMPvely.begin(), ZMPvely.end()));

    // mc_rtc::log::info("Feasibility");
    // mc_rtc::log::info(std::to_string(V_z_Max_X) + " ; " + std::to_string(V_z_Max_Y));
    // mc_rtc::log::info(m_Tc+((1/m_eta)*log(2*V_z_Max_X/(m_eta*m_dx))));
    // mc_rtc::log::info(m_Tc+((1/m_eta)*log(2*V_z_Max_Y/(m_eta*m_dy))));

    IntegrateZMPVel();
  }
}

Eigen::VectorXd ISMPC_Solver::solveQP()
{

  Eigen::QuadProgDense QP;

  int Nvar = m_Q.rows();
  int NIneqConstr = Aineq.rows();
  int NEqConstr = Aeq.rows();
  QP.tolerance(5e-3);
  QP.problem(Nvar, NEqConstr, NIneqConstr);
  QPsuccess = QP.solve(m_Q, m_p, Aeq, beq, Aineq, bineq);

  return QP.result();
}

void SupportPolygon::jarvis_march()
{
  // std::sort(_Corners.begin(),_Corners.end(),vec3d_x_comp());

  int index = std::min_element(_Corners.begin(), _Corners.end(), vec3d_x_comp()) - _Corners.begin();

  SupportPolygone_Corners.push_back(_Corners[index]);

  int l = index;
  int q = 0;
  while(true)
  {
    q = (l + 1) % _Corners.size();
    for(int i = 0; i < _Corners.size(); i++)
    {
      if(i == l)
      {
        continue;
      }
      Eigen::Vector3d v_q{_Corners[q].x(), _Corners[q].y(), 0};
      Eigen::Vector3d v_l{_Corners[l].x(), _Corners[l].y(), 0};
      Eigen::Vector3d v_i{_Corners[i].x(), _Corners[i].y(), 0};
      double d = ((v_q - v_l).cross(v_i - v_l)).z();
      if(d > 0 || (d == 0 && (v_i - v_l).norm() > (v_q - v_l).norm()))
      {
        q = i;
      }
    }
    l = q;
    if(l == index)
    {
      break;
    }
    SupportPolygone_Corners.push_back(_Corners[q]);
  }
}
