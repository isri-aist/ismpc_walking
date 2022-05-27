#include "ISMPC_Solver.h"

ISMPC_Solver::ISMPC_Solver() {}

ISMPC_Solver::ISMPC_Solver(double delta_controller, double delta, double Tp, double Tc)
{

  m_Tc = Tc;
  m_Tp = Tp;
  m_delta = delta;
  m_delta_control = delta_controller;

  m_eta = sqrt(mc_rtc::constants::GRAVITY / CoM_height);
  count_Dstep = 0;
  m_C = (int)std::round((m_Tc) / m_delta);
  m_P = (int)std::round((m_Tp) / m_delta);
  QPsuccess = false;

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

  w_k.setZero();
  
}

void ISMPC_Solver::configure(const ControllerConfiguration & config)
{
  m_dx_f = config.MPC_Footsteps_Constraint_size.x();
  m_dy_f = config.MPC_Footsteps_Constraint_size.y();
  m_dx = config.MPC_ZMP_Constraint_size.x();
  m_dy = config.MPC_ZMP_Constraint_size.y();
  m_dx_sg_s = config.MPC_ZMP_Constraint_size_sg_supp.x();
  m_dy_sg_s = config.MPC_ZMP_Constraint_size_sg_supp.y();
  m_Beta = config.Beta;
  m_Beta_stab = config.Beta_stab;
  m_Beta_traj = config.Beta_traj;
  Slide_ZMP_region = config.sliding_zmp_cstr_region;
  zmp_cstr_next_stp_ratio = config.MPC_ZMP_next_stp_cstr_ratio;
  rect_pose_offset = config.MPC_ZMP_cstr_square_offset;
  Allow_None = config.MPC_allow_None;
  m_Tc = config.Tc;
  m_Tp = config.Tp;
  m_delta = config.delta;
  m_delta_control = config.Controller_timestep;
  m_C = (int)std::round((m_Tc) / m_delta);
  m_P = (int)std::round((m_Tp) / m_delta);
  CoM_height = config.Stab_config.comHeight;
  m_eta = sqrt(mc_rtc::constants::GRAVITY / CoM_height);
  Use_Stability_Task = config.use_stability_task;
  rect_pose_offset_sg_supp = config.MPC_ZMP_cstr_square_offset_sg_supp;

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

  mc_rtc::log::info("[ISMPC] Configuration :");
  mc_rtc::log::info("Beta {}", m_Beta);
  mc_rtc::log::info("ZMP cstr\n{}", Eigen::Vector2d{m_dx, m_dy});
  mc_rtc::log::info("Footsteps cstr\n{}", Eigen::Vector2d{m_dx_f, m_dy_f});
  mc_rtc::log::info("CoM h {}", CoM_height);
  mc_rtc::log::info("Tp {} Tc {}", m_Tp, m_Tc);
  mc_rtc::log::info("MPC_delta {}", m_delta);
  mc_rtc::log::info("Controller delta {}", m_delta_control);
  mc_rtc::log::info("Use of whole polygon support {}", !Slide_ZMP_region);
  mc_rtc::log::info("Stability Task {}", Use_Stability_Task);
}

void ISMPC_Solver::InitStepGen(std::vector<sva::PTransformd> & steps , sva::PTransformd support_foot)
{
  m_Xf = Eigen::VectorXd::Zero(steps.size() + 1);
  m_Yf = Eigen::VectorXd::Zero(steps.size() + 1);
  m_Theta_f = Eigen::VectorXd::Zero(steps.size() + 1);

  Eigen::Vector3d support_pose(support_foot.translation());
  double support_ori(-mc_rbdyn::rpyFromMat(support_foot.rotation()).z());
  m_Xf(0) = support_pose.x();
  m_Yf(0) = support_pose.y();
  m_Theta_f(0) = support_ori; 

  for (int k = 0 ; k < steps.size() ; k++)
  {
    Eigen::Vector3d step_k_pose(steps[k].translation());
    double step_k_ori(-mc_rbdyn::rpyFromMat(steps[k].rotation()).z());
    m_Xf(k+1) = step_k_pose.x();
    m_Yf(k+1) = step_k_pose.y();
    m_Theta_f(k+1) = step_k_ori;
  }

  std::cout << "input xf " << m_Xf << std::endl;



  R_support_0 = sva::RotZ(-m_Theta_f(0));
  R_0_support = R_support_0.transpose();
}

void ISMPC_Solver::SetWalkingParameters(const Eigen::Vector3d & Pck,
                                        const Eigen::Vector3d & Vck,
                                        const Eigen::Vector3d & Pzk,
                                        const Eigen::Vector3d & Pfm1,
                                        const std::vector<double> & timesstp,
                                        std::string Tail,
                                        int Steps_Desired,
                                        int Steps)
{
  P_c_k = Pck; // - Offset;
  V_c_k = Vck;
  P_z_k = Pzk; // - Offset;
  m_Tail = Tail;
  m_Tail_save = Tail;
  m_eta = sqrt(g / P_c_k.z());
  P_u_k = P_c_k + (V_c_k / m_eta);
  m_Pfm1 = Pfm1; // - Offset;
  m_timestamp = timesstp;
  N_Steps = Steps;
  N_Steps_Desired = Steps_Desired;

  if(m_Xf.size() == 1)
  {
    mc_rtc::log::warning("[ISMPC] No Footsteps target provided");
    m_Xf = Eigen::VectorXd(Eigen::Vector2d{m_Xf(0), m_Pfm1.x()});
    m_Yf = Eigen::VectorXd(Eigen::Vector2d{m_Yf(0), m_Pfm1.y()});
    m_Theta_f = Eigen::VectorXd(Eigen::Vector2d{m_Theta_f(0), m_Pfm1.z()});
  }
}

void ISMPC_Solver::ZMP_Constraints()
{

  std::vector<Eigen::VectorXd> b_zmp_ineq;
  std::vector<Eigen::MatrixXd> Normal_zmp_Vec;
  double sgn = -1;
  if((sva::RotZ(-m_Theta_f(0)).transpose().block(0, 0, 2, 2) * Eigen::Vector2d{m_Xf(1) - m_Xf(0), m_Yf(1) - m_Yf(0)}).y()
     > 0) // Right Support
  {
    sgn = 1;
  }
  const Eigen::Vector3d & rect_offset_support =
      sva::RotZ(-m_Theta_f(0)) * Eigen::Vector3d{rect_pose_offset.x(), sgn * rect_pose_offset.y(), 0};

  const Eigen::Vector3d & rect_offset_support_sg =
      sva::RotZ(-m_Theta_f(0)) * Eigen::Vector3d{rect_pose_offset_sg_supp.x(), sgn * rect_pose_offset_sg_supp.y(), 0};

  const Eigen::Vector3d & rect_offset_swing =
      sva::RotZ(-m_Pfm1.z()) * Eigen::Vector3d{rect_pose_offset.x(), -sgn * rect_pose_offset.y(), 0};

  Rectangle Sliding_rect = Rectangle(Eigen::Vector3d{0, 0, m_Theta_f[j_f]}, Eigen::Vector3d{m_dx, m_dy, 0});
  Rectangle Sliding_rect_sg_supp =
      Rectangle(Eigen::Vector3d{0, 0, m_Theta_f[j_f]}, Eigen::Vector3d{m_dx_sg_s, m_dy_sg_s, 0});

  SupportPolygon Poly_Rect(Sliding_rect);
  SupportPolygon Poly_Rect_sg_supp(Sliding_rect_sg_supp);

  Rectangle Rect_jm1 = Rectangle(m_Pfm1 + rect_offset_swing, Eigen::Vector3d{m_dx, m_dy, 0});
  Rectangle Rect_j =
      Rectangle(Eigen::Vector3d{m_Xf(0), m_Yf(0), m_Theta_f(0)} + rect_offset_support, Eigen::Vector3d{m_dx, m_dy, 0});
  SupportPolygon SuppPoly(Rect_jm1, Rect_j);
  SupportPolygon S_Support_Poly(Rectangle(Eigen::Vector3d{m_Xf(0), m_Yf(0), m_Theta_f(0)} + rect_offset_support_sg,
                                          Eigen::Vector3d{m_dx_sg_s, m_dy_sg_s, 0}));

  ZMP_ref_traj.clear();
  ZMP_max_ref_traj.clear();
  ZMP_min_ref_traj.clear();
  All_poly.clear();

  Eigen::MatrixXd Delta; // Matrix to derive the ZMP position to ZMP velocity
  Delta = Eigen::MatrixXd::Identity(N_variable, N_variable);

  P_u_k_max = m_eta * m_delta * R_0_support * P_z_k;
  P_u_k_min = m_eta * m_delta * R_0_support * P_z_k;
  double NextStepTiming(0);
  if(m_timestamp.size() != 0)
  {
    NextStepTiming = m_timestamp[j_f];
  }
  double PrevStepTime = 0;

  for(int i = 0; i < m_C; i++)
  {

    if(m_tk + ((double)i) * m_delta >= NextStepTiming && j_f + 1 < m_timestamp.size())
    {
      m_D = (int)std::round(m_Tds / m_delta) - 1 - Tds_offset;

      j_f = std::min(j_f + 1, (int)m_Xf.size() - 1);
      j_fm1 = j_f - 1;
      count_Dstep = 0;
      sgn *= -1;

      const Eigen::Vector3d & rect_offset =
          sva::RotZ(-m_Theta_f(0)) * Eigen::Vector3d{rect_pose_offset.x(), sgn * rect_pose_offset.y(), 0};

      const Eigen::Vector3d & rect_offset_sg =
          sva::RotZ(-m_Theta_f(0))
          * Eigen::Vector3d{rect_pose_offset_sg_supp.x(), sgn * rect_pose_offset_sg_supp.y(), 0};

      Sliding_rect = Rectangle(Eigen::Vector3d{0, 0, m_Theta_f[j_f]} + rect_offset,
                               Eigen::Vector3d{m_dx, m_dy, 0} * zmp_cstr_next_stp_ratio);
      Sliding_rect_sg_supp =
          Rectangle(Eigen::Vector3d{0, 0, m_Theta_f[j_f]} + rect_offset_sg, Eigen::Vector3d{m_dx_sg_s, m_dy_sg_s, 0});
      Poly_Rect = SupportPolygon(Sliding_rect);
      Poly_Rect_sg_supp = SupportPolygon(Sliding_rect_sg_supp);

      NextStepTiming = m_timestamp[j_f];
      PrevStepTime = m_timestamp[j_fm1];
    }

    double DD = (double)m_D;
    for(int k = 0; k <= i; k++)
    {
      Delta(2 * i, 2 * k) = m_delta;
      Delta(2 * i + 1, 2 * k + 1) = m_delta;
    }

    double nn = std::max(0.0, std::min((double)count_Dstep, DD + 1.));

    // mc_rtc::log::info("i : {} ; j_f : {} ; DD : {} ; nn : {} ; C : {} ",i,j_f,DD,nn,m_C);
    // std::cout << "J_f "<< j_Max_C<< "j " <<  j_f << " i " << i<< std::endl;

    Eigen::Vector3d ZMP_Zone;

    if(j_f == 0 || !AutoFootstepPlacement)
    {
      Eigen::Vector3d P_f_j{m_Xf(j_f), m_Yf(j_f), 0};
      Eigen::Vector3d P_f_jm1 = m_Pfm1;
      // P_f_jm1 = P_z_k;
      if(j_f > 0)
      {
        P_f_jm1 = Eigen::Vector3d{m_Xf(j_fm1), m_Yf(j_fm1), 0.};
        if(!Slide_ZMP_region)
        {
          Rect_jm1 = Rectangle(P_f_jm1 + Eigen::Vector3d{0, 0, m_Theta_f(j_fm1)}, Eigen::Vector3d{m_dx, m_dy, 0});
          Rect_j = Rectangle(P_f_j + Eigen::Vector3d{0, 0, m_Theta_f(j_f)}, Eigen::Vector3d{m_dx, m_dy, 0});
          SuppPoly = SupportPolygon(Rect_jm1, Rect_j);
        }
        S_Support_Poly = Rectangle(Eigen::Vector3d{m_Xf(j_f), m_Yf(j_f), m_Theta_f(j_f)} + rect_offset_support_sg,
                                   Eigen::Vector3d{m_dx_sg_s, m_dy_sg_s, 0});
      }
      if(N_Steps == N_Steps_Desired)
      {
        P_f_j = (P_f_j + P_f_jm1) / 2;
        Rect_j = Rectangle(P_f_j, Eigen::Vector3d{m_dx, m_dy, 0});
        SuppPoly = SupportPolygon(Rect_jm1, Rect_j);
      }

      ZMP_Zone = (P_f_j * nn + P_f_jm1 * (DD + 1. - nn)) / (DD + 1.);
      ZMP_Zone.z() = m_Theta_f(j_f);
      Rectangle ZMP_rect = Rectangle(ZMP_Zone, Eigen::Vector3d{m_dx, m_dy, 0});

      // Eigen::Vector3d T_support_p0p1 = (sva::RotZ(-m_Theta_f(0)).transpose().block(0, 0, 2, 2) *
      // Eigen::Vector2d{m_Xf(1) - m_Xf(0), m_Yf(1) - m_Yf(0)});
      Eigen::Vector2d T_support_pm1p0 = (sva::RotZ(-m_Pfm1.z()).transpose().block(0, 0, 2, 2)
                                         * Eigen::Vector2d{m_Xf(0) - m_Pfm1.x(), m_Yf(0) - m_Pfm1.y()});
      double l = std::abs(T_support_pm1p0.y());
      Eigen::Vector3d ZMP_mid = 0.5 * (P_f_j + P_f_jm1);
      ZMP_mid.z() = -std::atan2(T_support_pm1p0.x(), T_support_pm1p0.y());
      Rectangle ZMP_rect_mid = Rectangle(ZMP_mid, Eigen::Vector3d{m_dx, l, 0});

      // ZMP_rect = ZMP_rect_mid;

      ZMP_ref_traj.push_back(ZMP_Zone.x() - P_z_k.x());
      ZMP_ref_traj.push_back(ZMP_Zone.y() - P_z_k.y());

      SupportPolygon Polygon(ZMP_rect);
      if(Slide_ZMP_region || nn == DD + 1)
      {
        if(nn == DD + 1)
        {
          Polygon = S_Support_Poly;
          ZMP_rect = S_Support_Poly.get_Rectangle(0);
        }

        ZMP_max_ref_traj.push_back(R_0_support * ZMP_rect.get_center() + Eigen::Vector3d{m_dx / 2, m_dy / 2, 0});
        ZMP_min_ref_traj.push_back(R_0_support * ZMP_rect.get_center() - Eigen::Vector3d{m_dx / 2, m_dy / 2, 0});
      }
      else
      {
        Polygon = SuppPoly;

        ZMP_max_ref_traj.push_back(R_0_support * ZMP_rect_mid.get_center() + Eigen::Vector3d{m_dx / 2, m_dy / 2, 0});
        ZMP_min_ref_traj.push_back(R_0_support * ZMP_rect_mid.get_center() - Eigen::Vector3d{m_dx / 2, m_dy / 2, 0});
      }

      if(i == 0)
      {
        SuppPolyCorners = Polygon.Get_Polygone_Corners();
        // std::cout << ZMP_Zone << std::endl;
      }

      Normal_zmp_Vec.push_back(Polygon.SupportPolygone_Normals.transpose());
      b_zmp_ineq.push_back(Polygon.Offset
                           - Polygon.SupportPolygone_Normals.transpose() * Eigen::Vector2d{P_z_k.x(), P_z_k.y()});

      if(!AutoFootstepPlacement && Slide_ZMP_region) All_poly.push_back(Polygon.Get_Polygone_Corners());
    }

    else if(j_f == 1)
    {

      Eigen::Vector2d T_support_pjpjm1 = (sva::RotZ(-m_Pfm1.z()).transpose().block(0, 0, 2, 2)
                                          * Eigen::Vector2d{m_Xf(j_f) - m_Xf(j_fm1), m_Yf(j_f) - m_Yf(j_fm1)});
      double l = T_support_pjpjm1.y();

      Eigen::Vector3d P_f_jm1{m_Xf(j_fm1), m_Yf(j_fm1), 0};
      Eigen::Vector3d P_f_j_min = Eigen::Vector3d{m_Xf(j_fm1), m_Yf(j_fm1), 0}
                                  + sva::RotZ(-m_Theta_f(j_f)) * Eigen::Vector3d{0., l, 0.}
                                  - (sva::RotZ(-m_Theta_f(j_f)) * Eigen::Vector3d{m_dx_f / 2, m_dy_f / 2, 0});
      Eigen::Vector3d P_f_j_max = Eigen::Vector3d{m_Xf(j_fm1), m_Yf(j_fm1), 0}
                                  + sva::RotZ(-m_Theta_f(j_f)) * Eigen::Vector3d{0., l, 0.}
                                  + (sva::RotZ(-m_Theta_f(j_f)) * Eigen::Vector3d{m_dx_f / 2, m_dy_f / 2, 0});

      Eigen::Vector3d ZMP_Zone_min = (P_f_j_min * nn + P_f_jm1 * (DD + 1 - nn)) / (DD + 1);
      Eigen::Vector3d ZMP_Zone_max = (P_f_j_max * nn + P_f_jm1 * (DD + 1 - nn)) / (DD + 1);

      ZMP_Zone = P_f_jm1 * (DD + 1 - nn) / (DD + 1);
      Eigen::Vector2d Dstep_state = Eigen::Vector2d::Ones() * (nn / (DD + 1));

      if(N_Steps + j_f == N_Steps_Desired)
      {
        ZMP_Zone = P_f_jm1 * (DD + 1 - 0.5 * nn) / (DD + 1);
        Dstep_state *= 0.5;
      }
      ZMP_Zone.z() = 0;

      ZMP_max_ref_traj.push_back(R_0_support * ZMP_Zone_max + Eigen::Vector3d{m_dx / 2, m_dy / 2, 0});
      ZMP_min_ref_traj.push_back(R_0_support * ZMP_Zone_min - Eigen::Vector3d{m_dx / 2, m_dy / 2, 0});

      SupportPolygon & Polygon(Poly_Rect);
      if(nn == DD + 1)
      {
        Polygon = Poly_Rect_sg_supp;
      }

      Eigen::VectorXd bcstr =
          Polygon.Offset 
          - Polygon.SupportPolygone_Normals.transpose() * Eigen::Vector2d{P_z_k.x(), P_z_k.y()}
          + Polygon.SupportPolygone_Normals.transpose() * Eigen::Vector2d{ZMP_Zone.x(), ZMP_Zone.y()};

      Normal_zmp_Vec.push_back(Polygon.SupportPolygone_Normals.transpose());
      b_zmp_ineq.push_back(bcstr);

      Delta(2 * i, 2 * m_C + 2 * (j_f - 1)) = -Dstep_state.x();
      Delta(2 * i + 1, 2 * m_C + 2 * (j_f - 1) + 1) = -Dstep_state.y();

      if(!AutoFootstepPlacement && Slide_ZMP_region) All_poly.push_back(Polygon.Get_Polygone_Corners());
      if(i == 0)
      {
        SuppPolyCorners = Polygon.Get_Polygone_Corners();
      }
    }

    else
    {
      Eigen::Vector2d Dstep_state_jf = Eigen::Vector2d::Ones() * (nn / (DD + 1));
      Eigen::Vector2d Dstep_state_jfm1 = Eigen::Vector2d::Ones() * ((DD + 1 - nn) / (DD + 1));

      if(N_Steps + j_f == N_Steps_Desired)
      {
        Dstep_state_jf = 0.5 * Dstep_state_jf;
        Dstep_state_jfm1 = Eigen::Vector2d::Ones() * ((DD + 1 - nn) / (DD + 1));
      }

      SupportPolygon & Polygon(Poly_Rect);
      if(nn == DD + 1)
      {
        Polygon = Poly_Rect_sg_supp;
      }

      Normal_zmp_Vec.push_back(Polygon.SupportPolygone_Normals.transpose());
      b_zmp_ineq.push_back(Polygon.Offset
                           - Polygon.SupportPolygone_Normals.transpose() * Eigen::Vector2d{P_z_k.x(), P_z_k.y()});

      Delta(2 * i, 2 * m_C + 2 * (j_f - 1)) = -Dstep_state_jf.x();
      Delta(2 * i + 1, 2 * m_C + 2 * (j_f - 1) + 1) = -Dstep_state_jf.y();
      Delta(2 * i, 2 * m_C + 2 * (j_fm1 - 1)) = -Dstep_state_jfm1.x();
      Delta(2 * i + 1, 2 * m_C + 2 * (j_fm1 - 1) + 1) = -Dstep_state_jfm1.y();
      if(i == 0)
      {
        SuppPolyCorners = Polygon.Get_Polygone_Corners();
      }
    }

    count_Dstep += 1;
  }

  int N_zmp_cstr = 0;
  for(int k = 0; k < Normal_zmp_Vec.size(); k++)
  {
    N_zmp_cstr += (int)Normal_zmp_Vec[k].rows();
  }
  // mc_rtc::log::success("ZMP cstr computed, Ncstr = {}", N_zmp_cstr);

  Eigen::MatrixXd ZMP_Cstr;
  ZMP_Cstr.resize(N_zmp_cstr, N_variable);
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

  b_zmp_traj = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ZMP_ref_traj.data(), ZMP_ref_traj.size());
  M_zmp_traj = Eigen::MatrixXd::Zero(b_zmp_traj.rows(), N_variable);
  M_zmp_traj.block(0, 0, b_zmp_traj.rows(), b_zmp_traj.rows()) =
      Eigen::MatrixXd::Identity(b_zmp_traj.rows(), b_zmp_traj.rows())
      * Delta.block(0, 0, b_zmp_traj.rows(), b_zmp_traj.rows());
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

  Aineq_steps = Eigen::MatrixXd::Zero(N_Footsteps_cstr, N_variable);
  Aineq_steps.block(0, 2 * m_C, N_Footsteps_cstr, 2 * j_Max_C) = Foosteps_Cstr * Delta;
  bineq_steps = b_steps;
}

void ISMPC_Solver::AntTailTrajectory()
{
  int PreviewSize = m_P - m_C;
  AfterTc_ZMP_trajectory;
  AfterTc_ZMP_trajectory.resize(2 * PreviewSize, 1);
  AfterTc_ZMP_trajectory.setZero();
  double DD = (double)m_D;
  for(int i = 0; i < PreviewSize; i++)
  {

    double NextStepTiming(0);
    if(m_timestamp.size() != 0)
    {
      NextStepTiming = m_timestamp[j_f];
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

    ZMP_max_ref_traj.push_back(R_0_support * StepZone);
    ZMP_min_ref_traj.push_back(R_0_support * StepZone);

    count_Dstep += 1;
    if(j_f == m_timestamp.size() - 1 && nn > DD / 2)
    {
      count_Dstep = ((int)(DD / 2));
    }

    if(m_tk + (m_C + (double)i + 1) * m_delta > NextStepTiming && j_f + 1 < m_timestamp.size())
    {
      if(N_Steps + j_f + 1 <= N_Steps_Desired || N_Steps_Desired < 0)
      {
        j_f = std::min(j_f + 1, (int)m_Xf.size() - 1);
        j_fm1 = j_f - 1;
        count_Dstep = 1;
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

  A_stab = Eigen::MatrixXd::Zero(2, N_variable);
  b_stab = Eigen::VectorXd::Zero(2);
  for(int j = 0; j < m_C; j++)
  {
    A_stab(0, 2 * j) = exp(-j * m_eta * m_delta);
    A_stab(1, 2 * j + 1) = exp(-j * m_eta * m_delta);
  }
  if(m_Tail == "Periodic")
  {
    b_stab(0) = m_eta * ((1 - exp(-m_eta * m_delta * m_C)) / (1 - exp(-m_eta * m_delta))) * (P_u_k.x() - P_z_k.x());
    b_stab(1) = m_eta * ((1 - exp(-m_eta * m_delta * m_C)) / (1 - exp(-m_eta * m_delta))) * (P_u_k.y() - P_z_k.y());
  }
  else if(m_Tail == "Truncated")
  {
    b_stab(0) = (m_eta / (1 - exp(-m_eta * m_delta))) * (P_u_k.x() - P_z_k.x());
    b_stab(1) = (m_eta / (1 - exp(-m_eta * m_delta))) * (P_u_k.y() - P_z_k.y());
  }
  else
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
    }
    // b_stab(0) = m_eta * ((1 - exp(-m_eta * m_delta * m_P)) / (1 - exp(-m_eta * m_delta))) * (P_u_k.x() - P_z_k.x())
    //          - Ant_Tail_X;
    // b_stab(1) = m_eta * ((1 - exp(-m_eta * m_delta * m_P)) / (1 - exp(-m_eta * m_delta))) * (P_u_k.y() - P_z_k.y())
    //          - Ant_Tail_Y;
    b_stab(0) = (m_eta / (1 - exp(-m_eta * m_delta))) * (P_u_k.x() - P_z_k.x() + (w_k.x()/std::pow(m_eta,2))) - Ant_Tail_X;
    b_stab(1) = (m_eta / (1 - exp(-m_eta * m_delta))) * (P_u_k.y() - P_z_k.y() + (w_k.y()/std::pow(m_eta,2))) - Ant_Tail_Y;
  }

  std::chrono::duration<double, std::milli> time_span = std::chrono::high_resolution_clock::now() - t_clock;
  double ProcessTime = time_span.count();
  // mc_rtc::log::success("Stability Constraints computed in : " + std::to_string(ProcessTime) + " ms");
}

void ISMPC_Solver::Compute_Stability_Range()
{
  P_u_k_min = P_z_k;
  P_u_k_max = P_z_k;
  Eigen::Vector3d Pzm_km1 = P_z_k;
  Pzm_km1.z() = 0;
  Eigen::Vector3d Pzm_k;
  Eigen::Vector3d PzM_km1 = P_z_k;
  PzM_km1.z() = 0;
  Eigen::Vector3d PzM_k;
  Eigen::Vector3d Vzm_k;
  Eigen::Vector3d VzM_k;
  for(int k = 1; k < ZMP_max_ref_traj.size(); k++)
  {
    Pzm_k = ZMP_min_ref_traj[k];
    PzM_k = ZMP_max_ref_traj[k];
    VzM_k = (PzM_k - PzM_km1) / m_delta;
    Vzm_k = (Pzm_k - Pzm_km1) / m_delta;
    P_u_k_max += ((1 - exp(-m_eta * m_delta)) / (m_eta)) * exp(-k * m_eta * m_delta) * VzM_k;
    P_u_k_min += ((1 - exp(-m_eta * m_delta)) / (m_eta)) * exp(-k * m_eta * m_delta) * Vzm_k;

    PzM_km1 = PzM_k;
    Pzm_km1 = Pzm_k;
  }
}

void ISMPC_Solver::IntegrateZMPVel()
{

  Eigen::Vector3d v_z_i = Eigen::Vector3d{m_ZMP_vel[0], m_ZMP_vel[m_C], 0};

  int N = (int)(m_delta / m_delta_control);

  m_X_MPC.clear();
  m_Y_MPC.clear();
  m_X_MPC.push_back(Integration_Mat * Eigen::Vector3d{P_c_k.x(), V_c_k.x(), P_z_k.x()} + Integration_Vec * v_z_i.x());
  m_Y_MPC.push_back(Integration_Mat * Eigen::Vector3d{P_c_k.y(), V_c_k.y(), P_z_k.y()} + Integration_Vec * v_z_i.y());

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

  int t_indx(1);
  double ts = m_timestamp[t_indx - 1];
  j_f = 0;
  j_Max_C = 0;
  if(m_timestamp.size() != 0)
  {
    while(ts + m_Tds < m_Tc)
    {
      t_indx += 1;

      if(t_indx > m_timestamp.size())
      {
        break;
      }
      ts = m_timestamp[t_indx - 1];
    }
    j_Max_C = t_indx + 1;
  }
  j_fm1 = j_f - 1;

  N_variable = 2 * (m_C + j_Max_C);

  m_D = (int)std::round(m_Tds / m_delta) - Tds_offset;
  count_Dstep = (int)(std::round(std::min(m_tk / m_delta, (double)(m_D + 1))));

  // mc_rtc::log::info("countD {}, m_D {} ,t_k : {}; Tc : {} ; Tds {} ; j_f_max : {}",count_Dstep,m_D,m_tk, m_Tc
  // ,m_Tds,j_Max_C);
  ZMP_Constraints();
  FootSteps_Constraints();
  Stability_Constraints();
  Compute_Stability_Range();
  // mc_rtc::log::info("Pu min\n{}", P_u_k_min);
  // mc_rtc::log::info("Pu max\n{}", P_u_k_max);

  Eigen::MatrixXd M_zmp = Eigen::MatrixXd::Zero(N_variable, N_variable);
  Eigen::MatrixXd M_steps = Eigen::MatrixXd::Zero(N_variable, N_variable);
  Eigen::MatrixXd m = Eigen::MatrixXd::Zero(2 * m_C, N_variable);
  M_zmp.block(0, 0, 2 * m_C, 2 * m_C) = Eigen::MatrixXd::Identity(2 * m_C, 2 * m_C);
  M_steps.block(2 * m_C, 2 * m_C, 2 * j_Max_C, 2 * j_Max_C) = Eigen::MatrixXd::Identity(2 * j_Max_C, 2 * j_Max_C);
  Eigen::VectorXd b_zmp = Eigen::VectorXd::Zero(N_variable);
  Eigen::VectorXd b_steps = Eigen::VectorXd::Zero(N_variable);

  for(int i = 0; i < j_Max_C; i++)
  {
    b_steps(2 * m_C + 2 * i) = m_Xf(i + 1);
    b_steps(2 * m_C + 2 * i + 1) = m_Yf(i + 1);
  }

  std::chrono::high_resolution_clock::time_point t_clock = std::chrono::high_resolution_clock::now();

  m_Q = Eigen::MatrixXd::Identity(M_zmp.rows(), M_zmp.cols()) * 1e-12 
        + (M_zmp.transpose() * M_zmp)
        + m_Beta * (M_steps.transpose() * M_steps) 
        + m_Beta_traj * (M_zmp_traj.transpose() * M_zmp_traj);

  m_p = (-M_zmp.transpose() * b_zmp) 
        + m_Beta * (-M_steps.transpose() * b_steps)
        + m_Beta_traj * (-M_zmp_traj.transpose() * b_zmp_traj);

  if(m_Tail == "None")
  {
    // m_p += m_Beta_stab * (-A_stab.transpose() * b_stab);
    // m_Q += m_Beta_stab * (A_stab.transpose() * A_stab);
    Aeq = Eigen::MatrixXd::Zero(1, N_variable);
    beq = Eigen::VectorXd::Zero(1);
  }
  else if(Use_Stability_Task)
  {
    m_p += m_Beta_stab * (-A_stab.transpose() * b_stab);
    m_Q += m_Beta_stab * (A_stab.transpose() * A_stab);
    Aeq = Eigen::MatrixXd::Zero(1, N_variable);
    beq = Eigen::VectorXd::Zero(1);
  }
  else
  {
    Aeq = A_stab;
    beq = b_stab;
  }

  Aineq = Eigen::MatrixXd::Zero(Aineq_steps.rows() + Aineq_zmp.rows(), N_variable);
  bineq = Eigen::VectorXd::Zero(bineq_steps.rows() + bineq_zmp.rows());
  Aineq << Aineq_zmp, Aineq_steps;
  bineq << bineq_zmp, bineq_steps;

  QP_Output = solveQP();
  stab_error = (A_stab * QP_Output - b_stab).norm();
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

  if(!QPsuccess && m_Tail != "None" && Allow_None && !Use_Stability_Task)
  {

    QPsuccess = false;

    mc_rtc::log::warning("[ISMPC] Ignoring Stability cstr");
    m_Tail = "None";
    Stability_Constraints();
    m_p += m_Beta_stab * (-A_stab.transpose() * b_stab);
    m_Q += m_Beta_stab * (A_stab.transpose() * A_stab);
    Aeq = Eigen::MatrixXd::Zero(1, N_variable);
    beq = Eigen::VectorXd::Zero(1);
    QP_Output = solveQP();
  }

  if(!QPsuccess)
  {

    // mc_rtc::log::error_and_throw<std::runtime_error>("QP Failed");
    mc_rtc::log::warning("[ISMPC] Ignoring QP");
    Eigen::VectorXd ineq = Aineq * QP_Output - bineq;
    // for (int i = 0 ; i < ineq.rows() ; i++)
    // {
    //   double in(ineq(i));
    //   if (in > 0)
    //   {
    //     mc_rtc::log::info("ctsr broken idx {}, {}",i,in);
    //   }
    // }
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

    if(m_Tail == "None" || Use_Stability_Task)
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
          P_u_k_2 += ((1 - exp(-m_eta * m_delta)) / (m_eta)) * exp(-k * m_eta * m_delta)
                     * Eigen::Vector3d{m_ZMP_vel[k], m_ZMP_vel[k + m_C], 0};
        }
        P_u_k_2 += ((1 - exp(-m_eta * m_delta)) / (m_eta)) * Eigen::Vector3d{Ant_Tail_X, Ant_Tail_Y, 0};
      }

      V_c_k = m_eta * (P_u_k_2 - P_c_k);
      // P_c_k = P_u_k_2 - V_c_k/m_eta;
      // Eigen::Vector3d P_u_error = P_u_k - P_u_k_2; P_u_error.z() = 0.;
      // Eigen::Vector3d P_z_corr = P_u_error / (1 - exp(m_eta * (0.1)));
      // mc_rtc::log::info("P_z_corr \n{}",P_z_corr);
    }

    IntegrateZMPVel();
  }
}

Eigen::VectorXd ISMPC_Solver::solveQP()
{

  int Nvar = m_Q.rows();
  int NIneqConstr = Aineq.rows();
  int NEqConstr = Aeq.rows();
  // QP.tolerance(5e-3);
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
      const Eigen::Vector3d & v_q{_Corners[q].x(), _Corners[q].y(), 0};
      const Eigen::Vector3d & v_l{_Corners[l].x(), _Corners[l].y(), 0};
      const Eigen::Vector3d & v_i{_Corners[i].x(), _Corners[i].y(), 0};
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
