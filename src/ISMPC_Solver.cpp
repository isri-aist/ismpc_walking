#include "../include/ismpc_walking/ISMPC_Solver.h"



ISMPC_Solver::ISMPC_Solver() {}

ISMPC_Solver::ISMPC_Solver(double delta_controller, double delta, double Tp, double Tc)
{

  m_Tc = Tc;
  m_Tp = Tp;
  m_delta = delta;
  m_delta_control = delta_controller;

  m_eta = sqrt(mc_rtc::constants::GRAVITY / CoM_height);
  count_Dstep = 0;
  m_C = static_cast<int>(m_Tc / m_delta);
  m_P = static_cast<int>(m_Tp / m_delta);
  QPsuccess = false;

  Compute_Integration_Matrix();
  Compute_Integration_Vector(0);

  w_k.setZero();
}

void ISMPC_Solver::configure(const ControllerConfiguration & config)
{
  m_dx_f = config.MPC_Footsteps_kin_Constraint_size.x();
  m_dy_f = config.MPC_Footsteps_kin_Constraint_size.y();
  m_dx_f_rect = config.MPC_Footsteps_Constraint_size.x();
  m_dy_f_rect = config.MPC_Footsteps_Constraint_size.y();
  m_dx = config.MPC_ZMP_Constraint_size.x();
  m_dy = config.MPC_ZMP_Constraint_size.y();
  m_dx_u = config.MPC_U_Constraint_size.x();
  m_dy_u = config.MPC_U_Constraint_size.y();
  m_dx_static = config.MPC_ZMP_cstr_square_static.x();
  m_dy_static = config.MPC_ZMP_cstr_square_static.y();
  m_Beta_step = config.Beta_step;
  m_Beta_u = config.Beta_u;
  m_Beta_stab = config.Beta_stab;
  m_Beta_traj = config.Beta_traj;
  m_lambda = config.lambda_;
  m_feet_distance = config.feet_ditance_;
  zmp_delay(config.zmp_delay);
  Slide_ZMP_region = config.sliding_zmp_cstr_region;
  zmp_cstr_next_stp_ratio = config.MPC_ZMP_next_stp_cstr_ratio;
  rect_pose_offset = config.MPC_ZMP_cstr_square_offset;
  rect_pose_offset_static = config.MPC_ZMP_static_cstr_square_offset;
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
  zmp_ref_offset = config.MPC_ZMP_ref_offset_sg_supp;
  zmp_ref_offset_end_step = config.MPC_ZMP_ref_offset_end_step;
  zmp_ref_offset_start_step = config.MPC_ZMP_ref_offset_start_step;

  Compute_Integration_Matrix();
  Compute_Integration_Vector(0);

  mc_rtc::log::info("[ISMPC] Configuration :");
  mc_rtc::log::info("Beta {}", m_Beta_step);
  mc_rtc::log::info("ZMP cstr\n{}", Eigen::Vector2d{m_dx, m_dy});
  mc_rtc::log::info("U cstr\n{}", Eigen::Vector2d{m_dx_u, m_dy_u});
  mc_rtc::log::info("Footsteps kin cstr\n{}", Eigen::Vector2d{m_dx_f, m_dy_f});
  mc_rtc::log::info("Footsteps cstr\n{}", Eigen::Vector2d{m_dx_f_rect, m_dy_f_rect});
  mc_rtc::log::info("CoM h {}", CoM_height);
  mc_rtc::log::info("Tp {} Tc {}", m_Tp, m_Tc);
  mc_rtc::log::info("MPC_delta {}", m_delta);
  mc_rtc::log::info("Controller delta {}", m_delta_control);
  mc_rtc::log::info("Use of whole polygon support {}", !Slide_ZMP_region);
  mc_rtc::log::info("Stability Task {}", Use_Stability_Task);
}



void ISMPC_Solver::init_MPC(const MPC_state & mpc_state,
                            std::string Tail,
                            int Steps_Desired,
                            int Step)
{
  P_c_k = mpc_state.Pck;
  V_c_k = mpc_state.Vck;
  P_z_k = mpc_state.Pzk;

  DoubleSupport = mpc_state.doubleSupport;
  m_t_lift = mpc_state.t_lift;
  
  m_tk = std::max(0., mpc_state.t_k);
  m_t_global = mpc_state.t;
  m_delay_elapsed = std::min( m_delay - (m_t_global - m_t_delay) , m_delay);
  if( m_t_global - m_t_delay > m_delta || m_tk == 0 || m_delay_elapsed < 0)
  {
    U_k = mpc_state.Uk;
    m_t_delay = m_t_global;
    m_delay_elapsed = m_delay;
  }


  P_z_k_delayed = P_z_k + (1 - exp(-m_lambda * m_delay_elapsed)) * (U_k - P_z_k);
  m_Tail = Tail;
  
  m_support_foot = mpc_state.input_Support_FootName;
  
  P_u_k = P_c_k + (V_c_k / m_eta);
  X_0_swing_foot_initial = mpc_state.X_0_Initial_SwingFoot;

  X_0_support_foot = mpc_state.X_0_SupportFoot;
 

  input_steps_ = mpc_state.planned_steps_;
  m_timestamp = mpc_state.planned_timesteps_;
  m_input_Tds = mpc_state.tds;


  N_Steps = Step;
  N_Steps_Desired = Steps_Desired;

  if(input_steps_.size() == 0)
  {
    mc_rtc::log::warning("[ISMPC] No Footsteps target provided");
    input_steps_.push_back(X_0_swing_foot_initial);
  }

  R_0_support = X_0_support_foot.rotation();
  R_support_0 = R_0_support.transpose();

  w_k.setZero();
  m_eta = sqrt(mc_rtc::constants::GRAVITY/CoM_height);
  Compute_Integration_Matrix();
}

void ISMPC_Solver::create_cstr_matrices(Eigen::MatrixXd & A_out, Eigen::VectorXd & b_out, std::vector<SupportPolygon> & A_in, const std::vector<Eigen::VectorXd> & b_in)
{
  Eigen::Index k = 0;
  Eigen::Index cstr_index = 0;
  for(size_t i_ineq = 0; i_ineq < A_in.size(); i_ineq++)
  {
    Eigen::Index n_vertice = (A_in[i_ineq].normals().rows());

    A_out.block(cstr_index, k, n_vertice, 2) = A_in[i_ineq].normals();
    b_out.segment(cstr_index, n_vertice) = b_in[i_ineq];

    k += 2;
    cstr_index += n_vertice;
  }
}

void ISMPC_Solver::create_cstr_matrices(Eigen::MatrixXd & A_out, Eigen::VectorXd & b_out, std::vector<Eigen::MatrixX2d> & A_in, const std::vector<Eigen::VectorXd> & b_in)
{
  Eigen::Index step = 0;
  Eigen::Index cstr_index = 0;
  for(Eigen::Index i_ineq = 0; i_ineq < static_cast<Eigen::Index>(A_in.size()); i_ineq++)
  {

    Eigen::MatrixX2d ineq = A_in[i_ineq];

    A_out.block(cstr_index, step, ineq.rows(), 2) = ineq.block(0, 0, ineq.rows(), 2);
    b_out.segment(cstr_index, ineq.rows()) = b_in[i_ineq].segment(0, ineq.rows());

    step += 2;
    cstr_index += ineq.rows();
  }
}

Eigen::MatrixXd ISMPC_Solver::create_zmp_matrix(bool addDelay )
{
  Eigen::MatrixXd A_out = Eigen::MatrixXd::Zero(2 * m_C , 2 * m_C );
  for(int i = 0; i < m_C; i++)
  {
    for(int k = 0; k <= i; k++)
    {
      double t_m_tk = (1 + i - k) * m_delta;
      // if(k == i && addDelay){t_m_tk -= ( i==0 ? m_delay_elapsed : m_delay);}
      A_out.block(2*i,2*k,2,2) = Eigen::Matrix2d::Identity() * (1-exp( - m_lambda * t_m_tk));
    }
  }
  return A_out;
}

Eigen::MatrixXd ISMPC_Solver::create_u_matrix()
{
  Eigen::MatrixXd A_out = Eigen::MatrixXd::Zero(2 * m_C , 2 * m_C );
  A_out = Eigen::MatrixXd::Zero(2 * m_C , 2 * m_C );
  for(int i = 0; i < m_C; i++)
  {
    for(int k = 0; k <= i; k++)
    {
      A_out.block(2*i,2*k,2,2) = Eigen::Matrix2d::Identity();
    }
  }
  return A_out;
}

void ISMPC_Solver::Compute_Integration_Matrix()
{
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
}

void ISMPC_Solver::Static_ZMP_Constraints()
{

  std::vector<Eigen::VectorXd> b_zmp_ineq;
  std::vector<Eigen::VectorXd> b_u_ineq;

  zmp_cstr_polygons.clear();
  std::vector<SupportPolygon> u_cstr_polygons;

  b_zmp_ineq.clear();
  double sgn = -1;

  if(m_support_foot == "RightFoot"){sgn = 1;} 
  const Eigen::Vector3d rect_offset_support =
      X_0_support_foot.rotation().transpose() * Eigen::Vector3d{rect_pose_offset_static.x(), sgn * rect_pose_offset_static.y(), 0};


  const Eigen::Vector3d rect_offset_swing =
      X_0_support_foot.rotation().transpose() * Eigen::Vector3d{rect_pose_offset_static.x(), -sgn * rect_pose_offset_static.y(), 0};

  Rectangle Rect_jm1 = Rectangle(X_0_swing_foot_initial, Eigen::Vector2d{m_dx_static, m_dy_static}, rect_offset_swing);
  Rectangle Rect_j = Rectangle(X_0_support_foot, Eigen::Vector2d{m_dx_static, m_dy_static}, rect_offset_support);
  Rectangle Rect_jm1_u = Rectangle(X_0_swing_foot_initial, Eigen::Vector2d{m_dx_u, m_dy_u});
  Rectangle Rect_j_u = Rectangle(X_0_support_foot, Eigen::Vector2d{m_dx_u, m_dy_u});

  SupportPolygon SuppPoly = SupportPolygon(Rect_jm1, Rect_j);
  SupportPolygon SuppPoly_u = SupportPolygon(Rect_jm1_u, Rect_j_u);

  m_double_support_polygon = SuppPoly;

  ZMP_ref_traj.clear();
  ZMP_max_ref_traj.clear();
  ZMP_min_ref_traj.clear();
  All_poly.clear();

  Eigen::MatrixXd Delta = Eigen::MatrixXd::Zero(N_variable, N_variable); // Matrix to derive the ZMP position from u
  Eigen::MatrixXd DeltaNoDelay = Eigen::MatrixXd::Zero(N_variable, N_variable); // Matrix to derive the ZMP position from u
  Eigen::MatrixXd u_Delta = Delta;
  Delta.block(0,0,2*m_C,2*m_C) = create_zmp_matrix(true); 
  DeltaNoDelay.block(0,0,2*m_C,2*m_C) = create_zmp_matrix(false); 
  u_Delta.block(0,0,2*m_C,2*m_C) = create_u_matrix(); 

  P_u_k_max = m_eta * m_delta * R_0_support * P_z_k;
  P_u_k_min = m_eta * m_delta * R_0_support * P_z_k;

  sva::PTransformd X_0_step_j = X_0_support_foot;
  sva::PTransformd X_0_step_jm1 = X_0_swing_foot_initial;

  for(int i = 0; i < m_C; i++)
  {

    sva::PTransformd X_0_step_stop =
        sva::PTransformd(X_0_step_j.rotation(), (Rect_j.get_center() + Rect_jm1.get_center()) * 0.5);

    sva::PTransformd ZMP_Zone = X_0_step_stop;


    ZMP_ref_traj.push_back(ZMP_Zone.translation().x() - P_z_k_delayed.x());
    ZMP_ref_traj.push_back(ZMP_Zone.translation().y() - P_z_k_delayed.y());

    zmp_cstr_polygons.push_back(SuppPoly);
    u_cstr_polygons.push_back(SuppPoly_u);

    ZMP_max_ref_traj.push_back(SuppPoly.get_center() + R_support_0 * Eigen::Vector3d{m_dx_static / 2, m_dy_static / 2, 0});
    ZMP_min_ref_traj.push_back(SuppPoly.get_center() - R_support_0 * Eigen::Vector3d{m_dx_static / 2, m_dy_static / 2, 0});

    if(i == 0)
    {
      SuppPolyCorners = zmp_cstr_polygons[i].Get_Polygone_Corners();
      m_ref_zmp = ZMP_Zone.translation();
    }

    Eigen::MatrixX2d normals(zmp_cstr_polygons.back().normals());
    Eigen::VectorXd offsets(zmp_cstr_polygons.back().offsets());

    b_zmp_ineq.push_back(offsets - normals * P_z_k_delayed.segment(0, 2));
    b_u_ineq.push_back(u_cstr_polygons.back().offsets() - u_cstr_polygons.back().normals() * P_z_k_delayed.segment(0, 2));

    All_poly.push_back(zmp_cstr_polygons.back().Get_Polygone_Corners());
  }

  int N_zmp_cstr = 0;
  for(size_t k = 0; k < zmp_cstr_polygons.size(); k++)
  {
    N_zmp_cstr += static_cast<int>(zmp_cstr_polygons[k].normals().rows());
  }
  int N_u_cstr = 0;
  for(size_t k = 0; k < u_cstr_polygons.size(); k++)
  {
    N_u_cstr += static_cast<int>(u_cstr_polygons[k].normals().rows());
  }
  // mc_rtc::log::success("ZMP cstr computed, Ncstr = {}", N_zmp_cstr);
  Eigen::MatrixXd ZMP_Cstr = Eigen::MatrixXd::Zero(N_zmp_cstr, N_variable);
  Eigen::VectorXd b_zmp = Eigen::VectorXd::Zero(ZMP_Cstr.rows());
  Eigen::MatrixXd U_Cstr = Eigen::MatrixXd::Zero(N_u_cstr, N_variable);
  Eigen::VectorXd b_u = Eigen::VectorXd::Zero(U_Cstr.rows());

  // std::cout << "ZMP_cstr_rows" << ZMP_Cstr.rows() << std::endl;

  create_cstr_matrices(ZMP_Cstr,b_zmp,zmp_cstr_polygons,b_zmp_ineq);

  create_cstr_matrices(U_Cstr,b_u,u_cstr_polygons,b_u_ineq);



  Aineq_zmp.resize(2 * ZMP_Cstr.rows(),N_variable);
  bineq_zmp.resize(Aineq_zmp.rows());

  Aineq_zmp << ZMP_Cstr * Delta , ZMP_Cstr * DeltaNoDelay;
  bineq_zmp << b_zmp , b_zmp;
  A_zmp = Delta;
  b_zmp_traj = Eigen::Map<Eigen::VectorXd>(ZMP_ref_traj.data(), ZMP_ref_traj.size());
  M_zmp_traj = Eigen::MatrixXd::Zero(b_zmp_traj.rows(), N_variable);
  M_zmp_traj.block(0, 0, b_zmp_traj.rows(), b_zmp_traj.rows()) = Delta.block(0, 0, b_zmp_traj.rows(), b_zmp_traj.rows());
}

void ISMPC_Solver::ZMP_Transition_Constraint(Eigen::MatrixXd & A_out,Eigen::VectorXd & b_out,SupportPolygon PolySS)
{
  const double t_transi_ds_ss = m_Tds - m_tk - m_delta;
  if(t_transi_ds_ss < 0)
  {
    A_out.resize(1,N_variable);
    A_out.setZero();
    b_out.resize(1);
    b_out.setZero();
    return;
  }
  const double dt = m_delta_control/2;
  const Eigen::Index indx_transi_ds_ss = static_cast<Eigen::Index>(t_transi_ds_ss / m_delta);
  const Eigen::Index N_integration = static_cast<Eigen::Index>(m_delta/dt);
  Eigen::MatrixXd A_zmp = Eigen::MatrixXd::Zero(2,N_variable);
  A_out.resize(N_integration * PolySS.offsets().rows(),N_variable);
  A_out.setZero();
  b_out.resize(A_out.rows());
  b_out.setZero();

  for (Eigen::Index i = 0 ; i < N_integration ; i++)
  {
    for(Eigen::Index k = 0 ; k <= indx_transi_ds_ss ; k++)
    {
      double t_m_tk = t_transi_ds_ss + static_cast<double>(i) * dt - static_cast<double>(k) * m_delta ;
      // if(k == i){t_m_tk -= ( i==0 ? m_delay_elapsed : m_delay);}
      A_zmp.block(0,2 * k , 2,2) = Eigen::Matrix2d::Identity() * (1 - exp(-m_lambda * (t_m_tk)));
    }
    A_out.block(i * PolySS.offsets().rows(),0,PolySS.offsets().rows(),N_variable) = PolySS.normals() * A_zmp;
    b_out.segment(i * PolySS.offsets().rows(),PolySS.offsets().rows()) = PolySS.offsets() - PolySS.normals() * P_z_k_delayed.segment(0,2);
  }
  
}

void ISMPC_Solver::ZMP_Constraints()
{
  std::chrono::high_resolution_clock::time_point t_clock = std::chrono::high_resolution_clock::now();

  // std::chrono::high_resolution_clock::time_point t_clock_0 = std::chrono::high_resolution_clock::now();

  std::vector<Eigen::VectorXd> b_zmp_ineq = std::vector<Eigen::VectorXd>{} ;
  std::vector<Eigen::VectorXd> b_u_ineq = std::vector<Eigen::VectorXd>{} ;
  zmp_cstr_polygons = std::vector<SupportPolygon>{};
  std::vector<SupportPolygon> u_cstr_polygons = std::vector<SupportPolygon>{};
  double sgn = -1; //change between 1 and -1 depending of support foot (1 if right)
  if(m_support_foot == "RightFoot") // Right Support
  {
    sgn = 1;
  }
  Eigen::Vector2d direction = Eigen::Vector2d::Zero();
  if( (input_steps_[0] * X_0_support_foot.inv()).translation().x() > 0.1 )
  {
    direction = Eigen::Vector2d{1.,0};
    
  }
  else if((input_steps_[0] * X_0_support_foot.inv()).translation().x() < -0.1)
  {
    direction = Eigen::Vector2d{-1,0};
  }


  Eigen::Vector3d rect_offset_support =
      X_0_support_foot.rotation().transpose() * Eigen::Vector3d{rect_pose_offset.x(), sgn * rect_pose_offset.y(), 0};

  Eigen::Vector3d rect_offset_swing =
      X_0_support_foot.rotation().transpose() * Eigen::Vector3d{rect_pose_offset.x(), -sgn * rect_pose_offset.y(), 0};

  Eigen::Vector3d zmp_ref_offset_sg =
      X_0_support_foot.rotation().transpose() * Eigen::Vector3d{zmp_ref_offset.x(), sgn * zmp_ref_offset.y(), 0};

  Eigen::Vector3d zmp_ref_end_step =
      X_0_support_foot.rotation().transpose() * Eigen::Vector3d{zmp_ref_offset_end_step.x() * direction.x(), 0, 0};
  Eigen::Vector3d zmp_ref_start_step =
      X_0_support_foot.rotation().transpose() * Eigen::Vector3d{zmp_ref_offset_start_step.x() * -direction.x(), 0, 0};

  Eigen::Vector3d zmp_ref_offset_swing =
      X_0_support_foot.rotation().transpose() * Eigen::Vector3d{zmp_ref_offset.x(), -sgn * zmp_ref_offset.y(), 0};


  // std::chrono::duration<double, std::milli> time_span_0 = std::chrono::high_resolution_clock::now() - t_clock_0;
  // mc_rtc::log::info("[ZMP cstr init] offset {} ms", time_span_0.count());

  // t_clock_0 = std::chrono::high_resolution_clock::now();
  Rectangle Sliding_rect =
      Rectangle(mc_rbdyn::rpyFromMat(X_0_support_foot.rotation()).z(), Eigen::Vector2d{m_dx, m_dy});
  Rectangle Sliding_rect_u =
      Rectangle(mc_rbdyn::rpyFromMat(X_0_support_foot.rotation()).z(), Eigen::Vector2d{m_dx_u, m_dy_u});


  Rectangle Rect_jm1 = Rectangle(X_0_swing_foot_initial, Eigen::Vector2d{m_dx, m_dy}, rect_offset_swing);
  Rectangle Rect_j = Rectangle(X_0_support_foot, Eigen::Vector2d{m_dx, m_dy}, rect_offset_support);

  Rectangle Rect_jm1_u = Rectangle(X_0_swing_foot_initial, Eigen::Vector2d{m_dx_u, m_dy_u}, rect_offset_swing);
  Rectangle Rect_j_u = Rectangle(X_0_support_foot, Eigen::Vector2d{m_dx_u, m_dy_u}, rect_offset_support);


  SupportPolygon Poly_Rect = SupportPolygon(Sliding_rect);
  SupportPolygon Poly_Rect_u = SupportPolygon(Sliding_rect_u);

  SupportPolygon SuppPoly = SupportPolygon(Rect_jm1, Rect_j);
  SupportPolygon SuppPoly_u = SupportPolygon(Rect_jm1_u, Rect_j_u);


  SupportPolygon S_Support_Poly = SupportPolygon(Rect_j);
  SupportPolygon S_Support_Poly_u = SupportPolygon(Rect_j_u);

  // TOFIX Make the controller crash when robot is falling
  // Eigen::MatrixXd Aineq_zmp_transi = Eigen::MatrixXd::Zero(0,N_variable);
  // Eigen::VectorXd bineq_zmp_transi = Eigen::VectorXd::Zero(0);
  // ZMP_Transition_Constraint(Aineq_zmp_transi,bineq_zmp_transi,S_Support_Poly);

  ZMP_ref_traj.clear();
  ZMP_max_ref_traj.clear();
  ZMP_min_ref_traj.clear();
  All_poly.clear();

  Eigen::MatrixXd Delta = Eigen::MatrixXd::Zero(N_variable, N_variable);
  Delta.block(0,0,2 * m_C , 2 * m_C) = create_zmp_matrix(true);
  Eigen::MatrixXd Delta_zmp_ref = Delta;

  P_u_k_max = m_eta * m_delta * R_0_support * P_z_k;
  P_u_k_min = m_eta * m_delta * R_0_support * P_z_k;
  double NextStepTiming(0);
  if(m_timestamp.size() != 0)
  {
    NextStepTiming = m_timestamp[j_f];
    // std::cout << "Ts " << NextStepTiming << std::endl;
  }
  double PrevStepTime = 0;

  sva::PTransformd X_0_step_j = X_0_support_foot;
  sva::PTransformd X_0_step_jm1 = X_0_swing_foot_initial;

  for(int i = 0; i < m_C; i++)
  {
    
    if(m_tk + static_cast<double>(i) * m_delta >= NextStepTiming && j_f + 1 < static_cast<int>(m_timestamp.size()) )
    {
      m_D = static_cast<int>(m_Tds / m_delta) - Tds_offset;

      // j_f = std::min(j_f + 1, (int)input_steps_.size() - 1);
      j_f += 1;
      j_fm1 = j_f - 1;
      count_Dstep = 1;
      sgn *= -1;

      NextStepTiming = m_timestamp[j_f];
      PrevStepTime = m_timestamp[j_fm1];

      X_0_step_jm1 = X_0_step_j;
      X_0_step_j = input_steps_[j_f - 1];
    
      direction = Eigen::Vector2d::Zero();
      if( (input_steps_[j_f] * X_0_step_j.inv()).translation().x() > 0.1 )
      {
        direction = Eigen::Vector2d{1.,0};
        
      }
      else if((input_steps_[j_f] * X_0_step_j.inv()).translation().x() < -0.1)
      {
        direction = Eigen::Vector2d{-1,0};
      }
      zmp_ref_end_step =
      X_0_step_j.rotation().transpose() * Eigen::Vector3d{zmp_ref_offset_end_step.x() * direction.x(), 0, 0};
      zmp_ref_start_step =
      X_0_step_j.rotation().transpose() * Eigen::Vector3d{zmp_ref_offset_start_step.x() * -direction.x(), 0, 0};

      Eigen::Vector3d offset = rect_offset_swing;
      rect_offset_swing = rect_offset_support;
      rect_offset_support = offset;
      offset = zmp_ref_offset_swing;
      zmp_ref_offset_swing = zmp_ref_offset_sg;
      zmp_ref_offset_sg = offset;

      Rect_jm1 = Rectangle(X_0_step_jm1, Eigen::Vector2d{m_dx, m_dy},rect_offset_swing);
      Rect_j = Rectangle(X_0_step_j, Eigen::Vector2d{m_dx, m_dy},rect_offset_support);

      Rect_jm1_u = Rectangle(X_0_step_jm1, Eigen::Vector2d{m_dx_u, m_dy_u}, rect_offset_swing);
      Rect_j_u = Rectangle(X_0_step_j, Eigen::Vector2d{m_dx_u, m_dy_u}, rect_offset_support);
      
      Sliding_rect = Rectangle(mc_rbdyn::rpyFromMat(X_0_step_jm1.rotation()).z(),
                               Eigen::Vector2d{m_dx, m_dy} * zmp_cstr_next_stp_ratio);

      Poly_Rect = SupportPolygon(Sliding_rect);

      Sliding_rect_u = Rectangle(mc_rbdyn::rpyFromMat(X_0_step_jm1.rotation()).z(),
                               Eigen::Vector2d{m_dx_u, m_dy_u});

      Poly_Rect_u = SupportPolygon(Sliding_rect_u);

    
    }

    const double n = std::max(0., std::min(static_cast<double>(m_D ), count_Dstep));

    const double alpha = std::min(1.0,std::max(0., n / (static_cast<double>(m_D))));

    // mc_rtc::log::info("i {} jf {} alpha {},D {}, Ts {}",i,j_f,alpha,m_D,NextStepTiming);

    
    
    if(j_f == 0 || !AutoFootstepPlacement)
    {
      // t_clock_0 = std::chrono::high_resolution_clock::now();
      if(j_f > 0)
      {
        if(!Slide_ZMP_region)
        {
          SuppPoly = SupportPolygon(Rect_jm1, Rect_j);
          SuppPoly_u = SupportPolygon(Rect_jm1_u, Rect_j_u);
        }
        
        S_Support_Poly = SupportPolygon(Rect_j);
        S_Support_Poly_u = SupportPolygon(Rect_j_u);
      }
      if(N_Steps == N_Steps_Desired && i == 0)
      {
        sva::PTransformd X_0_step_stop_j =
            sva::PTransformd(X_0_step_j.rotation(), (X_0_step_j.translation() + X_0_step_jm1.translation()) * 0.5);
        Rect_j = Rectangle(X_0_step_stop_j, Eigen::Vector2d{m_dx, m_dy});
        Rect_j_u = Rectangle(X_0_step_stop_j, Eigen::Vector2d{m_dx_u, m_dy_u});
        SuppPoly = SupportPolygon(Rect_jm1, Rect_j);
        SuppPoly_u = SupportPolygon(Rect_jm1_u, Rect_j_u);
      }

      sva::PTransformd ZMP_Zone =
          sva::PTransformd(X_0_step_j.rotation(),
                           (   Rect_j.get_center() + zmp_ref_offset_sg) * alpha + 
                           ( Rect_jm1.get_center() + zmp_ref_offset_swing) * (1-alpha) 
                            );
      
      Rectangle ZMP_rect = Rectangle(ZMP_Zone, Eigen::Vector2d{m_dx, m_dy});

      sva::PTransformd U_Zone =
          sva::PTransformd(X_0_step_j.rotation(),
                           (   Rect_j_u.get_center()) * alpha + 
                           ( Rect_jm1_u.get_center()) * (1-alpha) 
                            );
      
      Rectangle U_rect = Rectangle(U_Zone, Eigen::Vector2d{m_dx_u, m_dy_u});

      if(Slide_ZMP_region || alpha == 1)
      {
        if(alpha == 1)
        {
          zmp_cstr_polygons.push_back(S_Support_Poly);
          u_cstr_polygons.push_back(S_Support_Poly_u);
        }
        else
        {
          zmp_cstr_polygons.push_back(SupportPolygon(ZMP_rect));
          u_cstr_polygons.push_back(SupportPolygon(U_rect));

        }

        ZMP_max_ref_traj.push_back( ZMP_rect.get_center() + R_support_0 * Eigen::Vector3d{m_dx / 2, m_dy / 2, 0});
        ZMP_min_ref_traj.push_back( ZMP_rect.get_center() - R_support_0 * Eigen::Vector3d{m_dx / 2, m_dy / 2, 0});
      }
      else
      {
        u_cstr_polygons.push_back(SuppPoly_u);
        zmp_cstr_polygons.push_back(SuppPoly);
        
        ZMP_max_ref_traj.push_back(ZMP_rect.get_center() + R_support_0 * Eigen::Vector3d{m_dx / 2, m_dy / 2, 0});
        ZMP_min_ref_traj.push_back(ZMP_rect.get_center() - R_support_0 * Eigen::Vector3d{m_dx / 2, m_dy / 2, 0});
      }

      
      ZMP_ref_traj.push_back((Rect_j.get_center() + zmp_ref_offset_sg).x() - P_z_k_delayed.x());
      ZMP_ref_traj.push_back((Rect_j.get_center() + zmp_ref_offset_sg).y() - P_z_k_delayed.y());

      Eigen::MatrixX2d normals(zmp_cstr_polygons.back().normals());
      Eigen::VectorXd offsets(zmp_cstr_polygons.back().offsets());

      b_zmp_ineq.push_back(offsets - normals * P_z_k_delayed.segment(0, 2));
      b_u_ineq.push_back(u_cstr_polygons.back().offsets() - u_cstr_polygons.back().normals() * P_z_k_delayed.segment(0, 2));

      All_poly.push_back(zmp_cstr_polygons.back().Get_Polygone_Corners());

      // time_span_0 = std::chrono::high_resolution_clock::now() - t_clock_0;
      // mc_rtc::log::info("[ZMP cstr] Cstr at j {} took {} ms", j_f ,time_span_0.count());
    }

    else if(j_f == 1)
    {

      double l = sgn * m_feet_distance;

      sva::PTransformd X_0_step_j_min;
      sva::PTransformd X_0_step_j_max;
      X_0_step_j_min =
          sva::PTransformd(X_0_step_j.rotation(),
                           X_0_step_jm1.translation() + X_0_step_j.rotation().transpose() * ( Eigen::Vector3d{0., l, 0.}
                               - Eigen::Vector3d{m_dx_f / 2, int(m_support_foot == "LeftFoot" ) * m_dy_f, 0}) );
      X_0_step_j_max =
          sva::PTransformd(X_0_step_j.rotation(),
                           X_0_step_jm1.translation() + X_0_step_j.rotation().transpose() * ( Eigen::Vector3d{0., l, 0.}
                               +  Eigen::Vector3d{m_dx_f / 2, int(m_support_foot == "RightFoot" ) * m_dy_f, 0}) );


      sva::PTransformd ZMP_Zone_min(Eigen::Matrix3d::Identity(),
                                    (X_0_step_j_min.translation() * alpha + X_0_step_jm1.translation() * (1-alpha)));
      sva::PTransformd ZMP_Zone_max(Eigen::Matrix3d::Identity(),
                                    (X_0_step_j_max.translation() * alpha + X_0_step_jm1.translation() * (1-alpha)));
                                        

      sva::PTransformd ZMP_Zone =
          sva::PTransformd(Eigen::Matrix3d::Identity(), ( Rect_jm1.get_center() + zmp_ref_offset_swing) * (1-alpha));

      sva::PTransformd U_Zone =
          sva::PTransformd(Eigen::Matrix3d::Identity(), ( Rect_jm1_u.get_center()) * (1-alpha));

      ZMP_max_ref_traj.push_back( ZMP_Zone_max.translation() + X_0_step_j.rotation().transpose() * Eigen::Vector3d{m_dx / 2, m_dy / 2, 0});
      ZMP_min_ref_traj.push_back( ZMP_Zone_min.translation() - X_0_step_j.rotation().transpose() * Eigen::Vector3d{m_dx / 2, m_dy / 2, 0});

      zmp_cstr_polygons.push_back(Poly_Rect);
      u_cstr_polygons.push_back(Poly_Rect_u);

      Eigen::MatrixX2d normals(zmp_cstr_polygons[i].normals());
      Eigen::VectorXd offsets(zmp_cstr_polygons[i].offsets());

      Eigen::VectorXd bcstr = offsets 
                              - normals * P_z_k_delayed.segment(0, 2) + 
                                normals * ZMP_Zone.translation().segment(0, 2) + 
                                normals * (rect_offset_support).segment(0,2) * alpha;

      b_zmp_ineq.push_back(bcstr);
      b_u_ineq.push_back(u_cstr_polygons.back().offsets()
                        - u_cstr_polygons.back().normals() * P_z_k_delayed.segment(0, 2) + 
                          u_cstr_polygons.back().normals() * U_Zone.translation().segment(0, 2));

      ZMP_ref_traj.push_back( - P_z_k_delayed.x() + (rect_offset_support + zmp_ref_offset_sg).x());
      ZMP_ref_traj.push_back( - P_z_k_delayed.y() + (rect_offset_support + zmp_ref_offset_sg).y());


      Delta.block(2*i ,2 * m_C + 2 * (j_f - 1),2,2) = -Eigen::Matrix2d::Identity()*alpha;
      Delta_zmp_ref.block(2*i ,2 * m_C + 2 * (j_f - 1),2,2) = -Eigen::Matrix2d::Identity();

      All_poly.push_back(zmp_cstr_polygons.back().Get_Polygone_Corners());
      if(i == 0)
      {
        SuppPolyCorners = zmp_cstr_polygons.back().Get_Polygone_Corners();
      }
    }

    else
    {

      ZMP_ref_traj.push_back( - P_z_k_delayed.x() + (rect_offset_support + zmp_ref_offset_sg).x());
      ZMP_ref_traj.push_back( - P_z_k_delayed.y() + (rect_offset_support + zmp_ref_offset_sg).y());

      zmp_cstr_polygons.push_back(Poly_Rect);
      u_cstr_polygons.push_back(Poly_Rect_u);
      

      Eigen::MatrixX2d normals(zmp_cstr_polygons.back().normals());
      Eigen::VectorXd offsets(zmp_cstr_polygons.back().offsets());

      b_zmp_ineq.push_back(offsets - normals * P_z_k_delayed.segment(0, 2) + 
                                     normals * ((rect_offset_support).segment(0,2)) * alpha + 
                                     normals * ((rect_offset_swing).segment(0,2)) * (1-alpha) );

      b_u_ineq.push_back(u_cstr_polygons.back().offsets() - u_cstr_polygons.back().normals() * P_z_k_delayed.segment(0, 2) );

 
      Delta.block(2*i ,2 * m_C + 2 * (j_f - 1),2,2) = -Eigen::Matrix2d::Identity()*alpha;
      Delta.block(2*i ,2 * m_C + 2 * (j_fm1 - 1),2,2) = -Eigen::Matrix2d::Identity()*(1-alpha);

      Delta_zmp_ref.block(2*i ,2 * m_C + 2 * (j_f - 1),2,2) = -Eigen::Matrix2d::Identity();
      

      if(i == 0)
      {
        SuppPolyCorners = zmp_cstr_polygons[i].Get_Polygone_Corners();
      }
    }
    if(alpha==1)
    {
      ZMP_ref_traj[2 * i] += zmp_ref_end_step.x(); 
      ZMP_ref_traj[2 * i + 1] += zmp_ref_end_step.y(); 
    }
    else
    {
      ZMP_ref_traj[2 * i] += zmp_ref_start_step.x(); 
      ZMP_ref_traj[2 * i + 1] += zmp_ref_start_step.y();       
    }
    if(i == 0)
    {
      SuppPolyCorners = zmp_cstr_polygons.back().Get_Polygone_Corners();
      m_support_state = alpha;
      m_ref_zmp = Eigen::Vector3d{ZMP_ref_traj[0],ZMP_ref_traj[1],0} + P_z_k_delayed ;
    }

    count_Dstep += 1;
  }

  // std::cout << ("[ZMP cstr] generated") << std::endl;

  // t_clock = std::chrono::high_resolution_clock::now();

  int N_zmp_cstr = 0;
  int N_u_cstr = 0;
  for(size_t k = 0; k < zmp_cstr_polygons.size(); k++)
  {
    N_zmp_cstr += static_cast<int>(zmp_cstr_polygons[k].normals().rows());
  }
  for(size_t k = 0; k < u_cstr_polygons.size(); k++)
  {
    N_u_cstr += static_cast<int>(u_cstr_polygons[k].normals().rows());
  }
  // mc_rtc::log::success("ZMP cstr computed, Ncstr = {}", N_zmp_cstr);
  Eigen::MatrixXd ZMP_Cstr = Eigen::MatrixXd::Zero(N_zmp_cstr, N_variable);
  // std::cout << "ZMP_cstr_rows" << ZMP_Cstr.rows() << std::endl;

  Eigen::MatrixXd U_Cstr = Eigen::MatrixXd::Zero(N_u_cstr, N_variable);

  Eigen::VectorXd b_zmp = Eigen::VectorXd::Zero(ZMP_Cstr.rows());
  Eigen::VectorXd b_u = Eigen::VectorXd::Zero(U_Cstr.rows());

  create_cstr_matrices(ZMP_Cstr,b_zmp,zmp_cstr_polygons,b_zmp_ineq);
  create_cstr_matrices(U_Cstr,b_u,u_cstr_polygons,b_u_ineq);

  Eigen::MatrixXd u_Delta = Delta;
  u_Delta.block(0,0,2*m_C,2*m_C) = create_u_matrix();
  Eigen::MatrixXd DeltaNoDelay = Delta;
  DeltaNoDelay.block(0,0,2*m_C,2*m_C) = create_zmp_matrix(false);

  Aineq_zmp.resize(  ZMP_Cstr.rows(),N_variable);
  bineq_zmp.resize(Aineq_zmp.rows());

  Aineq_zmp << ZMP_Cstr * Delta;
  bineq_zmp << b_zmp;

  b_zmp_traj = Eigen::Map<Eigen::VectorXd>(ZMP_ref_traj.data(), ZMP_ref_traj.size());
  M_zmp_traj = Eigen::MatrixXd::Zero(b_zmp_traj.rows(), N_variable);
  M_zmp_traj.block(0, 0, b_zmp_traj.rows(), N_variable) = Delta_zmp_ref.block(0, 0, b_zmp_traj.rows(), N_variable);
  A_zmp = Delta.block(0,0,2 * m_C,N_variable);
  A_zmp.block(0,2 * m_C, 2 * m_C ,N_variable - 2* m_C).setZero();

  // time_span = std::chrono::high_resolution_clock::now() - t_clock;
  // mc_rtc::log::info("[ZMP cstr] matrix gen time {} ms", time_span.count());
  // std::cout << ("[ZMP cstr] matrix built") << std::endl;
}

void ISMPC_Solver::FootSteps_Constraints()
{
  std::vector<Eigen::VectorXd> b_kin_cstr_vec;
  std::vector<Eigen::MatrixX2d> kin_cstr_normals_vec;
  std::vector<Eigen::MatrixX2d> step_cstr_normals_vec;
  std::vector<Eigen::VectorXd> b_step_cstr_vec;
  Eigen::MatrixXd Delta = Eigen::MatrixXd::Identity(2 * j_Max_C, 2 * j_Max_C); // Matrix to differentiate two footsteps

  double l = m_feet_distance;
  if(m_support_foot == "LeftFoot"){l*=-1;}
  int N_footsteps_kin_cstr = 0;
  int N_footsteps_cstr = 0;
  for(int i = 0; i < j_Max_C; i++)
  {
    const double theta_i = mc_rbdyn::rpyFromMat(input_steps_[i].rotation()).z();
    sva::PTransformd & X_0_step_i = input_steps_[i];
    sva::PTransformd & X_0_step_im1 = X_0_support_foot;
    if(i != 0)
    {
      X_0_step_im1 = input_steps_[i - 1];
    }
    Eigen::Matrix3d R_Theta_i_0 = X_0_step_im1.rotation().transpose();

    Eigen::Vector3d offset =  R_Theta_i_0 * Eigen::Vector3d{0,l + (l/std::abs(l)) * m_dy_f/2,0};

    Rectangle Kinematic_Rectangle = Rectangle(theta_i, Eigen::Vector2d{m_dx_f, m_dy_f},offset);
    
  
    if(i > 0)
    {
      Delta.block(2*i,2*(i-1),2,2) = -Eigen::Matrix2d::Identity();
    }
    else
    {
      Kinematic_Rectangle = Rectangle(X_0_support_foot, Eigen::Vector2d{m_dx_f, m_dy_f}, offset);

      // Rectangle step_admissible_region_rect = Rectangle(X_0_step_i, Eigen::Vector2d{m_dx_f_rect, m_dy_f_rect});
      // SupportPolygon step_admissible_region_poly = SupportPolygon(step_admissible_region_rect);

      // step_cstr_normals_vec.push_back(step_admissible_region_poly.normals());
      // b_step_cstr_vec.push_back(step_admissible_region_poly.offsets());
      // N_footsteps_cstr += static_cast<int>(step_cstr_normals_vec.back().rows());
    }
    SupportPolygon Kinematic_Poly = SupportPolygon(Kinematic_Rectangle);
    b_kin_cstr_vec.push_back(Kinematic_Poly.offsets());
    kin_cstr_normals_vec.push_back(Kinematic_Poly.normals());

    N_footsteps_kin_cstr += static_cast<int>(kin_cstr_normals_vec.back().rows());
    l*=-1;
  }
  
  Eigen::MatrixXd foosteps_kin_cstr = Eigen::MatrixXd::Zero(N_footsteps_kin_cstr, 2 * (j_Max_C));
  Eigen::MatrixXd foosteps_cstr = Eigen::MatrixXd::Zero(N_footsteps_cstr, 2 * (j_Max_C));
  Eigen::VectorXd b_kin_cstr(N_footsteps_kin_cstr);
  Eigen::VectorXd b_steps_cstr(N_footsteps_cstr);
  Aineq_steps.resize(N_footsteps_kin_cstr + N_footsteps_cstr, N_variable);
  Aineq_steps.setZero();
  bineq_steps.resize(N_footsteps_kin_cstr + N_footsteps_cstr);
  bineq_steps.setZero();

  create_cstr_matrices(foosteps_kin_cstr,b_kin_cstr,kin_cstr_normals_vec,b_kin_cstr_vec);
  create_cstr_matrices(foosteps_cstr,b_steps_cstr,step_cstr_normals_vec,b_step_cstr_vec);

  Aineq_steps.block(0, 2 * m_C, N_footsteps_kin_cstr, 2 * j_Max_C) = foosteps_kin_cstr * Delta;
  bineq_steps.segment(0, N_footsteps_kin_cstr) = b_kin_cstr;
  // Aineq_steps.block(N_footsteps_kin_cstr, 2 * m_C, N_footsteps_cstr, 2 * j_Max_C) = foosteps_cstr;
  // bineq_steps.segment(N_footsteps_kin_cstr,N_footsteps_cstr) = b_steps_cstr;
}

void ISMPC_Solver::AntTailTrajectory()
{
  int PreviewSize = m_P - m_C;
  AfterTc_ZMP_trajectory;
  AfterTc_ZMP_trajectory.resize(2 * PreviewSize, 1);
  AfterTc_ZMP_trajectory.setZero();

  for(int i = 0; i < PreviewSize; i++)
  {

    double NextStepTiming(0);
    if(m_timestamp.size() != 0)
    {
      NextStepTiming = m_timestamp[j_f];
    }

    if(m_tk + static_cast<double>(m_C + i + 1) * m_delta > NextStepTiming)
    {
      if(N_Steps + j_f + 1 <= N_Steps_Desired || N_Steps_Desired < 0)
      {
        j_f += 1;
        if(j_f - 1 >= static_cast<int>(input_steps_.size()))
        {
          j_f -= 1;
          count_Dstep = (static_cast<double>(m_D) / 2) + 1;
        }
        else
        {
          j_fm1 = j_f - 1;
          count_Dstep = 1;
        }
      }
    }

    sva::PTransformd X_0_step_jm1 = X_0_swing_foot_initial;
    sva::PTransformd X_0_step_j = X_0_support_foot;

    if(j_f == 1)
    {
      X_0_step_j = input_steps_[j_f - 1];
      X_0_step_jm1 = X_0_support_foot;
    }
    else if(j_f > 1)
    {
      X_0_step_j = input_steps_[j_f - 1];
      X_0_step_jm1 = input_steps_[j_f - 2];
    }

    if(N_Steps + j_f == N_Steps_Desired)
    {
      X_0_step_j = sva::PTransformd(X_0_step_j.rotation(), (X_0_step_j.translation() + X_0_step_jm1.translation()) / 2);
    }
  
    int n = std::max(0., std::min( static_cast<double>(m_D) + 1., count_Dstep));

    double alpha = std::min(1.0,std::max(0., static_cast<double>(n) / (static_cast<double>(m_D))));


    Eigen::Vector3d StepZone = (X_0_step_j.translation() * alpha + X_0_step_jm1.translation() * (1 - alpha));

    AfterTc_ZMP_trajectory(i) = StepZone.x();
    AfterTc_ZMP_trajectory(i + PreviewSize) = StepZone.y();

    ZMP_max_ref_traj.push_back(R_0_support * StepZone);
    ZMP_min_ref_traj.push_back(R_0_support * StepZone);

    count_Dstep += 1;
    if(j_f - 1 == static_cast<int>(input_steps_.size()) && alpha > 0.5)
    {
      count_Dstep = static_cast<double>(m_D) / 2 + 1;
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

  A_stab.resize(2, N_variable);
  A_stab.setZero();
  b_stab = Eigen::VectorXd::Zero(2);
  Eigen::Vector3d u_delay = U_k - P_z_k;
  for(int j = 0; j < m_C; j++)
  {
    A_stab.block(0,2*j,2,2) = Eigen::Matrix2d::Identity() * (m_lambda/(m_lambda + m_eta)) * exp(-j * m_eta * m_delta);

  }
  A_stab *= exp(-m_eta * m_delay_elapsed);


  // if(m_Tail == "Periodic")
  // {
  //   b_stab(0) =
  //       m_eta * ((1 - exp(-m_eta * m_delta * m_C)) / (1 - exp(-m_eta * m_delta))) * (P_u_k.x() - (P_z_k.x() - w_k.x())*exp(-m_eta * m_delay));
  //   b_stab(1) =
  //       m_eta * ((1 - exp(-m_eta * m_delta * m_C)) / (1 - exp(-m_eta * m_delta))) * (P_u_k.y() - (P_z_k.y() - w_k.y())*exp(-m_eta * m_delay));
  // }
  // else if(m_Tail == "Truncated")
  // {
  //   b_stab(0) = P_u_k.x() - P_z_k.x()*exp(-m_eta * m_delay);
  //   b_stab(1) = P_u_k.y() - P_z_k.y()*exp(-m_eta * m_delay);
  // }
  // else
  // {
    // AntTailTrajectory();
    // int PreviewSize = (int)std::round(AfterTc_ZMP_trajectory.size() / 2);
    // Ant_Tail_X = 0;
    // Ant_Tail_Y = 0;
    // for(int k = 0; k < PreviewSize; k++)
    // {
    //   if(k < PreviewSize - 1)
    //   {
    //     Ant_Tail_X += exp(-(k + m_C) * m_eta * m_delta) * AfterTc_ZMP_velocity(k);
    //     Ant_Tail_Y += exp(-(k + m_C) * m_eta * m_delta) * AfterTc_ZMP_velocity(k + PreviewSize - 1);
    //   }
    // }

    // b_stab(0) = (m_eta / (1 - exp(-m_eta * m_delta))) * (P_u_k.x() - P_z_k.x() + w_k.x()) - Ant_Tail_X;
    // b_stab(1) = (m_eta / (1 - exp(-m_eta * m_delta))) * (P_u_k.y() - P_z_k.y() + w_k.y()) - Ant_Tail_Y;
    double l_d_w_p_e = (m_lambda/(m_lambda + m_eta)); 
    P_u_k = P_c_k + (V_c_k / m_eta);
    b_stab = ( P_u_k - ( (P_z_k_delayed - w_k) * exp(-m_eta * m_delay_elapsed)
                         + (P_z_k - w_k) + l_d_w_p_e * u_delay
                         - ( (P_z_k_delayed - w_k) + l_d_w_p_e * u_delay )* exp(-m_eta * m_delay_elapsed))
                         - w_k * exp(-m_eta * perturbation_duration)).segment(0,2) ;
    

    Eigen::Vector2d b_stan_alt = ( 
               P_u_k 
               - (1 - exp(-m_eta * m_delay_elapsed)) * (u_delay * l_d_w_p_e + (P_z_k - w_k))
               - (P_z_k_delayed - w_k )* exp(-m_eta * m_delay_elapsed)
               - w_k * exp(-m_eta * perturbation_duration) 
               ).segment(0,2);
    
    // mc_rtc::log::info(b_stab - b_stan_alt);
    b_stab = b_stan_alt;


  // }

  std::chrono::duration<double, std::milli> time_span = std::chrono::high_resolution_clock::now() - t_clock;
  double ProcessTime = time_span.count();
  // mc_rtc::log::success("Stability Constraints computed in : " + std::to_string(ProcessTime) + " ms");
}

void ISMPC_Solver::Compute_Stability_Range()
{
  P_u_k_min.setZero();
  P_u_k_max.setZero();

  Eigen::Vector3d PzM_k;
  Eigen::Vector3d u_m_k;
  Eigen::Vector3d u_M_k;

  Eigen::MatrixXd Delta; // Matrix to derive the ZMP position from u
  Eigen::VectorXd PzM = Eigen::VectorXd::Zero(2*m_C);
  Eigen::VectorXd Pzm = Eigen::VectorXd::Zero(2*m_C);
  Eigen::VectorXd Pz0 = Eigen::VectorXd::Zero(2*m_C);
  Delta = create_zmp_matrix(true);

  //mc_rtc::log::info("ZMP boundrie size {}\nControl size {}",ZMP_max_ref_traj.size(),m_C);
  for(size_t k = 1; k < ZMP_max_ref_traj.size(); k++)
  {
    Pzm.segment(2*k,2) = ZMP_min_ref_traj[k];
    PzM.segment(2*k,2) = ZMP_max_ref_traj[k];
    Pz0.segment(2*k,2) = P_z_k.segment(0,2);
  }
  Eigen::VectorXd u_M = Delta.inverse() * (PzM - Pz0);
  Eigen::VectorXd u_m = Delta.inverse() * (Pzm - Pz0);
  P_u_k_max.segment(0,2) = A_stab * u_M  + P_z_k.segment(0,2);
  P_u_k_min.segment(0,2) = A_stab * u_m  + P_z_k.segment(0,2);
  
}

void ISMPC_Solver::Compute_Standing_Stability_Range()
{
  Eigen::VectorXd offset = m_double_support_polygon.normals() * P_z_k.segment(0,2) * (1 - exp(-m_eta * 3 * m_delta)) + 
                           m_double_support_polygon.offsets() * exp(-m_eta * 3 * m_delta);
  m_feasibility_standing_region = SupportPolygon(m_double_support_polygon.normals(),offset);
  // for(auto & p : m_feasibility_standing_region.Get_Polygone_Corners())
  // {
  //   mc_rtc::log::info(p);
  // }
}

void ISMPC_Solver::Compute_Integration_Vector(int i)
{
  double l_p_w = (m_lambda + m_eta);
  double l_m_w = (m_lambda - m_eta);
  double com_param = 0.5 * m_eta * ( (1/l_p_w) * (exp((-l_p_w*i + (i+1) * m_eta)*m_delta_control) - exp((-l_p_w + m_eta) * (i+1) * m_delta_control) ));
  com_param -= 0.5 * m_eta * ( (1/l_m_w) * (exp((-l_m_w*i - (i+1) * m_eta)*m_delta_control) - exp((-l_m_w - m_eta) * (i+1) * m_delta_control) ));
  com_param += 1 - cosh(m_eta * m_delta_control);

  double comvel_param = 0.5 * (std::pow(m_eta,2)) * ( (1/l_p_w) * (exp((-l_p_w*i + (i+1) * m_eta)*m_delta_control) - exp((-l_p_w + m_eta) * (i+1) * m_delta_control) ));
  comvel_param += 0.5 * (std::pow(m_eta,2)) * ( (1/l_m_w) * (exp((-l_m_w*i - (i+1) * m_eta)*m_delta_control) - exp((-l_m_w - m_eta) * (i+1) * m_delta_control) ));
  comvel_param += - m_eta * sinh(m_eta * m_delta_control);

  Integration_Vec = Eigen::Vector3d{com_param , comvel_param, 1 - exp(-m_lambda * (i*m_delta_control))};
}

void ISMPC_Solver::Integrate()
{
  
  m_X_MPC.clear();
  m_Y_MPC.clear();
  int N = (int)(m_delta / m_delta_control);
  int N_delay = static_cast<int>(m_delay_elapsed/m_delta_control);
  int N_perturbation = static_cast<int>(perturbation_duration/m_delta_control);


  Eigen::Vector3d state_x = Eigen::Vector3d{P_c_k.x(), V_c_k.x(), P_z_k.x() - w_k.x()};
  Eigen::Vector3d state_y = Eigen::Vector3d{P_c_k.y(), V_c_k.y(), P_z_k.y() - w_k.y()};

  Eigen::Vector3d Pzi = Eigen::Vector3d{state_x(2),state_y(2),0};

  for(int k = 1; k < N_delay + 1; k++)
  {
    Compute_Integration_Vector(k);
    state_x(2) = Pzi.x();
    state_y(2) = Pzi.y();

    state_x = Integration_Mat * state_x + Integration_Vec * (U_k - P_z_k - w_k).x();
    state_y = Integration_Mat * state_y + Integration_Vec * (U_k - P_z_k - w_k).y();
    
    m_X_MPC.push_back(state_x);
    m_Y_MPC.push_back(state_y);

  }

  
  m_admittance_targets.clear();
  for (Eigen::Index i = 0 ; i < m_C; i++)
  {
    double u_x = P_z_k_delayed.x() - w_k.x() - state_x(2);
    double u_y = P_z_k_delayed.y() - w_k.y() - state_y(2);
    for (Eigen::Index j = 0 ; j <= i ; j++)
    {
      u_x += m_ZMP_u(j);
      u_y += m_ZMP_u(j + m_C);
    }
    Pzi = Eigen::Vector3d{state_x(2),state_y(2),0};
    m_admittance_targets.push_back(Eigen::Vector3d{u_x,u_y,0} + Pzi - P_z_k_delayed + P_z_k);


    for (int k = 0; k < N  ; k ++)
    {    

      Compute_Integration_Vector(k+1);
      if( i * N + k > N_perturbation )
      {
        Pzi += w_k;
        N_perturbation = m_C * N;
      }

      state_x(2) = Pzi.x();
      state_y(2) = Pzi.y();

      state_x = Integration_Mat * state_x + Integration_Vec * u_x;
      state_y = Integration_Mat * state_y + Integration_Vec * u_y;
      
      m_X_MPC.push_back(state_x);
      m_Y_MPC.push_back(state_y);

    }
  }

  for(size_t i = 0; i < std::min( m_Y_MPC.size() , static_cast<size_t>(N_perturbation)); i++)
  {
    m_X_MPC[i].z() += w_k.x();
    m_Y_MPC[i].z() += w_k.y();
  }
}

bool ISMPC_Solver::GetWalkingParameters(bool stop)
{
  std::chrono::high_resolution_clock::time_point t_clock = std::chrono::high_resolution_clock::now();
 
  m_feasibilitySolver.configure(m_eta,
                              m_delta_control,
                              Eigen::Vector2d{0.15,0.4},
                              Eigen::Vector2d{0.55,4},
                              Eigen::Vector2d{0.7,5},
                              Eigen::Vector2d{m_dx_f,m_dy_f},
                              Eigen::Vector2d{m_dx * 0.7  , m_dy},
                              m_feet_distance,8);
  std::vector<sva::PTransformd> & stepsRef = corr_steps_.size() != 0 ? corr_steps_ : input_steps_;

  bool feas_res = m_feasibilitySolver.solve(m_tk,m_t_lift,DoubleSupport,P_u_k.segment(0,2),P_z_k.segment(0,2),
                                          m_support_foot,
                                          X_0_support_foot,
                                          X_0_swing_foot_initial,
                                          m_input_Tds,input_steps_,
                                          m_timestamp);
  
  std::vector<double> optimalTs = m_feasibilitySolver.get_optimal_steps_timings() ;
  std::vector<double> optimalTds = m_feasibilitySolver.get_optimal_steps_ds_duration() ;
  std::vector<sva::PTransformd> optimalPf = m_feasibilitySolver.get_optimal_footsteps();
  
  // mc_rtc::log::info("optimal t {}",timings);
  // mc_rtc::log::info("optimal tds {}",timings_ds);
  // if(optimalPf.size() != 0)
  // {
  //   mc_rtc::log::info("ref p {}",input_steps_[0].translation());
  //   mc_rtc::log::info("optimal p {}",optimalPf[0].translation());
  // }
  if(feas_res)
  {
    m_timestamp = optimalTs;
    if(DoubleSupport){ m_Tds = optimalTds[0];}
    else if(optimalTds.size() > 1){m_Tds = optimalTds[1];}
    input_steps_ = optimalPf;
    m_feasibility_region = m_feasibilitySolver.get_feasibility_region();
  }
  else
  {
    m_timestamp = optimalTs;
    if(DoubleSupport){ m_Tds = optimalTds[0];}
    else if(optimalTds.size() > 1){m_Tds = optimalTds[1];}
    mc_rtc::log::warning("[ISMPC {}] Step feasibility QP fail",m_t_global);
  }
  
  QPsuccess = false;
  InStabilityRange = false;
  m_stop = stop;

  
  double tc = m_tk + m_Tc;
  size_t tstep_indx = 0;

  j_Max_C = 0;
  if(m_timestamp.size() != 0)
  {
    while(tc > m_timestamp[tstep_indx])
    {
      tstep_indx += 1;

      if(tstep_indx > m_timestamp.size())
      {
        break;
      }
    }
  }
  j_Max_C = static_cast<int>(tstep_indx) ;
  j_f = 0;
  j_fm1 = j_f - 1;

  N_variable = 2 * (m_C + j_Max_C);

  m_D = static_cast<int>(m_Tds / m_delta) - Tds_offset;
  count_Dstep = (std::min((m_tk / m_delta) , static_cast<double>(m_D)));
  if(!DoubleSupport)
  {
    count_Dstep = static_cast<double>(m_D);
  }
  std::chrono::duration<double, std::milli> time_span = std::chrono::high_resolution_clock::now() - t_clock;

  // mc_rtc::log::info("countD {}, m_D {} ,t_k : {}; Tc : {} ; Ts {} ; Tds {} ; j_f_max : {}",count_Dstep,m_D,m_tk,
  // m_Tc,m_timestamp[0],m_Tds,j_Max_C); 
  // mc_rtc::log::info("m_C {}",m_C); t_clock = std::chrono::high_resolution_clock::now();
  if(m_stop)
  {
    Static_ZMP_Constraints();
    m_feasibility_standing_region = SupportPolygon( m_feasibilitySolver.get_feasibility_region());
    m_feasibility_standing_region_swing = SupportPolygon(m_feasibilitySolver.get_feasibility_region(X_0_swing_foot_initial,X_0_support_foot) );
    // Compute_Standing_Stability_Range();
  }
  else
  {
    ZMP_Constraints();
  }
  // time_span = std::chrono::high_resolution_clock::now() - t_clock;
  // mc_rtc::log::info("ZMP cstr time {} ms",time_span.count());

  // t_clock = std::chrono::high_resolution_clock::now();
  FootSteps_Constraints();
  // time_span = std::chrono::high_resolution_clock::now() - t_clock;
  // mc_rtc::log::info("Steps cstr time {} ms",time_span.count());

  // t_clock = std::chrono::high_resolution_clock::now();
  Stability_Constraints();
  // time_span = std::chrono::high_resolution_clock::now() - t_clock;
  // mc_rtc::log::info("Stab cstr time {} ms",time_span.count());
  Compute_Stability_Range();

  if(!ComputeTrajectory)
  {
    return false;
  }

 
  Eigen::MatrixXd M_u = Eigen::MatrixXd::Zero(2*m_C, N_variable);
  // M_u.block(0, 0, 2 * m_C, 2 * m_C) =  (create_u_matrix() - create_zmp_matrix());
  Eigen::VectorXd b_u = Eigen::VectorXd::Zero(M_u.rows());
  M_u.block(0, 0, 2 * m_C, 2 * m_C) = Eigen::MatrixXd::Identity(2 * m_C , 2 * m_C);

  Eigen::MatrixXd M_steps = Eigen::MatrixXd::Zero(2*j_Max_C, N_variable);
  M_steps.block(0, 2 * m_C, 2 * j_Max_C, 2 * j_Max_C) = Eigen::MatrixXd::Identity(2 * j_Max_C, 2 * j_Max_C);
  
  Eigen::VectorXd b_steps = Eigen::VectorXd::Zero(2*j_Max_C);

  for(int i = 0; i < j_Max_C; i++)
  {
    b_steps.segment(2 * i, 2) = input_steps_[i].translation().segment(0, 2);
  }

  // t_clock = std::chrono::high_resolution_clock::now();

  m_Q = Eigen::MatrixXd::Identity(N_variable, N_variable) * 1e-12 + 
         m_Beta_u*(M_u.transpose() * M_u) + 
         m_Beta_step * (M_steps.transpose() * M_steps) + 
         m_Beta_traj * (M_zmp_traj.transpose() * M_zmp_traj);
         
  m_p = m_Beta_u*(-M_u.transpose() * b_u) + 
        m_Beta_step * (-M_steps.transpose() * b_steps) + 
        m_Beta_traj * (-M_zmp_traj.transpose() * b_zmp_traj);
     


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
  stab_error = (A_stab * QP_Output - b_stab).block(0,0,2,1);
  
  // std::cout << "QP out " << QP_Output << std::endl;

  Eigen::VectorXd zmp_vel_ = QP_Output.segment(0, 2 * m_C);
  // mc_rtc::log::info(zmp_vel_);
  if(!(((zmp_vel_ - zmp_vel_).array() == (zmp_vel_ - zmp_vel_).array()).all()))
  {

    mc_rtc::log::warning("[ISMPC] nan");
    QPsuccess = false;
    return true;
  }

  // time_span = std::chrono::high_resolution_clock::now() - t_clock;
  // mc_rtc::log::success("ZMPvel QP computed in {} ms ",time_span.count());

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
    stab_error = (A_stab * QP_Output - b_stab);
    mc_rtc::log::warning("[ISMPC] New stab error {}",stab_error);
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

    corr_steps_.clear();

    m_QP_zmp = (A_zmp * QP_Output).segment(0,2 * m_C);
    m_ZMP_u.resize(2 * m_C, 1);
    for(int k = 0; k < m_C; k++)
    {
      m_ZMP_u(k) = QP_Output(2 * k);
      m_ZMP_u(k + m_C) = QP_Output(2 * k + 1);
    }

    for(int k = 0; k < j_Max_C; k++)
    {
      if(AutoFootstepPlacement)
      {
        double xf = QP_Output(2 * m_C + 2 * k);
        double yf = QP_Output(2 * m_C + 2 * k + 1);

        corr_steps_.push_back(sva::PTransformd(input_steps_[k].rotation(), Eigen::Vector3d{xf, yf, 0}));
      }
      else
      {
        corr_steps_.push_back(input_steps_[k]);
      }
    }

    if(m_Tail == "None" || Use_Stability_Task)
    {
 

      Eigen::Vector2d P_u_k_2 = P_u_k.segment(0,2) + stab_error;
      V_c_k.segment(0,2) = m_eta * (P_u_k_2 - P_c_k.segment(0,2));
      // P_c_k.segment(0,2) = P_u_k_2 - P_c_k.segment(0,2)/m_eta;
      
      Eigen::Vector2d P_u_error = P_u_k.segment(0,2) - P_u_k_2; 
      
      // mc_rtc::log::info("P_u_error \n{}",P_u_error);
    }

    Integrate();
  }
  return true;
}

Eigen::VectorXd ISMPC_Solver::solveQP()
{

  int Nvar = static_cast<int>(m_Q.rows());
  int NIneqConstr = static_cast<int>(Aineq.rows());
  int NEqConstr = static_cast<int>(Aeq.rows());
  // QP.tolerance(1e-3);
  QP.problem(Nvar, NEqConstr, NIneqConstr);
  QPsuccess = QP.solve(m_Q, m_p, Aeq, beq, Aineq, bineq);

  return QP.result();
}
