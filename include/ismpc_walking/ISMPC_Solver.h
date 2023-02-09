#pragma once
#include <mc_control/api.h>
#include <mc_control/mc_controller.h>
#include "ControllerConfiguration.h"
#include "MPC_state.h"
#include "eigen-quadprog/QuadProg.h"
#include "eigen-quadprog/eigen_quadprog_api.h"
#include <Eigen/StdVector>
#include <eigen3/Eigen/Dense>
#include <thread>

struct Rectangle
{

public:
  Rectangle(double ori, const Eigen::Vector2d & size, const Eigen::Vector3d & offset = Eigen::Vector3d::Zero())
  {
    _center = offset;
    _angle = ori;
    _size.segment(0, 2) = size;
    compute_rect();
  }

  Rectangle(const sva::PTransformd & pose,
            const Eigen::Vector2d & size,
            const Eigen::Vector3d & offset = Eigen::Vector3d::Zero())
  {
    _center = pose.translation() + offset;
    _center.z() = 0;
    _angle = mc_rbdyn::rpyFromMat(pose.rotation()).z();
    _size.segment(0, 2) = size;
    compute_rect();
  }

  Rectangle(const Eigen::Vector3d & center,
            const Eigen::Vector2d & size,
            const Eigen::Vector3d & offset = Eigen::Vector3d::Zero())
  {
    _center = center + offset;
    _angle = _center.z();
    _size.segment(0, 2) = size;
    _center.z() = 0;
    compute_rect();
  }
  void compute_rect()
  {

    R << cos(_angle), -sin(_angle), 0, sin(_angle), cos(_angle), 0, 0, 0, 1;

    upper_left_corner = _center + R * Eigen::Vector3d{-_size.x() / 2, _size.y() / 2, 0};
    upper_right_corner = _center + R * Eigen::Vector3d{_size.x() / 2, _size.y() / 2, 0};
    lower_left_corner = _center + R * Eigen::Vector3d{-_size.x() / 2, -_size.y() / 2, 0};
    lower_right_corner = _center + R * Eigen::Vector3d{_size.x() / 2, -_size.y() / 2, 0};
    corners = {upper_left_corner, upper_right_corner, lower_right_corner, lower_left_corner};
  }
  void add_offset(const Eigen::Vector3d offset)
  {
    _center += Eigen::Vector3d{offset.x(), offset.y(), 0};
    compute_rect();
  }
  ~Rectangle()
  {
    corners.clear();
    _center.setZero();
    _size.setZero();
    _angle = 0;;
    R = Eigen::Matrix3d::Identity();
    upper_left_corner.setZero();
    upper_right_corner.setZero();
    lower_left_corner.setZero();
    lower_right_corner.setZero();
  }

  std::vector<Eigen::Vector3d> & Get_corners()
  {
    return corners;
    // return {upper_left_corner,lower_left_corner,lower_right_corner,upper_right_corner};
  }
  const Eigen::Vector3d & Up_Left_corner() const noexcept
  {
    return upper_left_corner;
  }
  const Eigen::Vector3d & Up_Right_corner() const noexcept
  {
    return upper_right_corner;
  }
  const Eigen::Vector3d & Dwn_Right_corner() const noexcept
  {
    return lower_right_corner;
  }
  const Eigen::Vector3d & Dwn_Left_corner() const noexcept
  {
    return lower_left_corner;
  }

  double get_yaw()
  {
    return _angle;
  }
  Eigen::Vector3d get_center()
  {
    return _center;
  }

private:
  Eigen::Vector3d _center;
  Eigen::Vector3d _size;
  double _angle;
  Eigen::Matrix3d R;
  Eigen::Vector3d upper_left_corner;
  Eigen::Vector3d upper_right_corner;
  Eigen::Vector3d lower_left_corner;
  Eigen::Vector3d lower_right_corner;
  std::vector<Eigen::Vector3d> corners;
};

struct vec3d_x_comp
{
  inline bool operator()(const Eigen::Vector3d & struct1, const Eigen::Vector3d & struct2)
  {
    return (struct1.x() < struct2.x());
  }
};

struct SupportPolygon
{

public:
  SupportPolygon() = default;
  SupportPolygon(const Rectangle Rect1, const Rectangle Rect2)
  {
    _Rectangles = {Rect1, Rect2};
    Compute_polygone();
  }
  SupportPolygon(const Rectangle Rect1)
  {
    _Rectangles = {Rect1};
    Compute_polygone();
  }
  SupportPolygon(const Eigen::MatrixX2d normals, Eigen::VectorXd offsets)
  {
    SupportPolygone_Normals = normals;
    Offset = offsets;
    cstr_to_polygone();
  }

  ~SupportPolygon()
  {
    _Rectangles.clear();
    _corners.clear();
    SupportPolygone_Corners.clear();
  }

  void jarvis_march();
  void convex_hull();

  std::vector<Eigen::Vector3d> & Get_Polygone_Corners()
  {
    return SupportPolygone_Corners;
  }

  const Eigen::MatrixX2d & normals()
  {
    return SupportPolygone_Normals;
  }

  const Eigen::VectorXd & offsets()
  {
    return Offset;
  }

  Eigen::Vector3d get_center()
  {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double n = static_cast<double>(_Rectangles.size());
    for (auto & r : _Rectangles)
    {
      center += (1/n) * r.get_center();
    }
    return center;
  }

  Rectangle & get_Rectangle(int indx)
  {
    if(indx < _Rectangles.size() - 1)
    {
      return _Rectangles[indx];
    }
    else
    {
      return _Rectangles[0];
    }
  }

private:
  void Compute_polygone()
  {
    if(_Rectangles.size() > 1)
    {
      // for (int r = 0 ; r < _Rectangles.size() ; r++)
      // {
      //     std::vector<Eigen::Vector3d> corners = _Rectangles[r].Get_corners();
      //     _corners.insert(_corners.end(),corners.begin(),corners.end());
      // }
      // jarvis_march();
      convex_hull();
    }
    else
    {
      SupportPolygone_Corners = _Rectangles[0].Get_corners();
    }
    SupportPolygone_Normals.resize(SupportPolygone_Corners.size(), 2);
    SupportPolygone_Edges_Center.resize(SupportPolygone_Corners.size(), 2);
    SupportPolygone_Vertices.resize(SupportPolygone_Corners.size(), 2);
    Offset.resize(SupportPolygone_Corners.size());
    Eigen::Matrix2d R_Vertices_0;

    for(size_t c = 0; c < SupportPolygone_Corners.size(); c++)
    {
      const Eigen::Vector3d & point_1 = SupportPolygone_Corners[c];
      const Eigen::Vector3d & point_2 = SupportPolygone_Corners[(c + 1) % SupportPolygone_Corners.size()];
      const Eigen::Vector3d vertice = (point_2 - point_1).normalized();
      const Eigen::Vector3d normal = vertical_vec.cross(vertice);
      SupportPolygone_Normals(c, 0) = normal.x();
      SupportPolygone_Normals(c, 1) = normal.y();
      SupportPolygone_Vertices(c, 0) = vertice.x();
      SupportPolygone_Vertices(c, 1) = vertice.y();
      SupportPolygone_Edges_Center(c, 0) = (((point_2 + point_1) / 2)).x();
      SupportPolygone_Edges_Center(c, 1) = (((point_2 + point_1) / 2)).y();

      R_Vertices_0 << SupportPolygone_Normals(c, 0), SupportPolygone_Vertices(c, 0), SupportPolygone_Normals(c, 1),
          SupportPolygone_Vertices(c, 1);

      Offset(c) = (R_Vertices_0.transpose() * SupportPolygone_Edges_Center.block(c, 0, 1, 2).transpose())(0);
    }
  }
  
  void cstr_to_polygone()
  {

    std::vector<int> vertices_indx;
    for(size_t i = 0 ; i < SupportPolygone_Normals.rows() ; i++)
    {
      int end_indx = (i + 1) % SupportPolygone_Normals.rows();
      Eigen::Vector2d ni = SupportPolygone_Normals.block(i,0,1,2).normalized();
      Eigen::Vector2d nip1 = SupportPolygone_Normals.block(end_indx,0,1,2).normalized();
      // mc_rtc::log::info("normal {}\n{}\nnext_normal{}\ndot prod {}",i,ni,nip1,ni.transpose() * nip1);
      if( std::abs( ni.transpose() * nip1 - 1) > 1e-4  )
      {

        vertices_indx.push_back(i);
        // mc_rtc::log::info("selected");
      }
    }
    // mc_rtc::log::info("corner {} selected {}",SupportPolygone_Normals.rows(),normals.size());
    for(size_t i = 0 ; i < vertices_indx.size() ; i++)
    {
      int start_indx = vertices_indx[i];
      int end_indx = vertices_indx[(i + 1) % vertices_indx.size()];
      Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
      Eigen::Vector2d o = Eigen::Vector2d::Zero(); 
      R.block(0,0,1,2) = SupportPolygone_Normals.block(start_indx,0,1,2);
      R.block(1,0,1,2) = SupportPolygone_Normals.block(end_indx,0,1,2);
      o.x() = Offset[start_indx];
      o.y() = Offset[end_indx];
      if(R.determinant() != 0)
      {
        Eigen::Vector2d p = R.inverse() * o;
        // mc_rtc::log::info("start {} ; end {} point {}",i,end_indx,p);
        // mc_rtc::log::info("R {}\no {}",R,o);
  
        SupportPolygone_Corners.push_back(Eigen::Vector3d{p.x(),p.y(),0});
      }
      
      
    
    }
  }

  Eigen::Vector3d vertical_vec = Eigen::Vector3d{0, 0, 1};
  std::vector<Rectangle> _Rectangles;
  std::vector<Eigen::Vector3d> _corners;
  std::vector<Eigen::Vector3d> SupportPolygone_Corners;
  Eigen::MatrixX2d SupportPolygone_Vertices;
  Eigen::MatrixX2d SupportPolygone_Edges_Center;
  Eigen::MatrixX2d SupportPolygone_Normals;
  Eigen::VectorXd Offset;
};

class ISMPC_Solver
{
public:
  ISMPC_Solver();

  /**
   * Initialize the fixed parameters of the MPC
   * @tparam delta_controller : controller timestep
   * @tparam delta : MPC timestep
   * @tparam Tp : Preview horizon lenght, horizon where footsteps and trajectory will be computed
   * @tparam Tc : Control horizon lenght, horizon where the CoM/ZMP trajectory will be computed (Tc < Tp)
   */
  ISMPC_Solver(double delta_controller, double delta, double Tp, double Tc);

  ~ISMPC_Solver() = default;
  /**
   * Initialize the fixed parameters of the MPC
   * @tparam delta_controller : controller timestep
   * @tparam delta : MPC timestep
   * @tparam Tp : Preview horizon lenght, horizon where footsteps and trajectory will be computed
   * @tparam Tc : Control horizon lenght, horizon where the CoM/ZMP trajectory will be computed (Tc < Tp)
   * @tparam Beta : Weight of the footsteps position in the QP cost function
   */
  void Init(double delta_controller, double delta, double Tp, double Tc, double Beta);

  void init_MPC(const MPC_state & mpc_state,
                const std::vector<sva::PTransformd> & steps,
                const std::vector<double> & timesstp,
                std::string Tail,
                int Steps_Desired,
                int Step);

  /**
   * Set the robot walking charateristics
   * @tparam Pck , Vck , Pzk : Initial CoM , CoMd and ZMP position
   * @tparam Pfm1 : Previous swing foot or actual support foot position
   * @tparam timesstp , timesindx : Steps timing and their indexes in the horizon
   * @tparam Tail , choice of the velocity tail (Truncated, Periodic, Anticipative or None)
   * @tparam Steps_Desired, steps choosen to be performed, the number of steps done must be updated manually with Steps
   */
  void SetWalkingParameters(const Eigen::Vector3d & Pck,
                            const Eigen::Vector3d & Vck,
                            const Eigen::Vector3d & Pzk,
                            const Eigen::Vector3d & Pfm1,
                            const std::vector<double> & timesstp,
                            std::string Tail,
                            int Steps_Desired,
                            int Steps);

  /**
   * Compute the CoM, CoMd, ZMP trajectory for previously set Walking parameters
   * @tparam PrevStepTime, previous footsteps timing
   * @tparam t_k, time of the computation
   * @tparam Tds,  double support duration
   */
  bool GetWalkingParameters(double t_k, bool stop);

  /**
   * @brief Set The constraints region for the ZMP (during each delta time) and the footsteps in the robot frame
   *
   * @param ZMP
   * @param FootSteps
   */
  void MPC_Constraints_region(Eigen::Vector2d ZMP, Eigen::Vector2d FootSteps)
  {
    m_dx = ZMP.x();
    m_dy = ZMP.y();
    m_dx_f = FootSteps.x();
    m_dy_f = FootSteps.y();
  }
  // void MPC_Constraints_region(Eigen::Vector2d ZMP){
  //     m_dx = ZMP.x() ; m_dy = ZMP.y();
  // }
  // void MPC_Constraints_region(Eigen::Vector2d FootSteps){
  //     m_dx_f = FootSteps.x() ; m_dy_f = FootSteps.y();
  // }
  Eigen::Vector2d ZMP_Constraints_Size()
  {
    return Eigen::Vector2d{m_dx, m_dy};
  }
  Eigen::Vector2d Footsteps_Constraints_Size()
  {
    return Eigen::Vector2d{m_dx_f, m_dy_f};
  }

  double delta_mpc() const noexcept
  {
    return m_delta;
  }
  double delta_control() const noexcept
  {
    return m_delta_control;
  }

  /**
   * Returns if previous MPC was feasible
   */
  const bool & QPsucceeded() const noexcept
  {
    return QPsuccess;
  }
  const std::string & Tail() const noexcept
  {
    return m_Tail;
  }

  const std::vector<sva::PTransformd> & inputs_steps() const noexcept
  {
    return input_steps_;
  }

  const std::vector<sva::PTransformd> & optimal_steps() const noexcept
  {
    return corr_steps_;
  }

  /**
   * Returns the computed trajectory, each vector3d in the vector contains the CoM , CoMd and ZMP value for a time step
   */
  const std::vector<Eigen::Vector3d> & X_MPC() const noexcept
  {
    return m_X_MPC;
  }
  /**
   * Returns the computed trajectory, each vector3d in the vector contains the CoM , CoMd and ZMP value for a time step
   */
  const std::vector<Eigen::Vector3d> & Y_MPC() const noexcept
  {
    return m_Y_MPC;
  }

  /**
   * Return the feasibility boundries in term of initial DCM in support foot frame
   */
  const Eigen::Vector3d & Puk_min()
  {
    return P_u_k_min;
  }

  /**
   * Return the feasibility boundries in term of initial DCM in support foot frame
   */
  const Eigen::Vector3d & Puk_max() const noexcept
  {
    return P_u_k_max; 
  }

  const Eigen::Matrix3d & Support_ori() const noexcept
  {
    return R_support_0;
  }

  const Eigen::Vector3d & Disturbance() const noexcept
  {
    return w_k;
  }

  void Disturbance(const Eigen::Vector3d w) noexcept
  {
    w_k = w/2;
  }

  /**
   * Returns the initial DCM used in the MPC in the world frame
   */
  const Eigen::Vector3d & Puk() const noexcept
  {
    return P_u_k; 
  }

  const Eigen::Vector3d & Uk()
  {
    return U_k;
  }

  const Eigen::Vector2d & stability_error() const noexcept
  {
    return stab_error;
  }

  const std::vector<Eigen::Vector3d> QP_zmp()
  {
    std::vector<Eigen::Vector3d> out;
    for (Eigen::Index i = 0 ; i < m_QP_zmp.size()/2 ; i++)
    {
      out.push_back(P_z_k_delayed + Eigen::Vector3d{m_QP_zmp(2*i),m_QP_zmp(2*i + 1) , 0});
    }
    return out;
  }

  /**
   * Set an initial DCM in the world frame
   */
  void Puk(Eigen::Vector3d puk)
  {
    P_u_k = puk ;
    P_u_k.z() = 0;
  }

  const Eigen::VectorXd & GetAfterTc_ZMP_trajectory()
  {
    return AfterTc_ZMP_trajectory;
  }

  const Eigen::VectorXd & ZMP_vel() const noexcept
  {
    return m_ZMP_u;
  }

  const Eigen::Vector3d & Initial_ZMP() const noexcept
  {
    return P_z_k;
  }

  const double get_lambda()
  {
    return m_lambda;
  }

  const double support_state()
  {
    return m_support_state;
  }

  Eigen::Vector3d zmp_ref() 
  {
    return m_ref_zmp;
  }

  void set_lambda(const double in)
  {
    m_lambda = in;
  }

  double zmp_delay()
  {
    return m_delay;
  }

  void zmp_delay(const double t)
  {
    m_delay = std::max(0.,std::min(m_delta,t));
  }

  std::vector<Eigen::Vector3d> feasibility_region()
  {
    if(m_stop)
    {
      return m_feasibility_standing_region.Get_Polygone_Corners();
    }
    else
    {
      Eigen::Vector3d p0 =  Puk_min();
      Eigen::Vector3d p2 =  Puk_max();
      Eigen::Vector3d p1 =
          p0 + R_support_0 * Eigen::Vector3d{ (R_0_support * (Puk_max() - Puk_min())).x(), 0, 0};
      Eigen::Vector3d p3 =
          p0 + R_support_0 * Eigen::Vector3d{ 0, (R_0_support * (Puk_max() - Puk_min())).y(), 0};
      return {p0, p1, p2, p3};
    }
  } 

  SupportPolygon & standing_feasibility_polygone()
  {
    return m_feasibility_standing_region;
  }

  const std::vector<Eigen::Vector3d> & get_polynome_support()
  {

    return SuppPolyCorners;
  }

  void configure(const ControllerConfiguration & config);

  void Allow_none(bool state)
  {
    Allow_None = state;
  }

  std::vector<Eigen::Vector3d> zmp_ref_traj()
  {
    std::vector<Eigen::Vector3d> Output;
    int n = static_cast<int>(b_zmp_traj.size() / 2);
    for(int i = 0; i < n; i++)
    {
      Output.push_back(Eigen::Vector3d{b_zmp_traj(2 * i), b_zmp_traj(2 * i + 1), 0} + P_z_k_delayed);
    }
    return Output;
  }
  const std::vector<Eigen::Vector3d> & admittance_references()
  {
    return m_admittance_targets;
  }

  bool AutoFootstepPlacement = false;

  std::vector<std::vector<Eigen::Vector3d>> All_poly;

  const std::vector<std::vector<Eigen::Vector3d>> & get_allpolys()
  {

    return All_poly;
  }



  bool stop()
  {
    return m_stop;
  }
  bool ComputeTrajectory = true;

private:
  /**
   * ZMP Trajectory constraints :
   * QP is build such as the output vector contains :
   * -The ZMP reference in x and y (world frame) ordered by timesteps then x then y
   * -The Optimized Footstep (if computed) in x and y ordered by timesteps then x then y
   * -The inequality constraints are set in the constraints matrix such as the first part represent the zmp position
   * constraints and then the Footsteps position constraints
   */
  void ZMP_Constraints();

  void ZMP_Transition_Constraint(Eigen::MatrixXd & A_out,Eigen::VectorXd & b_out,SupportPolygon PolySS);

  void Static_ZMP_Constraints();

  void Compute_Stability_Range();

  void Compute_Standing_Stability_Range();

  void FootSteps_Constraints();

  /**
   * Stability constraints are the QP equality constraints, first line is the X axis, second is the Y axis
   */
  void Stability_Constraints();


  void create_cstr_matrices(Eigen::MatrixXd & A_out, Eigen::VectorXd & b_out, std::vector<SupportPolygon> & A_in, const std::vector<Eigen::VectorXd> & b_in);

  void create_cstr_matrices(Eigen::MatrixXd & A_out, Eigen::VectorXd & b_out, std::vector<Eigen::MatrixX2d> & A_in, const std::vector<Eigen::VectorXd> & b_in);
  
  Eigen::MatrixXd create_zmp_matrix();
  Eigen::MatrixXd create_u_matrix();

  /**
   * Integrate The ZMP velocity to compute the CoM, CoMd and ZMP trajectory
   */
  void Integrate();

  void Compute_Integration_Vector(int i);

  /**
   * Generate a ZMP trajectory that is the middle point of the zmp square constraints between the preview and control
   * horizon. Trajectory is computed in terms of ZMP velocity
   */
  void AntTailTrajectory();

  Eigen::VectorXd solveQP();

  Eigen::Vector3d P_z_k; // Initial ZMP position
  Eigen::Vector3d P_z_k_delayed; //ZMP pose after input U_k during input delay
  Eigen::Vector3d P_c_k; // Initial CoM Position
  Eigen::Vector3d V_c_k; // Initial CoM Velocity
  Eigen::Vector3d P_u_k; // Initial Unstable Component/DCM
  Eigen::Vector3d U_k; //Current input acting on the pendulum
  Eigen::Vector3d m_Pfm1; // Swing Foot Pose Before Swinging orientation in z
  Eigen::Vector3d w_k; // Perturbance


  Eigen::Matrix3d R_support_0 = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R_0_support = Eigen::Matrix3d::Identity();
  Eigen::VectorXd m_ZMP_u; // Computed ZMP velocity in world frame
  std::vector<double> m_timestamp; // Step TimesStamp Computed at the footStep Generation

  sva::PTransformd X_0_support_foot;
  sva::PTransformd X_0_swing_foot_initial;
  std::vector<sva::PTransformd> input_steps_;
  std::vector<sva::PTransformd> corr_steps_;

  Eigen::Vector3d P_u_k_min; // Min initial DCM coordinates in support Foot Frame
  Eigen::Vector3d P_u_k_max; // Max initial DCM coordinates in support Foot Frame

  Eigen::VectorXd QP_Output;

  std::vector<Eigen::Vector3d> m_X_MPC; // Integrated CoM, CoMd, ZMP trajectory in world frame
  std::vector<Eigen::Vector3d> m_Y_MPC; // Integrated CoM, CoMd, ZMP trajectory in world frame

  double Ant_Tail_X = 0.0;
  double Ant_Tail_Y = 0.0;

  int N_Steps_Desired = -1;
  int N_Steps = 0;

  bool QPsuccess = false;
  Eigen::Vector2d stab_error = Eigen::Vector2d::Zero();
  Eigen::VectorXd m_QP_zmp;
  std::vector<Eigen::Vector3d> m_admittance_targets;
  bool Use_Stability_Task = false;
  bool Allow_None = true;
  bool InStabilityRange = false;
  bool m_stop = true;

  /**
   *Only during the first double support phase : If enabled, the admissible region is a sliding square,
   *otherwise it is a polygone defined by two  rectangle on both feets.
   */
  bool Slide_ZMP_region = false;

  double m_eta; // Prendulum frequency
  double CoM_height = 0.78;
  double g = 9.8; // Gravity acceleration
  double m_tk; //Represent the initial time in the MPC horizon
  double m_t_global; //Global time of the control scheme
  double m_Tc;
  double m_Tp; // Control & Preview horizon time
  double m_Tds; // Double Support Duration
  int Tds_offset = 0;
  double m_Dstep_ratio; // T_DoubleStep/T_Step
  double m_delta; // t_k - t_k-1
  double m_delta_control; // Controller timestep
  double N_integration = 1;
  double m_dx_static;
  double m_dy_static;
  double m_dx;
  double m_dy; // ZMP square size at one timestep
  double m_dx_u;
  double m_dy_u; // ZMP square size at one timestep
  Eigen::Vector2d rect_pose_offset; // cstr zone offset in the foot frame for y axis, positive offset is an offset
                                    // toward the other feet;
  Eigen::Vector2d rect_pose_offset_static; // cstr zone offset in the foot frame for y axis, positive offset is an offset
                                    // toward the other feet;

  Eigen::Vector2d zmp_ref_offset;          
  
  double zmp_cstr_next_stp_ratio = 2;
  double m_dx_f;
  double m_dy_f; // Step kinematic admissible Region
  double m_dx_f_rect;
  double m_dy_f_rect; // Step admissible region
  double m_Beta_u = 1;
  double m_Beta_step = 1e1;
  double m_Beta_stab = 1e5;
  double m_Beta_traj = 0.;
  double m_lambda = 100;
  double m_delay = 0; //delay ( < m_delta ) during which zmp is under previous input Uk
  double m_delay_elapsed = 0; //Between 0 and m_delay represent the remaining time the delay must be applied
  double m_t_delay = 0; // represent when the delay has been applied

  double m_feet_distance = 0.2; 
  std::string m_support_foot = "LeftFoot";
  int j_Max_C = 0; // Number of footsteps in the Control Horizon
  int j_f; // Index of the actual support foot
  int j_fm1; // Index of the previous support foot
  double m_support_state = 0;
  Eigen::Vector3d m_ref_zmp = Eigen::Vector3d::Zero(); //first ref zmp in the horizon
  int kfoot = 0;

  std::string m_Tail; // Velocity Tailing desired
  std::string m_Tail_save; // Save of the desired Velocity Tailing

  Eigen::VectorXd AfterTc_ZMP_velocity; // velocity generated by the midpoint between the ZMP constraints after Tc
  Eigen::VectorXd AfterTc_ZMP_trajectory;

  std::vector<double> ZMP_ref_traj;
  std::vector<Eigen::Vector3d> ZMP_min_ref_traj;
  std::vector<Eigen::Vector3d> ZMP_max_ref_traj;
  Eigen::MatrixXd M_zmp_traj;
  Eigen::VectorXd b_zmp_traj;

  // CoM,CoMd,ZMP Integration
  Eigen::Matrix3d Integration_Mat;
  Eigen::Vector3d Integration_Vec;

  //Pendulum dynamic to integrate state : \dot{s} = A * s + B * U 
  Eigen::Matrix3d m_dynamic_matrix_A;
  Eigen::MatrixXd m_dynamic_matrix_B;

  Eigen::VectorXd Pzk_Offset; // Vector that represent the intial position of the ZMP
  Eigen::MatrixXd C; // Temporary matrix to compute ZMP constraints with footsteps placements
  // Eigen::MatrixXd Delta; //Matrix to derive the ZMP position to ZMP velocity

  int m_C; // Number of indexs in the Control time length Tc = m_C * m_delta
  int m_P; // Number of indexs in the Preview time length Tp = m_P * m_delta
  int m_D; // Number of Iteration on the double steps period
  double count_Dstep; // Number bounded between 1 and m_D describing the position of the zone during the doubleStep timing

  std::vector<SupportPolygon> zmp_cstr_polygons;

  SupportPolygon m_double_support_polygon;
  SupportPolygon m_feasibility_standing_region;

  std::vector<Eigen::Vector3d> SuppPolyCorners;

  Eigen::MatrixXd Aineq_zmp; // Inequality ZMP Matrix
  Eigen::VectorXd bineq_zmp; // Inequality ZMP Vector

  Eigen::MatrixXd A_stab; // Equality stability cstr matrix
  Eigen::VectorXd b_stab; // Equality stability cstr vector

  Eigen::MatrixXd Aineq_steps; // Inequality Steps Matrix
  Eigen::VectorXd bineq_steps; // Inequality Steps Vector

  Eigen::MatrixXd A_zmp; // Matrix that computes the zmp from the QP output;

  // QP Problem
  Eigen::QuadProgDense QP;

  int N_variable;
  Eigen::MatrixXd m_Q; // QP Hessian
  Eigen::VectorXd m_p; // QP Grad
  Eigen::MatrixXd m_G; // QP constraints Matrix

  Eigen::MatrixXd Aeq; // Equality Matrix
  Eigen::VectorXd beq; // Equality Vector

  Eigen::MatrixXd Aineq; // Inequality Matrix
  Eigen::VectorXd bineq; // Inequality Vector
};
