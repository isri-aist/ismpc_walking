#pragma once
#include <eigen3/Eigen/Dense>
#include <vector>

struct MPC_state
{

  Eigen::Vector3d Get_CoM_planarTarget(int indx)
  {
    if(indx < X_MPC.size())
    {
      return Eigen::Vector3d{X_MPC[indx][0], Y_MPC[indx][0], 0};
    }
    std::cout << "[CoM access] Warning wrong index returning 0 vector" << std::endl;
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d Get_CoMVel_planarTarget(int indx)
  {
    if(indx < X_MPC.size())
    {
      return Eigen::Vector3d{X_MPC[indx][1], Y_MPC[indx][1], 0};
    }
    std::cout << "[CoMd access] Warning wrong index returning 0 vector" << std::endl;
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d Get_ZMP_planarTarget(int indx)
  {
    if(indx < X_MPC.size())
    {
      return Eigen::Vector3d{X_MPC[indx][2], Y_MPC[indx][2], 0};
    }
    std::cout << "[ZMP access] Warning wrong index returning 0 vector" << std::endl;
    return Eigen::Vector3d::Zero();
  }

  sva::PTransformd & Get_CorrectedFootstep(int indx)
  {

    return opti_steps[indx];
  }

  sva::PTransformd & Get_PlannedFootstep(int indx)
  {

    return planned_steps_[indx];
  }

  const std::vector<sva::PTransformd> & planned_steps() const noexcept
  {
    return planned_steps_;
  }

  const std::vector<sva::PTransformd> & optimal_steps() const noexcept
  {
    return opti_steps;
  }

  const std::vector<Eigen::Vector3d> & get_SupPolygon()
  {
    return SupPolygon;
  }
  const Eigen::VectorXd & get_Trajant()
  {
    return Traj_ant;
  }
  const std::vector<Eigen::Vector3d> & get_RefTraj()
  {
    return P_traj;
  }
  const std::vector<int> & getTimeStampIndex()
  {
    return TimeStampsIndex;
  }
  const std::vector<double> & getTimeStamp()
  {
    return TimeStamps;
  }
  double get_Ts(int indx)
  {
    return TimeStamps[indx];
  }
  double get_tds()
  {
    return tds;
  }
  void set_input_tds(double t)
  {
    input_tds = t;
  }

  const Eigen::Vector3d & getPzk()
  {
    return Pzk;
  }
  const Eigen::Vector3d & getPck()
  {
    return Pck;
  }
  const Eigen::Vector3d & getVck()
  {
    return Vck;
  }


  Eigen::Vector3d get_u(int indx)
  {
    double horizon_size = static_cast<double>(mpc_u_.size()) / 2;
    return Eigen::Vector3d{mpc_u_[indx] , mpc_u_[indx + static_cast<int>(horizon_size)],0.};
  }

  std::vector<Eigen::Vector3d>
      X_MPC; // Contain 3d vectors that represents in that order the CoM the CoMd and the ZMP for each timestep
  std::vector<Eigen::Vector3d>
      Y_MPC; // Contain 3d vectors that represents in that order the CoM the CoMd and the ZMP for each timestep
  std::vector<Eigen::Vector3d> SupPolygon;
  std::vector<Eigen::Vector3d> FeasibilityPolygon;
  Eigen::VectorXd Traj_ant;
  std::vector<Eigen::Vector3d> P_traj; // Vector containing the reference trajectory
  std::vector<int> TimeStampsIndex; // Index of the timing of each step
  std::vector<double> TimeStamps; // Timing of each step
  bool Tail = true;
  int kfoot = 0;
  double stab_error;
  bool QPSuccess;
  Eigen::Vector2d Pu_min;
  Eigen::Vector2d Pu_max;
  int Index = 0;

  Eigen::Vector3d initial_zmp_ = Eigen::Vector3d::Zero();

  double t_k = 0;
  Eigen::Vector3d Pck;
  Eigen::Vector3d Vck;
  Eigen::Vector3d Pzk;
  Eigen::Vector3d Pu;
  Eigen::Vector3d w; // Perturbation
  Eigen::VectorXd mpc_u_;
  std::vector<sva::MotionVecd> input_v_;
  std::vector<sva::PTransformd> input_steps_;
  std::vector<sva::PTransformd> planned_steps_;
  std::vector<double> input_timesteps_; // Input desired steps timings
  std::vector<sva::PTransformd> opti_steps;
  std::string input_Support_FootName;
  Eigen::Vector3d input_P_fm1;
  Eigen::Vector3d SupportFootPose;
  sva::PTransformd X_0_SupportFoot;
  sva::PTransformd X_0_Initial_SwingFoot;
  
  double tds = 0.25;
  double input_tds = 0.25;
  bool stop = true;
};
