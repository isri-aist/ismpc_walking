#pragma once
#include <eigen3/Eigen/Dense>
#include <vector>
#include "Polygon.h"

struct MPC_state
{

  Eigen::Vector3d Get_CoM_planarTarget(const size_t indx)
  {
    if(indx < X_MPC.size())
    {
      return Eigen::Vector3d{X_MPC[indx][0], Y_MPC[indx][0], 0};
    }
    std::cout << "[CoM access] Warning wrong index returning 0 vector" << std::endl;
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d Get_CoMVel_planarTarget(const size_t indx)
  {
    if(indx < X_MPC.size())
    {
      return Eigen::Vector3d{X_MPC[indx][1], Y_MPC[indx][1], 0};
    }
    std::cout << "[CoMd access] Warning wrong index returning 0 vector" << std::endl;
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d Get_ZMP_planarTarget(const size_t indx)
  {
    if(indx < X_MPC.size())
    {
      return Eigen::Vector3d{X_MPC[indx][2], Y_MPC[indx][2], 0};
    }
    std::cout << "[ZMP access] Warning wrong index returning 0 vector" << std::endl;
    return Eigen::Vector3d::Zero();
  }

  std::vector<Eigen::Vector2d> zmp_references()
  {
    std::vector<Eigen::Vector2d> output;
    // for (int indx = Index ; indx < X_MPC.size() ; indx++)
    // {
    //   output.push_back(Eigen::Vector2d{X_MPC[indx][2], Y_MPC[indx][2]});
    // }
    for (auto & ref : QP_zmp)
    {
      output.push_back(ref.segment(0,2));
    }
    return output; 
  }

  sva::PTransformd & Get_CorrectedFootstep(int indx)
  {

    return optimal_steps_[indx];
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
    return optimal_steps_;
  }

  std::vector<Eigen::Vector2d> admittance_references()
  {
    std::vector<Eigen::Vector2d> output;
    for (auto & ref : admittance_ref_)
    {
      output.push_back(ref.segment(0,2));
    }
    return output;
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

  const std::vector<double> & getTimeStamp()
  {
    return optimal_timesteps_;
  }
  double get_Ts(size_t indx)
  {
    if (indx < optimal_timesteps_.size())
    {
      return optimal_timesteps_[indx];
    }
    return 1.;
  }
  double get_tds()
  {
    return optimal_tds;
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
    if(indx/2 >= mpc_u_.size())
    {
      std::cout << "[U access] Warning wrong index returning 0 vector" << std::endl;
      return Eigen::Vector3d::Zero();
    }
    double horizon_size = static_cast<double>(mpc_u_.size()) / 2;
    return Eigen::Vector3d{mpc_u_[indx] , mpc_u_[indx + static_cast<int>(horizon_size)],0.};
  }

  std::vector<Eigen::Vector3d>
      X_MPC; // Contain 3d vectors that represents in that order the CoM the CoMd and the ZMP for each timestep
  std::vector<Eigen::Vector3d>
      Y_MPC; // Contain 3d vectors that represents in that order the CoM the CoMd and the ZMP for each timestep
  std::vector<Eigen::Vector3d> SupPolygon;
  std::vector<Eigen::Vector3d> FeasibilityPolygon;
  SupportPolygon FeasibilityPolygonStandingSwitch; //standing feasibility region if support foot is switched
  Eigen::VectorXd Traj_ant;
  std::vector<Eigen::Vector3d> P_traj; // Vector containing the reference trajectory
  bool Tail = true;
  int kfoot = 0;
  Eigen::Vector2d stab_error;
  bool QPSuccess;
  Eigen::Vector2d Pu_min;
  Eigen::Vector2d Pu_max;
  double alpha = 0;
  Eigen::Vector2d ref_zmp_ = Eigen::Vector2d::Zero();
  int Index = 0;

  Eigen::Vector3d initial_zmp_ = Eigen::Vector3d::Zero();

  double t_k = 0;
  double t;
  double t_lift = 0;
  Eigen::Vector3d Pck;
  Eigen::Vector3d Vck;
  Eigen::Vector3d Pzk;
  Eigen::Vector3d Uk;
  Eigen::Vector3d ComBias = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> admittance_ref_;
  Eigen::Vector3d Pu;
  Eigen::Vector3d w; // Perturbation
  Eigen::VectorXd mpc_u_;
  std::vector<Eigen::Vector3d> QP_zmp;
  std::vector<sva::MotionVecd> input_v_;
  double input_eta = 3.5;
  double eta = 3.5;
  std::vector<sva::PTransformd> input_steps_; // planner reference steps
  std::vector<sva::PTransformd> planned_steps_;
  std::vector<double> input_timesteps_; // Input desired steps timings
  std::vector<double> planned_timesteps_; //planner reference timesteps
  std::vector<sva::PTransformd> optimal_steps_; //Outputs steps from the
  std::vector<double> optimal_timesteps_; //Outputs timesteps from the mpc
  std::string input_Support_FootName;
  Eigen::Vector3d input_P_fm1;
  Eigen::Vector3d SupportFootPose;
  sva::PTransformd X_0_SupportFoot;
  sva::PTransformd X_0_Initial_SwingFoot;
  
  double tds = 0.25;
  double input_tds = 0.25;
  double optimal_tds = 0.25;
  bool stop = true;
  bool standing_mode = true;
  bool doubleSupport = true;
};
