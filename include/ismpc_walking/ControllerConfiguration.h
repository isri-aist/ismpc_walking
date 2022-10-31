#pragma once
#include <mc_control/mc_controller.h>

struct ControllerConfiguration
{
  double SwingFootWeight = 5000;
  double SwingFootStiffness = 300;
  double CoM_Stiff = 100;
  double CoM_Weight = 1000;
  Eigen::Vector2d Std_Admittance = {1e-2, 1e-2};
  Eigen::Vector2d Impact_Admittance = {1e-2, 1e-2};

  double Controller_timestep = 5e-3;

  Eigen::Vector3d CoMWeight_Dim{1, 1, 1};
  Eigen::Vector6d SwingFootWeight_Dim = Eigen::Vector6d::Ones();
  Eigen::Vector6d SwingFootStiffness_Dim = Eigen::Vector6d::Ones();

  double Ts_max = 2;
  double T_ds_min = 0.25;
  double T_ss_min = 0.75;
  double Ts_min = T_ds_min + T_ss_min;

  Eigen::Vector2d Footsteps_Generation_Kinematics_cstr{0.3, 0.1};

  double Beta = 1e4; // 1e4;
  double Beta_stab = 1e7;
  double Beta_traj = 0.;
  Eigen::Vector2d Beta_range{1e10, 1e2};
  bool sliding_zmp_cstr_region = false;
  bool use_stability_task = false;
  double FootStepHeight = 0.04;
  double maxVelX = 0.15;
  double minVelX = -0.15;
  double delta = 5e-2; // MPC TimeStep;
  double Tc = 1.5; // Control Time
  double Tp = 4; // Preview Time
  double Double_Step_Ratio = 0.25;
  double impact_threshold = 30;

  double MPC_ZMP_Constraint_max_size = 0.2;
  double MPC_ZMP_Constraint_min_size = 0.01;
  Eigen::Vector2d MPC_ZMP_Constraint_size{0.1, 0.08};
  Eigen::Vector2d MPC_ZMP_Constraint_size_sg_supp{0.1, 0.08};
  double MPC_ZMP_next_stp_cstr_ratio = 2;
  bool MPC_allow_None = true;
  Eigen::Vector2d MPC_ZMP_cstr_square_offset = Eigen::Vector2d::Zero();
  Eigen::Vector2d MPC_ZMP_cstr_square_offset_sg_supp = Eigen::Vector2d::Zero();
  Eigen::Vector2d MPC_Footsteps_Constraint_size{0.1, 0.1};
  Eigen::Vector2d MPC_Footsteps_kin_Constraint_size{0.1, 0.1};

  bool Stabilizer_dcm_bias_estimator = true;
  bool Stabilizer_dcm_filter = true;
  sva::MotionVecd CoPMaxVel = sva::MotionVecd(Eigen::Vector3d{1.5, 1.5, 0.3}, Eigen::Vector3d{.5, .5, 0.3});

  double wrench_filter_cutoff;
  double gamma_filter_cutoff;

  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration Stab_config;
  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration Stab_config_standing;
  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration Stab_config_sg_supp;
  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration Stab_config_dbl_supp;

  double Ts_ = 0.6;
  double Ls_ = 0.15;
  double alpha = 0.4; // Average Step duration/lentgh/speed (To process the timestamp)
};
