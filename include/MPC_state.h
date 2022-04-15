#pragma once
#include <eigen3/Eigen/Dense>
#include <vector>

struct MPC_state
{

    Eigen::Vector3d Get_CoM_planarTarget(int indx)
    {
        return Eigen::Vector3d{X_MPC[indx][0], Y_MPC[indx][0], 0};
    }
    Eigen::Vector3d Get_CoMVel_planarTarget(int indx)
    {
        return Eigen::Vector3d{X_MPC[indx][1], Y_MPC[indx][1], 0};
    }
    Eigen::Vector3d Get_ZMP_planarTarget(int indx)
    {
        return Eigen::Vector3d{X_MPC[indx][2], Y_MPC[indx][2], 0.};
    }
    Eigen::Vector3d Get_CorrectedFootstep(int indx, bool add_orientation = false)
    {
        if (add_orientation)
        {
            return Eigen::Vector3d{Xf_Corr(indx),Yf_Corr(indx),0};
        }
        return Eigen::Vector3d{Xf_Corr(indx),Yf_Corr(indx),Thetaf(indx)};
    }
    Eigen::Vector3d Get_PlannedFootstep(int indx, bool add_orientation = false)
    {
        if (add_orientation)
        {
            return Eigen::Vector3d{Xf(indx),Yf(indx),0};
        }
        return Eigen::Vector3d{Xf(indx),Yf(indx),Thetaf(indx)};
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


    Eigen::VectorXd Xf = Eigen::VectorXd::Zero(2); 
    Eigen::VectorXd Xf_Corr = Eigen::VectorXd::Zero(2); ; //Support foot coordinates
    Eigen::VectorXd Yf = Eigen::VectorXd::Zero(2); ; 
    Eigen::VectorXd Yf_Corr = Eigen::VectorXd::Zero(2); ; //Support foot coordinates
    Eigen::VectorXd Thetaf = Eigen::VectorXd::Zero(2); ;
    std::vector<Eigen::Vector3d> X_MPC; //Contain 3d vectors that represents in that order the CoM the CoMd and the ZMP for each timestep
    std::vector<Eigen::Vector3d> Y_MPC; //Contain 3d vectors that represents in that order the CoM the CoMd and the ZMP for each timestep
    std::vector<Eigen::Vector3d> SupPolygon;
    Eigen::VectorXd Traj_ant;
    std::vector<Eigen::Vector3d> P_traj; //Vector containing the reference trajectory 
    std::vector<int> TimeStampsIndex; //Index of the timing of each step 
    std::vector<double> TimeStamps; //Timing of each step

};
