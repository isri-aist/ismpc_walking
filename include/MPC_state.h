#pragma once
#include <eigen3/Eigen/Dense>
#include <vector>

struct MPC_state
{

    Eigen::Vector3d Get_CoM_planarTarget(int indx)
    {
        if (indx < X_MPC.size())
        {
            return Eigen::Vector3d{X_MPC[indx][0], Y_MPC[indx][0], 0};
        }
        std::cout << "[CoM access] Warning wrong index returning 0 vector" << std::endl;
        return Eigen::Vector3d::Zero();    
    }

    Eigen::Vector3d Get_CoMVel_planarTarget(int indx)
    {
        if (indx < X_MPC.size())
        {
            return Eigen::Vector3d{X_MPC[indx][1], Y_MPC[indx][1], 0};
        }
        std::cout << "[CoMd access] Warning wrong index returning 0 vector" << std::endl;
        return Eigen::Vector3d::Zero();

        
    }

    Eigen::Vector3d Get_ZMP_planarTarget(int indx)
    {
        if (indx < X_MPC.size())
        {
            return Eigen::Vector3d{X_MPC[indx][2], Y_MPC[indx][2], 0};
        }
        std::cout << "[ZMP access] Warning wrong index returning 0 vector" << std::endl;
        return Eigen::Vector3d::Zero();

    }

    Eigen::Vector3d Get_CorrectedFootstep(int indx, bool add_orientation = false)
    {
        if(indx < Xf_Corr.size() )
        {
            if (add_orientation)
            {
                return Eigen::Vector3d{Xf_Corr(indx),Yf_Corr(indx),0};
            }
            return Eigen::Vector3d{Xf_Corr(indx),Yf_Corr(indx),Thetaf(indx)};
        }
        std::cout << "[Corr Footsteps access] Warning wrong index returning 0 vector" << std::endl;
        return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d Get_PlannedFootstep(int indx, bool add_orientation = false)
    {
        if(indx < Xf.size() )
        {
            if (add_orientation)
            {
                return Eigen::Vector3d{Xf(indx),Yf(indx),0};
            }
            return Eigen::Vector3d{Xf(indx),Yf(indx),Thetaf(indx)};
        }
        std::cout << "[Footsteps access] Warning wrong index returning 0 vector" << std::endl;
        return Eigen::Vector3d::Zero();
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

    const Eigen::VectorXd & getXf()
    {
        return Xf;
    }
    const Eigen::VectorXd & getYf()
    {
        return Yf;
    }
    const Eigen::VectorXd & getThetaf()
    {
        return Thetaf;
    }
    const Eigen::VectorXd & getXf_corr()
    {
        return Xf_Corr;
    }
    const Eigen::VectorXd & getYf_corr()
    {
        return Yf_Corr;
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
    bool Tail = true;
    int kfoot = 0;
    double stab_error;
    bool QPSuccess;
    Eigen::Vector2d Pu_min;
    Eigen::Vector2d Pu_max;
    int Index = 0;

    Eigen::Vector3d Pck;
    Eigen::Vector3d Vck;
    Eigen::Vector3d Pzk;
    Eigen::Vector3d Pu;
    Eigen::Vector3d w; //Perturbation
    std::vector<double> input_T;
    std::vector<sva::MotionVecd> input_v_;
    std::vector<sva::PTransformd> planner_steps;
    std::vector<double> input_timesteps_; //Input desired steps timings
    std::vector<sva::PTransformd> input_steps_; //Input desired FootSteps positions 
    std::string input_Support_FootName;
    Eigen::Vector3d input_P_fm1;
    Eigen::Vector3d SupportFootPose;

};
