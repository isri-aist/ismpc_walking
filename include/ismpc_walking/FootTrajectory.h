#pragma once
#include <Eigen/Dense>
#include <SpaceVecAlg/SpaceVecAlg>
#include<iostream>
#include<fstream>
#include "eigen-quadprog/eigen_quadprog_api.h"
#include "eigen-quadprog/QuadProg.h"


class FootTrajectory {
    public :
        FootTrajectory();
        ~FootTrajectory() = default;
       
        void Init(double t, double t0, double tf,Eigen::Vector3d p0,Eigen::Vector3d pf,Eigen::Vector3d pt, Eigen::Vector3d v, Eigen::Vector3d a);
    
        sva::PTransformd GetTrajectory(){
            
            return sva::PTransformd(sva::RotX(swingFootOrientation(0)) * sva::RotY(swingFootOrientation(1)) * sva::RotZ(swingFootOrientation(2)),
                                    swingFootPosition);
        }

        sva::MotionVecd GetVelocity(){
            return sva::MotionVecd(swingFootOrientationVelocity,swingFootVelocity);
        }

        sva::MotionVecd GetAccel(){
            return sva::MotionVecd(swingFootOrientationAcc,swingFootAcc);
        }
  
        std::vector<Eigen::Vector3d> TrajGUI();

        void setZOffset(double z){
            z_offset = std::max(0.0,std::min(z,1e-2));
        }
    
        void set_Z_ContactOffset(double z){
            Z_contact_offset = std::max(0.0,std::min(z,5e-3));
        }

        void Update_Traj(){
            New_traj = true;
        }

        void Update_Swing_Foot_State(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc, 
                                    Eigen::Vector3d ori, Eigen::Vector3d ori_vel, Eigen::Vector3d ori_acc){
            swingFootPosition = pos;
            swingFootVelocity = vel;
            swingFootAcc = acc;
            swingFootOrientation = ori;
            swingFootOrientationVelocity = ori_vel;
            swingFootOrientationAcc = ori_acc;

            New_traj = true;
        }
        void Update_Swing_Foot_State(const sva::PTransformd & pos, const sva::MotionVecd & vel, const sva::MotionVecd & acc){

            Eigen::MatrixXd R_0_pos = pos.rotation();
            double roll = atan2(R_0_pos(1, 2), R_0_pos(2, 2));
            double pitch = -asin(R_0_pos(0, 2));
            double yaw = atan2(R_0_pos(0, 1), R_0_pos(0, 0));
            swingFootPosition = pos.translation();
            swingFootVelocity = vel.linear();
            swingFootAcc = acc.linear();
            swingFootOrientation = Eigen::Vector3d{roll,pitch,yaw};
            swingFootOrientationVelocity = vel.angular();
            swingFootOrientationAcc = acc.angular();

            New_traj = true;
        }
        void Update_Swing_Foot_State(const sva::MotionVecd & acc){
    
            swingFootAcc = acc.linear();
            swingFootOrientationAcc = acc.angular();

            New_traj = true;
        }


        std::vector<Eigen::Vector3d> getSwingFootTrajectory(const sva::PTransformd & X_0_StepTarget,
                                            const sva::PTransformd & X_0_StartPose,
                                            double t,
                                            double stepHeight,
                                            double dur,
                                            double t0,
                                            double delta);

        

    private:
        bool QPsuccess = false;
        
        Eigen::VectorXd GetCoeffs(double t, double dur, Eigen::VectorXd boundaryConditions, bool IneqCstr_On);

        Eigen::VectorXd solveQP();


        double z_offset = 0.0;
        double Z_contact_offset = 0.0;

        Eigen::Vector3d swingFootPosition;
        Eigen::Vector3d swingFootVelocity;
        Eigen::Vector3d swingFootOrientation;
        Eigen::Vector3d swingFootOrientationVelocity;
        Eigen::Vector3d swingFootAcc;
        Eigen::Vector3d swingFootOrientationAcc;

        Eigen::Vector3d swingFootPosition_0;
        Eigen::Vector3d swingFootVelocity_0; 
        Eigen::Vector3d swingFootAcc_0;  
        Eigen::Vector3d swingFootOrientation_0; 
        Eigen::Vector3d swingFootOrientationVelocity_0; 
        Eigen::Vector3d swingFootOrientationAcc_0; 

        double duration = 0;
        double duration_Z = 0;
        double prev_dur = 0;

        double zi = 0;
        double zf = 0;

        double pitch_f = 0; 

        bool Z_up = false;

        Eigen::Vector3d swingFootPosition_1; 
        Eigen::Vector3d swingFootOrientation_1; 

        bool New_traj = false;

        int nXY = 6;
        int nZ = 10;
        double m_t = 0.0;
        double m_t0 = 0.0;
        double m_tf = 0.0;
   
        Eigen::Vector3d m_p0;
        Eigen::Vector3d m_at;
        Eigen::Vector3d m_vt;
        Eigen::Vector3d m_pt;

        Eigen::Vector3d m_pf;                

        Eigen::VectorXd coeffs;

        // Eigen::VectorXd m_Polynome_W;

        // Eigen::VectorXd m_Polynome_Z;

        // Eigen::VectorXd m_Polynome_X;

        // Eigen::VectorXd m_Polynome_Y;

        Eigen::VectorXd prev_poly_X;
        Eigen::VectorXd prev_poly_Y;
        Eigen::VectorXd prev_poly_Z;
        Eigen::VectorXd prev_poly_w;

        Eigen::VectorXd prev_poly_pitch;

        std::vector<Eigen::Vector3d> trajgui;

        //QP Problem
        Eigen::MatrixXd m_Q;  //QP Hessian
        Eigen::VectorXd m_p;  //QP Grad
        Eigen::MatrixXd m_G;  // QP constraints Matrix

        Eigen::MatrixXd Aeq; //Equality Matrix
        Eigen::VectorXd beq; //Equality Vector

        Eigen::MatrixXd Aineq; //Inequality Matrix
        Eigen::VectorXd bineq; //Inequality Vector


};


