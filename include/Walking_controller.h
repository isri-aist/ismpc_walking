#pragma once
#include <mc_control/mc_controller.h>
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>
#include <mc_tasks/AddRemoveContactTask.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/CoPTask.h>
#include <Tasks/QPContactConstr.h>
#include <mc_tasks/AdmittanceTask.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/OrientationTask.h>
#include <mc_tasks/SurfaceTransformTask.h>
#include <mc_control/api.h>
#include "joystick/joystick.hh"
#include "FootStepGenerator.h"
#include "FootTrajectory.h"
#include "ISMPC_Solver.h"
#include "api.h"
#include "eigen-quadprog/eigen_quadprog_api.h"
#include "eigen-quadprog/QuadProg.h"
#include "ControllerConfiguration.h"

struct Walking_controller_DLLAPI Walking_controller : public mc_control::MCController
{
public : 

    Walking_controller(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);
    
    void ROS_Spinner();

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;

    void getTransformations();
    Eigen::Vector3d computeInSupportFootFlat(const Eigen::Vector3d & t_world);
    Eigen::Vector3d computeVelocityInSupportFoot(const Eigen::Vector3d & v_world);
    sva::ForceVecd measuredContactWrench();
    Eigen::Vector3d computeZMP();
    /**
     * Update the stabilizer task with the ISMPC outputs stored in X_MPC and Y_MPC vectors
     */
    void MoveCoM(double t);
    /**
     * Handle the contact of the foot and the trajectory depending on the planned footsteps
     */ 
    bool MoveFeet(double t);
    /**
     * Compute the planned footsteps/timings and the CoM/ZMP trajectory
     * Footsteps coordinates are stored in XfCorr/YfCorr/ThetafCorr 
     * CoM/ZMP trajectory is stored in X_MPC and Y_MPC vectors that contains for each timestep a 3d vector with in that order 
     * the CoM, the CoMd, and the ZMP  
     */
    void ComputeWalkingTrajectory();
    void WalkingTrajectoryLoop();
    void switchFootSupport();
    void joypadLoop();
    void UpdateFootRatio();
    /**
     * Update the values of Pck Vck and Pzk from the robot
    */
    void UpdateInitialVectors();
    /**
     * Generate the Reference Velocity for constant inputs throught the preveiw horizon
     */
    void GenReferenceVelocity(double vx, double vy, double omega);

    void updateTasks();

    void reset_MPC_states(){
      t_k = 0 ; countStart = count + 1; kfoot = 0; 
      ComputeTrajectoryOnce = true;    

    }

    ControllerConfiguration Controller_Config;

    void Configure(const mc_rtc::Configuration & config){
        
        Controller_Config.Beta = config("ismpc")("beta");
        Controller_Config.Beta_range = config("ismpc")("safety_thresholds")("beta_range");
        Controller_Config.MPC_ZMP_Constraint_size = config("ismpc")("zmp_cstr_square");
        Controller_Config.MPC_ZMP_Constraint_min_size = config("ismpc")("safety_thresholds")("zmp_cstr_square_min");
        Controller_Config.MPC_ZMP_Constraint_max_size = config("ismpc")("safety_thresholds")("zmp_cstr_square_max");
        Controller_Config.MPC_Footsteps_Constraint_size = config("ismpc")("footsteps_cstr_square");
        Controller_Config.Double_Step_Ratio = config("ismpc")("double_support_ratio");
        Controller_Config.sliding_zmp_cstr_region = config("ismpc")("sliding_zmp_cstr_region");
        Controller_Config.delta =  config("ismpc")("delta");

        Controller_Config.Tp = config("footstepsGeneration")("Tp");
        Controller_Config.Footstps_Generation_Ts_range = config("footstepsGeneration")("Ts_range");
        Controller_Config.Footsteps_Generation_Kinematics_cstr = config("footstepsGeneration")("kinematics_cstr");
        Controller_Config.Footsteps_Generation_feet_distance = config("footstepsGeneration")("feet_distance");
        Controller_Config.Foosteps_Generation_mean_vel = config("footstepsGeneration")("mean_speed");
        

        Controller_Config.SwingFootStiffness = config("tasks")("swingfoot_stiffness");
        Controller_Config.SwingFootWeight = config("tasks")("swingfoot_weight");
        Controller_Config.SupportFootStiffness = config("tasks")("supportfoot_stiffness");
        Controller_Config.SupportFootWeight = config("tasks")("supportfoot_weight");
        Controller_Config.SwingFootStiffness_Dim = config("tasks")("swingfoot_dimstiffness");
        Controller_Config.SwingFootWeight_Dim = config("tasks")("swingfoot_dimweight");

        Controller_Config.CoM_Stiff = config("tasks")("stabilizer")("com_stiff");
        Controller_Config.CoM_Weight = config("tasks")("stabilizer")("com_weight");
        Controller_Config.CoMStiffness_Dim = config("tasks")("stabilizer")("com_dimstiffness");
        Controller_Config.CoMWeight_Dim = config("tasks")("stabilizer")("com_dimweight");
        Controller_Config.Stab_P_gain = config("tasks")("stabilizer")("P_gain");
        Controller_Config.Stab_I_gain = config("tasks")("stabilizer")("I_gain");
        Controller_Config.Impact_Admittance = config("tasks")("stabilizer")("impact_admittance");
        Controller_Config.Std_Admittance = config("tasks")("stabilizer")("std_admittance");
        
        Configure(Controller_Config);

    }

    void Configure(const ControllerConfiguration & config){
        Controller_Config = config;
        Controller_Config.update_config();
        Controller_Config.Beta = std::min(Controller_Config.Beta_range(1),std::max(Controller_Config.Beta_range(0),Controller_Config.Beta));
        Controller_Config.MPC_ZMP_Constraint_size.x() = std::min(Controller_Config.MPC_ZMP_Constraint_max_size,
                                                                 std::max(Controller_Config.MPC_ZMP_Constraint_min_size,
                                                                 Controller_Config.MPC_ZMP_Constraint_size.x()));
        Controller_Config.MPC_ZMP_Constraint_size.y() = std::min(Controller_Config.MPC_ZMP_Constraint_max_size,
                                                                 std::max(Controller_Config.MPC_ZMP_Constraint_min_size,
                                                                 Controller_Config.MPC_ZMP_Constraint_size.y()));
        MPCSolver.configure(Controller_Config);
        FootStpGen.configure(Controller_Config);
    }




protected:
    void addToGUI();

    void add_FootSteps_GUI();

    void create_datastore(){

        datastore().make<Eigen::Vector3d>("ZMP_target", Eigen::Vector3d::Zero());
        datastore().make<Eigen::Vector3d>("DCM_target", Eigen::Vector3d::Zero());
        datastore().make<Eigen::Vector3d>("ZMP", Eigen::Vector3d::Zero());
        datastore().make<Eigen::Vector3d>("DCM", Eigen::Vector3d::Zero());
        datastore().make<bool>("Stop",false);
        datastore().make<bool>("Start_Trigger",false);
        datastore().make<bool>("Stop_Trigger",false);
        datastore().make<bool>("Update_Config_trigger",false);
        datastore().make<ControllerConfiguration>("Controller_config",Controller_Config);

    }
    void update_datastore(){
     
        datastore().assign<Eigen::Vector3d>("ZMP_target",zmpTarget);
        datastore().assign<Eigen::Vector3d>("DCM_target",dcmTarget);
        datastore().assign<Eigen::Vector3d>("DCM",StabTask->measuredDCM());
        datastore().assign<Eigen::Vector3d>("ZMP",StabTask->measuredZMP());
        auto & start_trigg = datastore().get<bool>("Start_Trigger");
        if (start_trigg && !Robot_Walking){
            t_k = 0 ; ComputeTrajectoryOnce = true; Stop = false;
            Robot_Walking = true;
            datastore().assign<bool>("Start_Trigger",false);
        }
        auto & stop_trigg = datastore().get<bool>("Stop_Trigger");
        if (stop_trigg && Robot_Walking){
            Stop = true;
            datastore().assign<bool>("Stop_Trigger",false);
        }
        datastore().assign<bool>("Stop",Robot_Walking);

        auto & update_trig = datastore().get<bool>("Update_Config_trigger");
        if (update_trig)
        {
            const auto & controller_conf = datastore().get<ControllerConfiguration>("Controller_config");
            Configure(controller_conf);
            Controller_Config = controller_conf;
            datastore().assign<bool>("Update_Config_trigger",false);
        }


    }


    std::shared_ptr<mc_tasks::lipm_stabilizer::StabilizerTask> StabTask;    


    // std::shared_ptr<mc_tasks::CoMTask> CoMTask;

    mc_tasks::lipm_stabilizer::ContactState SwingFootContact;
    mc_tasks::lipm_stabilizer::ContactState SupportFootContact;

    std::shared_ptr<mc_tasks::OrientationTask> left_foot_ori;
    std::shared_ptr<mc_tasks::SurfaceTransformTask> SwingFootTask;
    std::shared_ptr<mc_tasks::SurfaceTransformTask> SupportFootTask;
    std::shared_ptr<mc_tasks::SurfaceTransformTask>  leftSwingFootTask;
    std::shared_ptr<mc_tasks::SurfaceTransformTask>  rightSwingFootTask;

    std::shared_ptr<mc_tasks::PostureTask> armTask;
    

private:
    
    Eigen::Vector3d dcmTarget;
    Eigen::Vector3d dcmMeasured;
    Eigen::Vector3d zmpCorr;
    Eigen::Vector3d zmpTarget;
    Eigen::Vector3d zmpMeasured;
    Eigen::Vector2d supportMin;
    Eigen::Vector2d supportMax;
    std::string LeftFootLinkName_ = "";
    std::string RightFootLinkName_ = "";

    Eigen::VectorXd Traj_ant;

    double eta(){
        return std::sqrt(std::abs(mc_rtc::constants::gravity.z())/Controller_Config.CoMz0);
    }

    double T_conv = 0.1;

    std::chrono::high_resolution_clock::time_point t_clock;

    std::chrono::high_resolution_clock::time_point t_test;

    bool User_Foot_Contact = true; //Both user feets are on the ground
    
    double x = 0.4 ; double y = 0.1 ; double z = 30; //Coordinate for a specified footstep position
    
    bool UpperBody_Control_left = false; //Enable UpperBody Control
    bool UpperBody_Control_right = false; //Enable UpperBody Control
    bool HandMoionPrediction_On = false; //Enable UpperBody Control Prediction
    bool UseRealRobot = false; //To use the real robots data
    bool Stop = true ; //If true, the robot is at stop or the robot is about to stop at next step;
    bool Robot_Walking = false; //If false, the robot is not moving;
    bool ComputeTrajectoryOnce = true; 
    bool WalkingTrajectory_Computing = false;
    bool emergencyFlag = false; //Stop controller run loop
    bool AutoFootstepPlacement = true; //To enable the Autofootstep placement MPC 

    
    double LeftFootRatio = 0.5; double PrevLeftFootRatio = 0.5;
    double Ratio_target = 0.5; //A left foot ratio that set a zmp target when Stop
    double t = 0; //General timing in a step
    double t_lift = 0; //Time when the swing foot contact has been removed
    double t_contact = 0; //Time when foot hit the ground
    double ProcessTime = 0.0;
    double ControllerLoopTime = 0;
    double T_Steps = 1.1;
    double PrevStepTiming = 0;

    int N_Steps = 0 ;
    int N_Steps_Desired = -1;

    double t_stop = 0;   
    int count_stop = 0;

    std::vector<std::vector<double>> V; //Input Reference velocity vector, contains in that order : the x-direction, the y-directon velocity, the rotating velocity
    std::vector<double> T{T_Steps}; //Input desired steps timings
    std::vector<Eigen::Vector3d> Pf; //Input desired FootSteps positions (angle in .z())

    Eigen::Vector3d SwingFootAcc;
    Eigen::Vector3d SwingFootVel;
    sva::PTransformd X_0_SwingFootInitial; //Swing Foot Pose Before Swinging
    
    Eigen::Vector3d SwingFootInitialPose; //Previous Swing Foot pose at the time of the MPC computation 

    double SwingFootInitialAngle = 0.0;
    double Tds; //Double Step Time duration
    int count = 0; //Controller iterations
    double t_k = 0;
    double controller_timestep; 
    int countStart = 0; //Controller Itaration at the time Start;
    int Index = 0; //Index of the target CoM CoMd ZMP in the vector returned by the MPC 
    bool Swing_Foot_Contact = true ; 
    bool DoubleSupport_state = true;
    std::string Tail; //Velocity tail, either "Periodic" Or "Truncated"
    Eigen::Vector3d CoM0; //Initial CoM Position for the MPC 
    Eigen::Vector3d Pzk;  //Initial ZMP Position for the MPC
    Eigen::Vector3d Pck;  //Initial CoM Position for the MPC 
    Eigen::Vector3d Puk;
    Eigen::Vector3d Vck; //Initial CoM Speed for the MPC
    Eigen::Vector3d SupportFootPose; // Initial  Foot Support at the time of computation

    Eigen::Vector3d Pz; //ZMP Position

    Eigen::Matrix3d R_body_world_Step = Eigen::Matrix3d::Identity(); //Orientation of floating base updated at each steps



    FootStepGen FootStpGen; //FootStep Position generator
    ISMPC_Solver MPCSolver; 
    FootTrajectory SwingFootTrajectory;

    bool FeetUp = false;
    
    mc_rtc::Configuration config_;

    Eigen::VectorXd Xf; Eigen::VectorXd Xf_Corr; //Support foot coordinates
    Eigen::VectorXd Yf; Eigen::VectorXd Yf_Corr; //Support foot coordinates
    Eigen::VectorXd Thetaf;
    std::vector<double> Vx;
    std::vector<double> Vy;
    std::vector<double> Omega;
    
    std::vector<Eigen::Vector3d> X_MPC; //Contain 3d vectors that represents in that order the CoM the CoMd and the ZMP for each timestep
    std::vector<Eigen::Vector3d> Y_MPC; //Contain 3d vectors that represents in that order the CoM the CoMd and the ZMP for each timestep
    std::vector<int> TimeStampsIndex; //Index of the timing of each step 
    std::vector<double> TimeStamps; //Timing of each step
    double Vx_i = 0; double Vy_i = 0 ; double Omega_i = 0;
    
    std::vector<Eigen::Vector3d> P_traj; //Vector containing the reference trajectory 

    std::vector<Eigen::Vector3d> predictedZMPWorld; //Use to display ZMP
    std::vector<Eigen::Vector3d> predictedCoMWorld; //Use to display CoM

    std::string supportFootName;
    std::string swingFootName;
    
    sva::PTransformd ReferenceFrame_Origin_Offset = sva::PTransformd::Identity();

    sva::PTransformd leftFootTransformZero;
    sva::PTransformd rightFootTransformZero;

    sva::PTransformd X_0_leftFoot;
    Eigen::Matrix3d R_0_leftFoot;
    Eigen::Matrix3d R_leftFoot_0;
    Eigen::Vector3d T_leftFoot_0;

    sva::PTransformd X_0_rightFoot;
    Eigen::Matrix3d R_0_rightFoot;
    Eigen::Matrix3d R_rightFoot_0;
    Eigen::Vector3d T_rightFoot_0;

    sva::PTransformd X_0_support;
    sva::PTransformd X_0_swing;
    sva::PTransformd X_0_support_flat;
    sva::PTransformd X_0_support_real;
    sva::PTransformd X_support_0;
    sva::PTransformd X_support_0_real;
    Eigen::Matrix3d R_0_support;
    Eigen::Vector3d T_0_support;
    Eigen::Matrix3d R_0_support_real;
    Eigen::Vector3d T_0_support_real;
    Eigen::Matrix3d R_support_0;
    Eigen::Vector3d T_support_0;
    Eigen::Matrix3d R_support_0_real;
    Eigen::Vector3d T_support_0_real;
    Eigen::Matrix3d R_swing_0;
    Eigen::Matrix3d R_0_swing;
    Eigen::Vector3d T_swing_0;

    Eigen::Matrix3d floatingbaseWorldOri;
    Eigen::Vector3d floatingbaseWorldPos;
    sva::PTransformd X_world_floatingbase;
    Eigen::Vector3d floatingbaseWorldRPY; 

    double currentLeftLeg = 0;
    double currentRightLeg = 0;

    unsigned int leftShoulderIndex = 0;
    unsigned int leftLegIndex = 0;
    unsigned int rightShoulderIndex = 0;
    unsigned int rightLegIndex = 0;

    std::vector<std::string> SwingFootJoints;

    int kfoot = 0; //Current Support Foot

    //For Joystick {
    double maxVelX=0.15;
    double minVelX=-0.15;
    double vRefX = 0 ; double vRefY = 0 ; double omegaRef = 0;
    double PrevVrefX = 0 ;
    Joystick joystick;
    JoystickEvent event;
    bool joystickConnected = true;
    //For Joystick }
    
    std::vector<Eigen::Vector3d> SupPolygon;
    

};




