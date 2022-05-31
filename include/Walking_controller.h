#pragma once
#include <mc_observers/KinematicInertialObserver.h>
#include <mc_observers/ObserverPipeline.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/logging.h>
#include <mc_solver/ConstraintSetLoader.h>
#include <chrono>
#include <unistd.h>
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
#include <mc_control/fsm/Controller.h>
#include "FootTrajectory.h"
#include "ISMPC_Solver.h"
#include "MPC_state.h"
#include "api.h"
#include "eigen-quadprog/eigen_quadprog_api.h"
#include "eigen-quadprog/QuadProg.h"
#include "ControllerConfiguration.h"
#include <condition_variable>
#include <mutex>

struct Walking_controller_DLLAPI Walking_controller : public mc_control::MCController
{
public : 

    Walking_controller(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    ~Walking_controller() override;
    
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
    /**
     * Update the values of Pck Vck and Pzk from the robot
    */
    void UpdateInitialVectors();
    /**
     * Generate the Reference Velocity for constant inputs throught the preveiw horizon
     */

    void UpdatePlanner_input();

    void updateTasks();


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
        Controller_Config.MPC_ZMP_next_stp_cstr_ratio = config("ismpc")("next_stp_cstr_ratio");
        Controller_Config.MPC_ZMP_cstr_square_offset = config("ismpc")("offset");
        Controller_Config.MPC_allow_None = config("ismpc")("allow_none_tail");
        Controller_Config.Tc = config("ismpc")("Tc");
        Controller_Config.delta = config("ismpc")("delta");
        Controller_Config.use_stability_task = config("ismpc")("use_stability_task");
        Controller_Config.Beta_stab  = config("ismpc")("beta_stab");
        Controller_Config.Beta_traj = config("ismpc")("beta_traj");
        Controller_Config.MPC_ZMP_cstr_square_offset_sg_supp = config("ismpc")("offset_sg_supp");
        Controller_Config.MPC_ZMP_Constraint_size_sg_supp = config("ismpc")("zmp_cstr_square_sg_supp");

        Controller_Config.Tp = config("footsteps_planner")("Tp");
        Controller_Config.Footstps_Generation_Ts_range = config("footsteps_planner")("Ts_limit");
        Controller_Config.Footsteps_Generation_Kinematics_cstr = config("footsteps_planner")("kinematics_cstr");
        Controller_Config.Footsteps_Generation_feet_distance = config("footsteps_planner")("feet_distance");
        Controller_Config.Foosteps_Generation_mean_vel = config("footsteps_planner")("mean_speed");
        

        Controller_Config.SwingFootStiffness = config("tasks")("swingfoot_stiffness");
        Controller_Config.SwingFootWeight = config("tasks")("swingfoot_weight");
        Controller_Config.SupportFootStiffness = config("tasks")("supportfoot_stiffness");
        Controller_Config.SupportFootWeight = config("tasks")("supportfoot_weight");
        Controller_Config.SwingFootStiffness_Dim = config("tasks")("swingfoot_dimstiffness");
        Controller_Config.SwingFootWeight_Dim = config("tasks")("swingfoot_dimweight");

        
        Configure(Controller_Config);

    }

    void Configure(const ControllerConfiguration & config){
        Controller_Config = config;
        // Controller_Config.update_config();
         
        Controller_Config.Beta = std::min(Controller_Config.Beta_range(1),std::max(Controller_Config.Beta_range(0),Controller_Config.Beta));
        Controller_Config.MPC_ZMP_Constraint_size.x() = std::min(Controller_Config.MPC_ZMP_Constraint_max_size,
                                                                 std::max(Controller_Config.MPC_ZMP_Constraint_min_size,
                                                                 Controller_Config.MPC_ZMP_Constraint_size.x()));
        Controller_Config.MPC_ZMP_Constraint_size.y() = std::min(Controller_Config.MPC_ZMP_Constraint_max_size,
                                                                 std::max(Controller_Config.MPC_ZMP_Constraint_min_size,
                                                                 Controller_Config.MPC_ZMP_Constraint_size.y()));
        MPCSolver.configure(Controller_Config);
    }

protected:
    void addToGUI();

    void AddToLog();

    void add_FootSteps_GUI();

    void create_datastore(){

        datastore().make<Eigen::Vector3d>("ismpc_walking::ZMP_target", Eigen::Vector3d::Zero());
        datastore().make<Eigen::Vector3d>("ismpc_walking::DCM_target", Eigen::Vector3d::Zero());
        datastore().make<Eigen::Vector3d>("ismpc_walking::ZMP", Eigen::Vector3d::Zero());
        datastore().make<Eigen::Vector3d>("ismpc_walking::DCM", Eigen::Vector3d::Zero());
        datastore().make<bool>("ismpc_walking::Stop",false);
        datastore().make<bool>("ismpc_walking::Start_Trigger",false);
        datastore().make<bool>("ismpc_walking::Stop_Trigger",false);
        datastore().make<bool>("ismpc_walking::Update_Config_trigger",false);
        datastore().make<ControllerConfiguration>("ismpc_walking::Controller_config",Controller_Config);
        
        datastore().make<std::string>("ismpc_walking::supportFootName", "RightFoot");
        datastore().make<std::string>("ismpc_walking::swingFootName", "LeftFoot");

        datastore().make<double>("ismpc_walking::T_step_in",0.0); //Steps Timings input
        datastore().make<double>("ismpc_walking::t",0.0); //General timing in a step
        datastore().make<double>("ismpc_walking::Ts",0.0); // TimeStamps

        datastore().make<Eigen::Vector3d>("ismpc_walking::input_vel",Eigen::Vector3d::Zero()); 
        datastore().make<double>("ismpc_walking::input_timestep",1.0); 
        datastore().make<int>("ismpc_walking::steps_target",-1); 


    }
    
    void update_datastore(){
     
        datastore().assign<Eigen::Vector3d>("ismpc_walking::ZMP_target",zmpTarget);
        datastore().assign<Eigen::Vector3d>("ismpc_walking::DCM_target",dcmTarget);
        datastore().assign<Eigen::Vector3d>("ismpc_walking::DCM",StabTask->measuredDCM());
        datastore().assign<Eigen::Vector3d>("ismpc_walking::ZMP",StabTask->measuredZMP());
        auto & start_trigg = datastore().get<bool>("ismpc_walking::Start_Trigger");
        if (start_trigg && !Robot_Walking){
            t_k = 0 ; ComputeTrajectoryOnce = true; Stop = false;
            Robot_Walking = true;
            datastore().assign<bool>("ismpc_walking::Start_Trigger",false);
        }
        auto & stop_trigg = datastore().get<bool>("ismpc_walking::Stop_Trigger");
        if (stop_trigg && Robot_Walking){
            Stop = true;
            datastore().assign<bool>("ismpc_walking::Stop_Trigger",false);
        }
        datastore().assign<bool>("ismpc_walking::Stop",Robot_Walking);

        auto & update_trig = datastore().get<bool>("ismpc_walking::Update_Config_trigger");
        if (update_trig)
        {
            const auto & controller_conf = datastore().get<ControllerConfiguration>("ismpc_walking::Controller_config");
            Configure(controller_conf);
            Controller_Config = controller_conf;
            datastore().assign<bool>("ismpc_walking::Update_Config_trigger",false);
        }

        datastore().assign<std::string>("ismpc_walking::supportFootName",supportFootName);
        datastore().assign<std::string>("ismpc_walking::swingFootName",swingFootName);
        if (mpc_state_.input_timesteps_.size() != 0){
            datastore().assign<double>("ismpc_walking::T_step_in",mpc_state_.input_timesteps_[0]);
        }
        datastore().assign<double>("ismpc_walking::t",t);
        if (mpc_state_.TimeStamps.size() != 0){
            datastore().assign<double>("ismpc_walking::Ts",mpc_state_.TimeStamps[0]);
        }

        T_Steps = datastore().get<double>("ismpc_walking::input_timestep");
        reference_velocity = datastore().get<Eigen::Vector3d>("ismpc_walking::input_vel");
        N_Steps_Desired = datastore().get<int>("ismpc_walking::steps_target"); 

    }

    void wait_for_mpc_thread();


    std::shared_ptr<mc_tasks::lipm_stabilizer::StabilizerTask> StabTask;    

    std::shared_ptr<mc_tasks::OrientationTask> left_foot_ori;
    std::shared_ptr<mc_tasks::SurfaceTransformTask> SwingFootTask;
    std::shared_ptr<mc_tasks::SurfaceTransformTask> SupportFootTask;
    std::shared_ptr<mc_tasks::SurfaceTransformTask>  leftSwingFootTask;
    std::shared_ptr<mc_tasks::SurfaceTransformTask>  rightSwingFootTask;

    std::shared_ptr<mc_tasks::PostureTask> armTask;
    

private:

    std::mutex mutex_mpc_;
    MPC_state mpc_thread_state;
    MPC_state mpc_state_;
    bool MPC_thread_on = false;
    bool NewThreadState = false;
    std::thread WalkingTrajectoryThread;
    
    Eigen::Vector3d dcmTarget;
    Eigen::Vector3d dcmMeasured;
    Eigen::Vector3d zmpCorr;
    Eigen::Vector3d zmpTarget;
    Eigen::Vector3d zmpMeasured;
    Eigen::Vector2d supportMin;
    Eigen::Vector2d supportMax;
    std::string torsoBodyName_ = "";
    std::string LeftFootLinkName_ = "";
    std::string RightFootLinkName_ = "";

    Eigen::VectorXd Traj_ant;

    double eta(){
        return std::sqrt(std::abs(mc_rtc::constants::gravity.z())/Controller_Config.Stab_config.comHeight);
    }


    std::chrono::high_resolution_clock::time_point t_clock;

    std::chrono::high_resolution_clock::time_point t_test;

    bool User_Foot_Contact = true; //Both user feets are on the ground
    
    double x = 0.4 ; double y = 0.1 ; double z = 30; //Coordinate for a specified footstep position

    bool UseRealRobot = false; //To use the real robots data
    bool UseMPCState = true;
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

    Eigen::Vector3d SwingFootAcc;
    Eigen::Vector3d SwingFootVel;
    sva::PTransformd X_0_SwingFootInitial; //Swing Foot Pose Before Swinging
    sva::PTransformd X_0_SwingFootInitial_real;
    
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

    bool Use_w = false;
    std::string Tail = "Anticipative"; //Velocity tail, either "Periodic" Or "Truncated"

    Eigen::Vector3d SupportFootPose; // Initial  Foot Support at the time of computation

    Eigen::Matrix3d R_body_world_Step = Eigen::Matrix3d::Identity(); //Orientation of floating base updated at each steps

    std::string supportFootName = "RightFoot";
    std::string swingFootName = "LeftFoot";

    ISMPC_Solver MPCSolver; 
    FootTrajectory SwingFootTrajectory;

    bool FeetUp = false;
    
    mc_rtc::Configuration config_;


    double Vx_i = 0; double Vy_i = 0 ; double Omega_i = 0;
    Eigen::Vector3d reference_velocity;
    
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

    Eigen::Vector6d footcontact_dof;


    int kfoot = 0; //Current Support Foot

    //For Joystick {
    double maxVelX=0.15;
    double minVelX=-0.15;
    double vRefX = 0 ; double vRefY = 0 ; double omegaRef = 0;
    double PrevVrefX = 0 ;
    bool joystickConnected = true;
    //For Joystick }
    
    std::vector<Eigen::Vector3d> SupPolygon;


    

};






