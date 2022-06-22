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
#include <mc_tasks/AddRemoveContactTask.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/CoPTask.h>
#include <Tasks/QPContactConstr.h>
#include <mc_tasks/AdmittanceTask.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/OrientationTask.h>
#include <mc_tasks/SurfaceTransformTask.h>
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>
#include <mc_rbdyn/Robots.h>
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

struct Walking_controller_DLLAPI Walking_controller : public mc_control::fsm::Controller
{
public : 

    Walking_controller(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    ~Walking_controller() override;
    
    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;

    ControllerConfiguration Controller_Config;
    mc_rtc::Configuration planner_config_;

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

    bool double_support_state()
    {
        return DoubleSupport_state;
    }
    bool stop_phase()
    {
        return Stop;
    }
    bool robot_walking()
    {
        return Robot_Walking;
    }
    void start_stop()
    {
        Stop = !Stop;
    }
    double get_t()
    {
        return t;
    }
    double next_ts()
    {
        if (mpc_state_.TimeStamps.size() != 0)
        {
            return mpc_state_.TimeStamps[0];
        }
        return 0.;
    }
    double tds()
    {
        return mpc_state_.get_tds();
    }
    void tds(double t_ds)
    {
        input_tds = mc_filter::utils::clampAndWarn(t_ds, Controller_Config.T_ds_min,
                                                    Controller_Config.Ts_max / Controller_Config.Double_Step_Ratio,
                                                    "Tds capped");
    }
    double ts()
    {
        return T_Steps;
    }
    void ts(double ts)
    {
        T_Steps = mc_filter::utils::clampAndWarn(ts, Controller_Config.T_ds_min + Controller_Config.T_ss_min, Controller_Config.Ts_max,
                                        "Ts capped");
    }
    int n_steps()
    {
        return N_Steps_Desired;
    }
    void n_steps(int steps)
    {
        N_Steps_Desired = steps;
    }
    void SwitchFootSupport_manual()
    {
        if(!Robot_Walking)
        {
            mc_rtc::log::info("[ismpc_walking] switching support foot manually");
            switchFootSupport();
            updateTasks();
            MoveFeet(0);
            // UpdatePlanner_input();
            ComputeTrajectoryOnce = true;
        }
    }



protected:

    void getTransformations();
    sva::ForceVecd compute_momentum_contact_point();
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

    void addToGUI();

    void AddToLog();

    void add_FootSteps_GUI();

    void create_datastore(){

        datastore().make_call("ismpc_walking::stop_phase", [this]() -> bool { return Stop; });
        datastore().make_call("ismpc_walking::robot_walking", [this]() -> bool { return Robot_Walking; });
        datastore().make_call("ismpc_walking::double_support", [this]() -> bool { return DoubleSupport_state; });
        datastore().make_call("ismpc_walking::start/stop", [this]() { Stop = !Stop; });
        datastore().make_call("ismpc_walking::configure", [this](const ControllerConfiguration & config) { Configure(config); });
        datastore().make_call("ismpc_walking::get_config", [this]() -> ControllerConfiguration & { return Controller_Config; });
        datastore().make_call("ismpc_walking::zmp_target", [this]() -> Eigen::Vector3d { return zmpTarget; });
        datastore().make_call("ismpc_walking::dcm_target", [this]() -> Eigen::Vector3d { return dcmTarget; });
        datastore().make_call("ismpc_walking::zmp", [this]() -> Eigen::Vector3d { return StabTask->measuredDCM(); });
        datastore().make_call("ismpc_walking::dcm", [this]() -> Eigen::Vector3d { return StabTask->measuredZMP(); });
        datastore().make_call("ismpc_walking::support_foot_name", [this]() -> std::string { return supportFootName; });
        datastore().make_call("ismpc_walking::swing_foot_name", [this]() -> std::string { return swingFootName; });
        datastore().make_call("ismpc_walking::t", [this]() -> double { return t; });
        datastore().make_call("ismpc_walking::t_lift", [this]() -> double { return t_lift; });
        datastore().make_call("ismpc_walking::t_contact", [this]() -> double { return t_contact; });
        datastore().make_call("ismpc_walking::next_ts", [this]() -> double { return ts(); });
        datastore().make_call("ismpc_walking::get_ts_target", [this]() -> double { return T_Steps; });
        datastore().make_call("ismpc_walking::set_ts", [this](double t) { return ts(t); });
        datastore().make_call("ismpc_walking::set_tds", [this](double t) { return tds(t); });
        datastore().make_call("ismpc_walking::get_tds", [this]() -> double { return input_tds; });
        datastore().make_call("ismpc_walking::set_n_step", [this](int n) { N_Steps_Desired = n; });
        datastore().make_call("ismpc_walking::set_ref_vel", [this](Eigen::Vector3d vel) { reference_velocity = vel; });
        datastore().make_call("ismpc_walking::tds_by_ratio", [this](bool val) { Tds_by_ratio = val; });
        datastore().make_call("ismpc_walking::arm_swing_off", [this]() { armTask->weight(0); });
        datastore().make_call("ismpc_walking::arm_swing_on", [this]() { armTask->weight(10); });
        datastore().make_call("ismpc_walking::switch_support_foot", [this]() { SwitchFootSupport_manual(); });

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
    bool UseMPCState = false;
    bool Stop = true ; //If true, the robot is at stop or the robot is about to stop at next step;
    bool Robot_Walking = false; //If false, the robot is not moving;
    bool ComputeTrajectoryOnce = true; 
    bool WalkingTrajectory_Computing = false;
    bool emergencyFlag = false; //Stop controller run loop
    bool AutoFootstepPlacement = true; //To enable the Autofootstep placement MPC
    bool Tds_by_ratio = true;    

    
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
    double input_tds = 0.25; //Double Step Time duration
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

    Eigen::Vector3d StaticPose = Eigen::Vector3d::Zero();
    
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


    int kfoot = 0; //indx of the target step in the step plan

    //For Joystick {
    double maxVelX=0.15;
    double minVelX=-0.15;
    double vRefX = 0 ; double vRefY = 0 ; double omegaRef = 0;
    double PrevVrefX = 0 ;
    bool joystickConnected = true;
    //For Joystick }
    
    std::vector<Eigen::Vector3d> SupPolygon;


    

};






