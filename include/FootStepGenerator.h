#pragma once
#include <mc_control/api.h>
#include <mc_control/mc_controller.h>
#include "eigen-quadprog/eigen_quadprog_api.h"
#include "eigen-quadprog/QuadProg.h"
#include <lipm_walking/utils/polynomials.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include "ControllerConfiguration.h"

struct Steps_timings_output{

    public :
        Steps_timings_output(){};
        ~Steps_timings_output() = default;
        bool QPsuccess;
        Eigen::VectorXd Ts;
        double loss;

};

class FootStepGen {
    public:
        FootStepGen();
        FootStepGen(Eigen::Vector2d Min_Max_T, Eigen::Vector2d K_cstr, double mean_v, double l, double Tp, double delta)
        {
            Ts_min_ = Min_Max_T(0) ; Ts_max_ = Min_Max_T(1);
            l_ = l; Tp_ = Tp; delta_ = delta; d_h_x = K_cstr.x(); d_h_y = K_cstr.y(); 
            Ts_ = (Ts_max_ + Ts_min_)/2;
            v_ = mean_v;
            Tp_ = Tp;
            P_ = (int)(Tp_ / delta_);

        }

        void configure(const ControllerConfiguration & config){
            Ts_min_ = config.Ts_min ; Ts_max_ = config.Ts_max;
            l_ = config.Footsteps_Generation_feet_distance; 
            Tp_ = config.Tp; delta_ = config.delta; 
            d_h_x = config.Footsteps_Generation_Kinematics_cstr.x(); 
            d_h_y = config.Footsteps_Generation_Kinematics_cstr.y();
            Ts_ = (Ts_max_ + Ts_min_)/2;
            v_ = config.Foosteps_Generation_mean_vel;
            P_ = (int)(Tp_ / delta_);
        }

        ~FootStepGen() = default;
        /**
         * Initialize the footsteps Generator
         * @tparam supportFootName 
         * @tparam P_f0 Support Foot Coordinate (Angle in P_f_0.z())
         * @tparam V Reference velocity inputs from t0 (can be empty)
         * @tparam Tstep desired ordered Steps Timing (can be empty)
         * @tparam Pf desired ordered Footsteps coordinate (can be empty) (angle in z coordinate)  
         */
        void Init(std::string supportFootName, 
                  Eigen::Vector3d P_f0 ,
                  const std::vector<std::vector<double> >  & V, const std::vector<double> & Tstep, std::vector<Eigen::Vector3d> Pf);
        
        //Return The footsteps X coordinates
        Eigen::VectorXd Xf();
        //Return The footsteps Y coordinates
        Eigen::VectorXd Yf();
        //Return The footsteps Theta values
        Eigen::VectorXd Theta_f();
        //Return The QP constraint upper boundries vector 
        std::vector<double> StepsTiming();
        //Return The Indexes in the preview horizon of the Steps Timings
        std::vector<int> TimesIndex();
        //Return the reference trajectory in the preview horizon
        std::vector<Eigen::Vector3d> Ref_Traj();
        //Compute The Footsteps and the Steps Timings
        int GetFoosteps();
        
        int Get_Nsteps(){
            return N_steps;
        }
        
        

    private:
  
        Eigen::VectorXd solveQP();

        /**
         * Compute N points trajectory between P_s_0 and P_s_1
         * @return Points Coordonate and angle of the trajectory
         */
        std::vector< Eigen::Vector3d > GetRefTrajectory(const Eigen::Vector3d & P_s_0, const Eigen::Vector3d & P_s_1);

        std::vector< std::vector<double> > GetVelocityProfile(const Eigen::Vector3d & P_s_0, double V_Max, double V_Min, const std::vector<Eigen::Vector3d> & Traj);
        
        //Compute the Steps Timing dependings of the given parameter
        void GetStepsTimings();
        Steps_timings_output Get_constrained_Ts(const Eigen::VectorXd & Ts_candidate,
                                                const std::vector<Eigen::Vector2d> & StepsTimings_Upper_Lower_cstr);
        
        

        /**
         * return the position of the reference velocity integratin the velocity profile
         * @tparam k_end time index desired
         * @return Coordinate of the integrated ref velocity at time index k with orientation in z
         */
        Eigen::Vector3d IntegrateVelProfile(int k_end); 
        int Get_ki(int k, int kfoot);
        
        std::string supportFoot = "RightFoot";

        HoubaPolynomial<Eigen::Vector2d> path;


        double d_h_x ; //Next step tolerance zone
        double d_h_y ; //Next step tolerance zone
        double l_; //Distance between foot
        double robot_height_ = 150; //in cm
        double m_t0; //Initial Time
        double t1;
        double Theta_Max_; //Max angle between two steps
        double v_min_ = 0.;
        double v_max_ = 1;
        double Tp_; // Preview horizon time
        int P_ = 2; // Preview horizon time indexes
        double delta_ ; //t_k - t_k-1
        int F_ ; //footsteps number
        double Ts_min_; double Ts_max_; //Step Time lenght limits
        double m_Lmax; //Step lenght limit;
        double Ts_ ; double m_Ls_ ; double v_ ; //Cruise Parameters
        int N_steps = -1;

        Eigen::Vector3d P_f_0;  //Actual Support foot coordinates (angle in P_s_0.z())

        std::vector<double> StepsTimings_; //Contains the time of each steps
        std::vector<int> StepsTimings_indx_; //Index timing of each steps
        std::vector<int> FootSteps_indx_; //Index of the input steps position for the right step timing
        Eigen::VectorXd Xf_ ; Eigen::VectorXd Yf_ ; Eigen::VectorXd Theta_f_ ; //Output Steps Coordinates
        Eigen::VectorXd m_Ts ;  //Steps Duration

        std::vector<double> Vx_ ; std::vector<double> Vy_; std::vector<double> Omega_; //Input velocities
        Eigen::MatrixXd V_;
        std::vector<Eigen::Vector3d> P_traj_; //Position of reference trajectory for each timesteps 
        
        std::vector<double> Ts_in_ ; //Input Step Timing
        std::vector<double> Xs_in_ ; std::vector<double> Ys_in_ ; std::vector<double> Theta_s_in_ ; //Input Steps Coordinates
        
        Eigen::Matrix3d R_support_world;
        
        //QP Problem
        bool QPsuccess = false;
        Eigen::MatrixXd Q_; //QP Hessian
        Eigen::VectorXd p_; //QP Grad

        Eigen::MatrixXd Aeq; //Equality Matrix
        Eigen::VectorXd beq; //Equality Vector

        Eigen::MatrixXd Aineq; //Inequality Matrix
        Eigen::VectorXd bineq; //Inequality Vector

          
};

class Admissible_Region {

    public :
        Admissible_Region(const Eigen::Vector3d & center,const Eigen::Vector3d & size){
            _center = center; _angle = _center.z(); _size = size; _size.z() = 0;
            _center.z() = 0;
            R.setZero();
            R(0,0) = cos(_angle) ; R(0,1) = -sin(_angle);
            R(1,0) = sin(_angle) ; R(1,1) =  cos(_angle);
            R(2,2) = 1;
            upper_left_corner  = _center + R * Eigen::Vector3d{-_size.x()/2, _size.y()/2,0};
            upper_right_corner = _center + R * Eigen::Vector3d{ _size.x()/2, _size.y()/2,0};
            lower_left_corner  = _center + R * Eigen::Vector3d{-_size.x()/2,-_size.y()/2,0};
            lower_right_corner = _center + R * Eigen::Vector3d{ _size.x()/2,-_size.y()/2,0};
            
            std::vector<Eigen::Vector3d> Polygone_Corners = {upper_left_corner,upper_right_corner,lower_right_corner,lower_left_corner};
            Polygone_Normals.resize(2,Polygone_Corners.size());
            Polygone_Edges_Center.resize(2,Polygone_Corners.size());
            Polygone_Vertices.resize(2,Polygone_Corners.size());
            Offset.resize(Polygone_Corners.size());
            for (int c = 0 ; c < Polygone_Corners.size() ; c++){
                Eigen::Vector3d point_1 = Polygone_Corners[c];
                Eigen::Vector3d point_2 = Polygone_Corners[ (c+1)%Polygone_Corners.size() ];
                Eigen::Vector3d vertice = (point_2 - point_1).normalized();
                Eigen::Vector3d normal = Eigen::Vector3d{0,0,1}.cross(vertice).normalized();
                Polygone_Normals(0,c) = normal.x();
                Polygone_Normals(1,c) = normal.y();
                Polygone_Vertices(0,c) = vertice.x();
                Polygone_Vertices(1,c) = vertice.y();


                Eigen::Matrix2d R_Vertices_0;
                R_Vertices_0(0,0) = Polygone_Normals(0,c); R_Vertices_0(1,0) = Polygone_Vertices(0,c);
                R_Vertices_0(1,0) = Polygone_Normals(1,c); R_Vertices_0(1,1) = Polygone_Vertices(1,c);

                Offset(c) = (R_Vertices_0.transpose() * Eigen::Vector2d{ point_1.x(),point_1.y()}).x();

 
            }

        }
        ~Admissible_Region() = default;

        std::vector<Eigen::Vector3d> Get_corners(){
            return {upper_left_corner,upper_right_corner,lower_right_corner,lower_left_corner};
        }

        Eigen::MatrixXd Polygone_Normals;
        Eigen::VectorXd Offset;

    private:
        
        Eigen::Vector3d _center;
        Eigen::Vector3d _size;
        double _angle;
        Eigen::Matrix3d R;
        Eigen::Vector3d upper_left_corner;
        Eigen::Vector3d upper_right_corner;
        Eigen::Vector3d lower_left_corner;
        Eigen::Vector3d lower_right_corner;
        Eigen::MatrixXd Polygone_Vertices;
        Eigen::MatrixXd Polygone_Edges_Center;

};

