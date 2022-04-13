#pragma once
#include <mc_control/api.h>
#include <mc_control/mc_controller.h>
#include <thread>
#include "eigen-quadprog/eigen_quadprog_api.h"
#include "eigen-quadprog/QuadProg.h"
#include "ControllerConfiguration.h"


class ISMPC_Solver {
    public : 
        ISMPC_Solver();

        /**
         * Initialize the fixed parameters of the MPC
         * @tparam delta_controller : controller timestep
         * @tparam delta : MPC timestep
         * @tparam Tp : Preview horizon lenght, horizon where footsteps and trajectory will be computed
         * @tparam Tc : Control horizon lenght, horizon where the CoM/ZMP trajectory will be computed (Tc < Tp)
         */
        ISMPC_Solver(double delta_controller ,double delta, double Tp, double Tc);
        
        ~ISMPC_Solver() = default;
        /**
         * Initialize the fixed parameters of the MPC
         * @tparam delta_controller : controller timestep
         * @tparam delta : MPC timestep
         * @tparam Tp : Preview horizon lenght, horizon where footsteps and trajectory will be computed
         * @tparam Tc : Control horizon lenght, horizon where the CoM/ZMP trajectory will be computed (Tc < Tp)
         * @tparam Beta : Weight of the footsteps position in the QP cost function
         */
        void Init(double delta_controller ,double delta, double Tp, double Tc, double Beta);

        /**
         * Initialize the footsteps parameters for the MPC
         * @tparam Xf, Yf, Thetaf : Footsteps coordinates throught time 
         */
        void InitStepGen( const Eigen::VectorXd & Xf, const Eigen::VectorXd & Yf, const Eigen::VectorXd & Thetaf);

        /**
         * Set the robot walking charateristics
         * @tparam Pck , Vck , Pzk : Initial CoM , CoMd and ZMP position
         * @tparam Pfm1 : Previous swing foot or actual support foot position
         * @tparam timesstp , timesindx : Steps timing and their indexes in the horizon
         * @tparam Tail , choice of the velocity tail (Truncated, Periodic, Anticipative or None)
         * @tparam Steps_Desired, steps choosen to be performed, the number of steps done must be updated manually with Steps 
         */
        void SetWalkingParameters(const Eigen::Vector3d & Pck,const Eigen::Vector3d & Vck, const Eigen::Vector3d & Pzk, const Eigen::Vector3d & Pfm1  , 
                                    const std::vector<double> & timesstp, const std::vector<int> & timesindx, std::string Tail, int Steps_Desired, int Steps);
        
        /**
         * Compute the CoM, CoMd, ZMP trajectory for previously set Walking parameters
         * @tparam PrevStepTime, previous footsteps timing
         * @tparam t_k, time of the computation
         * @tparam Tds,  double support duration
         */
        void GetWalkingParameters(double PrevStepTime, double t_k, double Tds);
        
        /**
         * @brief Set The constraints region for the ZMP (during each delta time) and the footsteps in the robot frame
         * 
         * @param ZMP 
         * @param FootSteps 
         */
        void MPC_Constraints_region(Eigen::Vector2d ZMP,Eigen::Vector2d FootSteps){
            m_dx = ZMP.x() ; m_dy = ZMP.y();
            m_dx_f = FootSteps.x() ; m_dy_f = FootSteps.y();
        }
        // void MPC_Constraints_region(Eigen::Vector2d ZMP){
        //     m_dx = ZMP.x() ; m_dy = ZMP.y();
        // }
        // void MPC_Constraints_region(Eigen::Vector2d FootSteps){
        //     m_dx_f = FootSteps.x() ; m_dy_f = FootSteps.y();
        // }
        Eigen::Vector2d ZMP_Constraints_Size(){
            return Eigen::Vector2d{m_dx,m_dy};
        }
        Eigen::Vector2d Footsteps_Constraints_Size(){
            return Eigen::Vector2d{m_dx_f,m_dy_f};
        }



        /**
         * Returns if previous MPC was feasible
         */
        bool QPsucceeded(){
            return QPsuccess;
        }
        std::string Tail(){
            return m_Tail;
        }
        Eigen::VectorXd Xf(){
            Eigen::VectorXd Output(m_Xf.size());
            for(int i = 0 ; i < m_Xf.size() ; i++){Output(i) = m_Xf(i) + Offset.x() ;} 
            return Output;
        }
        Eigen::VectorXd Yf(){
            Eigen::VectorXd Output(m_Yf.size());
            for(int i = 0 ; i < m_Yf.size() ; i++){Output(i) = m_Yf(i) + Offset.y() ;} 
            return Output;
        }

        /**
         * Return the corrected footsteps
         */ 
        Eigen::VectorXd Xf_Corr(){
            Eigen::VectorXd Output(m_Xf_Corr.size());
            for(int i = 0 ; i < m_Xf_Corr.size() ; i++){Output(i) = m_Xf_Corr(i) + Offset.x() ;} 
            return Output;
        }

        /**
         * Return the corrected footsteps
         */ 
        Eigen::VectorXd Yf_Corr(){
            Eigen::VectorXd Output(m_Yf_Corr.size());
            for(int i = 0 ; i < m_Yf_Corr.size() ; i++){Output(i) = m_Yf_Corr(i) + Offset.y() ;} 
            return Output;
        }
        Eigen::VectorXd Thetaf(){
            return m_Theta_f;
        }

        /**
         * Returns the computed trajectory, each vector3d in the vector contains the CoM , CoMd and ZMP value for a time step
         */
        std::vector<Eigen::Vector3d> X_MPC(){
            return m_X_MPC;
        }
        /**
         * Returns the computed trajectory, each vector3d in the vector contains the CoM , CoMd and ZMP value for a time step
         */
        std::vector<Eigen::Vector3d> Y_MPC(){
            return m_Y_MPC;
        }

        /** 
         * Return the feasibility boundries in term of initial DCM in support foot frame
         */
        Eigen::Vector3d Puk_min(){
            return sva::RotZ(m_Theta_f(0)) * (P_u_k_min + Offset);
        }

        /** 
         * Return the feasibility boundries in term of initial DCM in support foot frame
         */
        Eigen::Vector3d Puk_max(){
            return sva::RotZ(m_Theta_f(0)) * (P_u_k_max + Offset);
        }

        /**
         * Returns the initial DCM used in the MPC in the world frame
         */
        Eigen::Vector3d Puk(){
            return P_u_k + Offset;
        }

        /**
         * Set an initial DCM in the world frame
         */
        void Puk(Eigen::Vector3d puk){
            P_u_k = puk - Offset; P_u_k.z() = 0;
        }

        Eigen::VectorXd GetAfterTc_ZMP_trajectory(){
            int Size = (int) AfterTc_ZMP_trajectory.size()/2;
            Eigen::VectorXd Output =AfterTc_ZMP_trajectory;
            for(int i = 0; i < Size ; i++){
                Output(i) += Offset.x();
                Output(i + Size) += Offset.y();
            }
        return Output;
        }

        /**
         * Returns ZMP Constraints squares
         * 
         * @return A vector Containing :  Coordinates of the ZMP constraints throught time
         * @tparam lower_upper, 0 : upper ; 1: lower
         * @tparam axis, 0 : X ; 1 : Y
         */
        Eigen::VectorXd ZMP_boundries(int upper_lower , int axis){
        if (upper_lower == 0){
            if(axis == 0){
            return ZMP_Upper_Limits_X;
            }
            else{
            return ZMP_Upper_Limits_Y;
            }
        }
        else{
            if(axis == 0){
            return ZMP_Lower_Limits_X;
            }
            else{
            return ZMP_Lower_Limits_Y;
            }
        }
        }

        Eigen::VectorXd ZMP_vel(){
            return m_ZMP_vel;
        }

        std::vector<Eigen::Vector3d> get_polynome_support(){
            std::vector<Eigen::Vector3d> Output;
            for (int k = 0; k < SuppPolyCorners.size() ; k++){
                Output.push_back(SuppPolyCorners[k] + Offset);
            }
            return Output;
        }

        void configure(const ControllerConfiguration & config){
            
            m_dx_f = config.MPC_Footsteps_Constraint_size.x();
            m_dy_f = config.MPC_Footsteps_Constraint_size.y();
            m_dx  = config.MPC_ZMP_Constraint_size.x();
            m_dy  = config.MPC_ZMP_Constraint_size.y();
            m_Beta = config.Beta;
            Slide_ZMP_region = config.sliding_zmp_cstr_region;
            // std::cout << "ZMP slide " << Slide_ZMP_region << std::endl;
        }
        
        bool AutoFootstepPlacement = false;

        
        
    private : 

        /**
         * ZMP Trajectory constraints : 
         * QP is build such as the output vector contains : 
         * -The ZMP velocities in x and y (world frame) ordered by timesteps then x then y 
         * -The Optimized Footstep (if computed) in x and y ordered by timesteps then x then y 
         * -The inequality constraints are set in the constraints matrix such as the first part represent the zmp position constraints and then the
         * Footsteps position constraints
         */
        void ZMP_Constraints();

        void FootSteps_Constraints();
        
        /** 
         * Stability constraints are the QP equality constraints, first line is the X axis, second is the Y axis
         */
        void Stability_Constraints();

        /** 
         * Integrate The ZMP velocity to compute the CoM, CoMd and ZMP trajectory
         */ 
        void IntegrateZMPVel();

        /**
         * Generate a ZMP trajectory that is the middle point of the zmp square constraints between the preview and control horizon.
         * Trajectory is computed in terms of ZMP velocity
         */ 
        void AntTailTrajectory();

        Eigen::VectorXd solveQP();
        
        Eigen::Vector3d P_z_k; //Initial ZMP position
        Eigen::Vector3d P_c_k; //Initial CoM Position
        Eigen::Vector3d V_c_k; //Initial CoM Velocity
        Eigen::Vector3d P_u_k; //Initial Unstable Component/DCM
        Eigen::Vector3d m_Pfm1; //Swing Foot Pose Before Swinging orientation in z
        
        Eigen::Vector3d Offset; //Offset added to coordinated such as trajectory are computed using the support foot as origin 

        Eigen::VectorXd m_Xf ; Eigen::VectorXd m_Yf ;//Steps Coordinates. Also Include the standing foot coordinates
        Eigen::VectorXd m_Theta_f ; //Steps Angle. Also include the standing foot angle
        Eigen::VectorXd m_Xf_Corr ; Eigen::VectorXd m_Yf_Corr ; //Footstep after replanning;
        Eigen::VectorXd m_ZMP_vel; //Computed ZMP velocity in world frame
        std::vector<double> m_timestamp; //Step TimesStamp Computed at the footStep Generation
        std::vector<int> m_timesIndex; // TimeStep Index Computed at the footStep Generation

        Eigen::Vector3d P_u_k_min; //Min initial DCM coordinates in support Foot Frame
        Eigen::Vector3d P_u_k_max; //Max initial DCM coordinates in support Foot Frame
        
        Eigen::VectorXd QP_Output;

        std::vector<Eigen::Vector3d> m_X_MPC; //Integrated CoM, CoMd, ZMP trajectory in world frame
        std::vector<Eigen::Vector3d> m_Y_MPC; //Integrated CoM, CoMd, ZMP trajectory in world frame

        double Ant_Tail_X = 0.0;
        double Ant_Tail_Y = 0.0;

        int N_Steps_Desired = -1;
        int N_Steps = 0;
          
        bool QPsuccess = false;
        bool InStabilityRange = false;
        
        /**
        *Only during the first double support phase : If enabled, the admissible region is a sliding square,
        *otherwise it is a polygone defined by two  rectangle on both feets.
        */
        bool Slide_ZMP_region = false; 

        double m_eta; //Prendulum frequency
        double g = 9.8; //Gravity acceleration
        double m_tk;
        double m_Tc ; double m_Tp; // Control & Preview horizon time
        double m_Tds; //Double Support Duration
        double m_Dstep_ratio; //T_DoubleStep/T_Step
        double m_delta ; //t_k - t_k-1
        double m_delta_control; //Controller timestep
        double m_dx ; double m_dy; //ZMP square size at one timestep
        double m_dx_f ; double m_dy_f; //Step Placement Region
        double m_Beta = 1e1;
        int j_Max_C = 0; //Number of footsteps in the Control Horizon
        
        int j_f ; //Index of the actual support foot 
        int j_fm1; //Index of the previous support foot

        int kfoot = 0;

        std::string m_Tail; //Velocity Tailing desired
        std::string m_Tail_save; //Save of the desired Velocity Tailing

        Eigen::VectorXd AfterTc_ZMP_velocity; //velocity generated by the midpoint between the ZMP constraints after Tc
        Eigen::VectorXd AfterTc_ZMP_trajectory;
        
        //CoM,CoMd,ZMP Integration
        Eigen::Matrix3d Integration_Mat;
        Eigen::Vector3d Integration_Vec;

        Eigen::VectorXd Pzk_Offset; //Vector that represent the intial position of the ZMP
        Eigen::MatrixXd C; //Temporary matrix to compute ZMP constraints with footsteps placements
        // Eigen::MatrixXd Delta; //Matrix to derive the ZMP position to ZMP velocity

        int m_C; //Number of indexs in the Control time length Tc = m_C * m_delta
        int m_P; //Number of indexs in the Preview time length Tp = m_P * m_delta
        int m_D;         //Number of Iteration on the double steps period
        int count_Dstep; //Number bounded between 1 and m_D describing the position of the zone during the doubleStep timing
       
        
        //Display
        Eigen::VectorXd ZMP_Lower_Limits_X; //For GUI Display Only
        Eigen::VectorXd ZMP_Lower_Limits_Y; //For GUI Display Only
        Eigen::VectorXd ZMP_Upper_Limits_X; //For GUI Display Only
        Eigen::VectorXd ZMP_Upper_Limits_Y; //For GUI Display Only
       
        std::vector<Eigen::Vector3d> SuppPolyCorners;

        Eigen::MatrixXd Aineq_zmp; //Inequality ZMP Matrix 
        Eigen::VectorXd bineq_zmp; //Inequality ZMP Vector

        Eigen::MatrixXd Aineq_steps; //Inequality Steps Matrix 
        Eigen::VectorXd bineq_steps; //Inequality Steps Vector

        //QP Problem
        Eigen::MatrixXd m_Q;  //QP Hessian
        Eigen::VectorXd m_p;  //QP Grad
        Eigen::MatrixXd m_G;  // QP constraints Matrix

        Eigen::MatrixXd Aeq; //Equality Matrix
        Eigen::VectorXd beq; //Equality Vector

        Eigen::MatrixXd Aineq; //Inequality Matrix
        Eigen::VectorXd bineq; //Inequality Vector



};

class Rectangle {

    public :
        Rectangle(const Eigen::Vector3d & center, const Eigen::Vector3d & size){
            _center = center; _angle = _center.z(); _size = size;
            _center.z() = 0;
            R.setZero();
            R(0,0) = cos(_angle) ; R(0,1) = -sin(_angle);
            R(1,0) = sin(_angle) ; R(1,1) =  cos(_angle);
            R(2,2) = 1;
            upper_left_corner  = _center + R * Eigen::Vector3d{-size.x()/2, size.y()/2,0};
            upper_right_corner = _center + R * Eigen::Vector3d{ size.x()/2, size.y()/2,0};
            lower_left_corner  = _center + R * Eigen::Vector3d{-size.x()/2,-size.y()/2,0};
            lower_right_corner = _center + R * Eigen::Vector3d{ size.x()/2,-size.y()/2,0};

        }
        ~Rectangle() = default;

        std::vector<Eigen::Vector3d> Get_corners(){
            return {upper_left_corner,upper_right_corner,lower_right_corner,lower_left_corner};
        }

    private:
        
        Eigen::Vector3d _center;
        Eigen::Vector3d _size;
        double _angle;
        Eigen::Matrix3d R;
        Eigen::Vector3d upper_left_corner;
        Eigen::Vector3d upper_right_corner;
        Eigen::Vector3d lower_left_corner;
        Eigen::Vector3d lower_right_corner;

};

struct vec3d_x_comp
{
    inline bool operator() (const Eigen::Vector3d& struct1, const Eigen::Vector3d& struct2)
    {
        return (struct1.x() < struct2.x());
    }
};

class SupportPolygon{

    public:
        SupportPolygon(const Rectangle & Rect1,const Rectangle & Rect2){
            _Rectangles = {Rect1,Rect2};
            Compute_polygone();
        }
        SupportPolygon(const Rectangle & Rect1){
             _Rectangles = {Rect1};
             Compute_polygone();
        }

        ~SupportPolygon() = default;

        void Compute_polygone(){
            for (int r = 0; r < _Rectangles.size() ; r++){
                std::vector<Eigen::Vector3d> corners = _Rectangles[r].Get_corners();
                for (int c = 0; c < corners.size() ; c++){
                    _Corners.push_back(corners[c]);
                }
            }
            if (_Rectangles.size() > 1){
                jarvis_march();
            }
            else{
                SupportPolygone_Corners = _Rectangles[0].Get_corners();
            }
            SupportPolygone_Normals.resize(2,SupportPolygone_Corners.size());
            SupportPolygone_Edges_Center.resize(2,SupportPolygone_Corners.size());
            SupportPolygone_Vertices.resize(2,SupportPolygone_Corners.size());
            Offset.resize(SupportPolygone_Corners.size());
            for (int c = 0 ; c < SupportPolygone_Corners.size() ; c++){
                Eigen::Vector3d point_1 = SupportPolygone_Corners[c];
                Eigen::Vector3d point_2 = SupportPolygone_Corners[ (c+1)%SupportPolygone_Corners.size() ];
                Eigen::Vector3d vertice = (point_2 - point_1).normalized();
                Eigen::Vector3d normal = Eigen::Vector3d{0,0,1}.cross(vertice).normalized();
                SupportPolygone_Normals(0,c) = normal.x();
                SupportPolygone_Normals(1,c) = normal.y();
                SupportPolygone_Vertices(0,c) = vertice.x();
                SupportPolygone_Vertices(1,c) = vertice.y();
                SupportPolygone_Edges_Center(0,c) = (((point_2 + point_1)/2)).x();
                SupportPolygone_Edges_Center(1,c) = (((point_2 + point_1)/2)).y();

                Eigen::Matrix2d R_Vertices_0;
                R_Vertices_0(0,0) = SupportPolygone_Normals(0,c); R_Vertices_0(1,0) = SupportPolygone_Vertices(0,c);
                R_Vertices_0(1,0) = SupportPolygone_Normals(1,c); R_Vertices_0(1,1) = SupportPolygone_Vertices(1,c);

                Offset(c) = (R_Vertices_0.transpose() * Eigen::Vector2d{ SupportPolygone_Edges_Center(0,c),
                                                                         SupportPolygone_Edges_Center(1,c)}).x();

 
            }
        }

        void jarvis_march();

        std::vector<Eigen::Vector3d> Get_Polygone_Corners(){
            return SupportPolygone_Corners;
        }

        
        Eigen::MatrixXd SupportPolygone_Normals;
        

        Eigen::VectorXd Offset;


    private:

        std::vector<Rectangle> _Rectangles;
        std::vector<Eigen::Vector3d> _Corners;
        std::vector<Eigen::Vector3d> SupportPolygone_Corners;
        Eigen::MatrixXd SupportPolygone_Vertices;
        Eigen::MatrixXd SupportPolygone_Edges_Center;



};