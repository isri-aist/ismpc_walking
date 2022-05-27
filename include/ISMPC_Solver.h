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
         * @tparam Footsteps coordinates throught time 
         */
        void InitStepGen( std::vector<sva::PTransformd> & steps , sva::PTransformd support_foot);

        /**
         * Set the robot walking charateristics
         * @tparam Pck , Vck , Pzk : Initial CoM , CoMd and ZMP position
         * @tparam Pfm1 : Previous swing foot or actual support foot position
         * @tparam timesstp , timesindx : Steps timing and their indexes in the horizon
         * @tparam Tail , choice of the velocity tail (Truncated, Periodic, Anticipative or None)
         * @tparam Steps_Desired, steps choosen to be performed, the number of steps done must be updated manually with Steps 
         */
        void SetWalkingParameters(const Eigen::Vector3d & Pck,const Eigen::Vector3d & Vck, const Eigen::Vector3d & Pzk, const Eigen::Vector3d & Pfm1  , 
                                    const std::vector<double> & timesstp, std::string Tail, int Steps_Desired, int Steps);
        
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
        Eigen::Vector2d ZMP_Constraints_Size() {
            return Eigen::Vector2d{m_dx,m_dy};
        }
        Eigen::Vector2d Footsteps_Constraints_Size() {
            return Eigen::Vector2d{m_dx_f,m_dy_f};
        }

        double delta_mpc() const noexcept
        {
            return m_delta;
        }
        double delta_control() const noexcept
        {
            return m_delta_control;
        }


        /**
         * Returns if previous MPC was feasible
         */
        const bool & QPsucceeded() const noexcept{
            return QPsuccess;
        }
        const std::string & Tail() const noexcept{
            return m_Tail;
        }
        const Eigen::VectorXd & Xf(){
            return m_Xf ;//+ Eigen::VectorXd::Ones(m_Xf.size()) * Offset.x();;

        }
        const Eigen::VectorXd & Yf(){
            return m_Yf ;  //+ Eigen::VectorXd::Ones(m_Yf.size()) * Offset.y();;
        }

        /**
         * Return the corrected footsteps
         */ 
        const Eigen::VectorXd & Xf_Corr(){

            return m_Xf_Corr;// + Eigen::VectorXd::Ones(m_Xf_Corr.size()) * Offset.x();
        }

        /**
         * Return the corrected footsteps
         */ 
        const Eigen::VectorXd & Yf_Corr(){

            return m_Yf_Corr ;//+ Eigen::VectorXd::Ones(m_Yf_Corr.size()) * Offset.y();;
        }
        const Eigen::VectorXd & Thetaf() const noexcept{
            return m_Theta_f;
        }

        /**
         * Returns the computed trajectory, each vector3d in the vector contains the CoM , CoMd and ZMP value for a time step
         */
        const std::vector<Eigen::Vector3d> & X_MPC() const noexcept{
            return m_X_MPC;
        }
        /**
         * Returns the computed trajectory, each vector3d in the vector contains the CoM , CoMd and ZMP value for a time step
         */
        const std::vector<Eigen::Vector3d> & Y_MPC() const noexcept{
            return m_Y_MPC;
        }

        /** 
         * Return the feasibility boundries in term of initial DCM in support foot frame
         */
        const Eigen::Vector3d & Puk_min(){
            return P_u_k_min ;//+ R_0_support * Offset;
        }

        /** 
         * Return the feasibility boundries in term of initial DCM in support foot frame
         */
        const Eigen::Vector3d & Puk_max() const noexcept {
            return P_u_k_max ;//+ R_0_support * Offset;
        }

        const Eigen::Matrix3d & Support_ori() const noexcept
        {
            return R_support_0;
        }

        const Eigen::Vector3d & Disturbance() const noexcept
        {
            return w_k;
        }

        void Disturbance(const Eigen::Vector3d & w) noexcept
        {
            w_k = w;
        }

        /**
         * Returns the initial DCM used in the MPC in the world frame
         */
        const Eigen::Vector3d & Puk() const noexcept{
            return P_u_k ;//+ Offset;
        }

        double stability_error() const noexcept
        {
            return stab_error;
        }

        /**
         * Set an initial DCM in the world frame
         */
        void Puk(Eigen::Vector3d puk){
            P_u_k = puk - Offset; P_u_k.z() = 0;
        }

        const Eigen::VectorXd & GetAfterTc_ZMP_trajectory(){
            // int Size = (int) AfterTc_ZMP_trajectory.size()/2;
            // Eigen::VectorXd Output =AfterTc_ZMP_trajectory;
            // for(int i = 0; i < Size ; i++){
            //     Output(i) += Offset.x();
            //     Output(i + Size) += Offset.y();
            // }
            return AfterTc_ZMP_trajectory;
        }


        const Eigen::VectorXd & ZMP_vel() const noexcept{
            return m_ZMP_vel;
        }

        const std::vector<Eigen::Vector3d> & get_polynome_support(){
            // std::vector<Eigen::Vector3d> Output;
            // for (int k = 0; k < SuppPolyCorners.size() ; k++){
            //     Output.push_back(SuppPolyCorners[k] + Offset);
            // }
            return SuppPolyCorners;
        }

        void configure(const ControllerConfiguration & config);
        
        void Allow_none(bool state)
        {
            Allow_None = state;
        }

        std::vector<Eigen::Vector3d> zmp_ref_traj() 
        {
            std::vector<Eigen::Vector3d> Output;
            int n = (int) (b_zmp_traj.size()/2);
            for (int i = 0 ; i < n ; i++)
            {
                Output.push_back(Eigen::Vector3d{b_zmp_traj(2*i),b_zmp_traj(2*i +1),0} + P_z_k + Offset);
            }
            return Output;
        }

        bool AutoFootstepPlacement = false;

        std::vector<std::vector<Eigen::Vector3d>> All_poly;

        const std::vector<std::vector<Eigen::Vector3d>> & get_allpolys(){
            // std::vector<std::vector<Eigen::Vector3d>> Output;
            // for (int k = 0; k < All_poly.size() ; k++){
            //     std::vector<Eigen::Vector3d> poly;
            //     for (int i = 0 ; i < All_poly[k].size() ; i++)
            //     {
            //         poly.push_back(All_poly[k][i] + Offset);
            //     }
            //     Output.push_back(poly);
            // }
            return All_poly;
        }

        
        
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

        void Compute_Stability_Range();
        

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
        Eigen::Vector3d w_k; //Perturbance
        
        Eigen::Vector3d Offset; //Offset added to coordinated such as trajectory are computed using the support foot as origin 

        Eigen::Matrix3d R_support_0 = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d R_0_support = Eigen::Matrix3d::Identity();
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
        double stab_error = 0.;
        bool Use_Stability_Task = false;
        bool Allow_None = true;
        bool InStabilityRange = false;
        
        /**
        *Only during the first double support phase : If enabled, the admissible region is a sliding square,
        *otherwise it is a polygone defined by two  rectangle on both feets.
        */
        bool Slide_ZMP_region = false; 

        double m_eta; //Prendulum frequency
        double CoM_height = 0.78;
        double g = 9.8; //Gravity acceleration
        double m_tk;
        double m_Tc ; double m_Tp; // Control & Preview horizon time
        double m_Tds; //Double Support Duration
        int Tds_offset = 2;
        double m_Dstep_ratio; //T_DoubleStep/T_Step
        double m_delta ; //t_k - t_k-1
        double m_delta_control; //Controller timestep
        double m_dx ; double m_dy; //ZMP square size at one timestep
        double m_dx_sg_s ; double m_dy_sg_s; //ZMP square size at one timestep in single support;
        Eigen::Vector2d rect_pose_offset; //cstr zone offset in the foot frame for y axis, positive offset is an offset toward the other feet;
        Eigen::Vector2d rect_pose_offset_sg_supp; //ss cstr zone offset in the foot frame for y axis, positive offset is an offset toward the other feet;
        double zmp_cstr_next_stp_ratio = 2;
        double m_dx_f ; double m_dy_f; //Step Placement Region
        double m_Beta = 1e1;
        double m_Beta_stab = 1e5;
        double m_Beta_traj = 0.;
        int j_Max_C = 0; //Number of footsteps in the Control Horizon
        int j_f ; //Index of the actual support foot 
        int j_fm1; //Index of the previous support foot

        int kfoot = 0;

        std::string m_Tail; //Velocity Tailing desired
        std::string m_Tail_save; //Save of the desired Velocity Tailing

        Eigen::VectorXd AfterTc_ZMP_velocity; //velocity generated by the midpoint between the ZMP constraints after Tc
        Eigen::VectorXd AfterTc_ZMP_trajectory;

        std::vector<double> ZMP_ref_traj;
        std::vector<Eigen::Vector3d> ZMP_min_ref_traj;
        std::vector<Eigen::Vector3d> ZMP_max_ref_traj;
        Eigen::MatrixXd M_zmp_traj;
        Eigen::VectorXd b_zmp_traj;
       
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
       
        


        std::vector<Eigen::Vector3d> SuppPolyCorners;


        Eigen::MatrixXd Aineq_zmp; //Inequality ZMP Matrix 
        Eigen::VectorXd bineq_zmp; //Inequality ZMP Vector

        Eigen::MatrixXd A_stab; //Equality stability cstr matrix
        Eigen::VectorXd b_stab; //Equality stability cstr vector

        Eigen::MatrixXd Aineq_steps; //Inequality Steps Matrix 
        Eigen::VectorXd bineq_steps; //Inequality Steps Vector

        //QP Problem
        Eigen::QuadProgDense QP; 
        
        int N_variable;
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
            upper_right_corner = _center + R * Eigen::Vector3d{size.x()/2, size.y()/2,0};
            lower_left_corner  = _center + R * Eigen::Vector3d{-size.x()/2,-size.y()/2,0};
            lower_right_corner = _center + R * Eigen::Vector3d{size.x()/2,-size.y()/2,0};

        }
        ~Rectangle() = default;

        std::vector<Eigen::Vector3d> Get_corners(){
            return {upper_left_corner,upper_right_corner,lower_right_corner,lower_left_corner};
            // return {upper_left_corner,lower_left_corner,lower_right_corner,upper_right_corner};
        }
        const Eigen::Vector3d & Up_Left_corner() const noexcept
        {
            return upper_left_corner;
        }
        const Eigen::Vector3d & Up_Right_corner() const noexcept
        {
            return upper_right_corner;
        }
        const Eigen::Vector3d & Dwn_Right_corner() const noexcept
        {
            return lower_right_corner;
        }
        const Eigen::Vector3d & Dwn_Left_corner() const noexcept
        {
            return lower_left_corner;
        }

        double get_yaw()
        {
            return _angle;
        }
        Eigen::Vector3d get_center()
        {
            return _center;
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
                const Eigen::Vector3d & point_1 = SupportPolygone_Corners[c];
                const Eigen::Vector3d & point_2 = SupportPolygone_Corners[ (c+1)%SupportPolygone_Corners.size() ];
                const Eigen::Vector3d & vertice = (point_2 - point_1).normalized();
                const Eigen::Vector3d & normal = Eigen::Vector3d{0,0,1}.cross(vertice).normalized();
                SupportPolygone_Normals(0,c) = normal.x();
                SupportPolygone_Normals(1,c) = normal.y();
                SupportPolygone_Vertices(0,c) = vertice.x();
                SupportPolygone_Vertices(1,c) = vertice.y();
                SupportPolygone_Edges_Center(0,c) = (((point_2 + point_1)/2)).x();
                SupportPolygone_Edges_Center(1,c) = (((point_2 + point_1)/2)).y();

                Eigen::Matrix2d R_Vertices_0;
                R_Vertices_0 << SupportPolygone_Normals(0,c) , SupportPolygone_Vertices(0,c), SupportPolygone_Normals(1,c), SupportPolygone_Vertices(1,c);

                Offset(c) = (R_Vertices_0.transpose() * Eigen::Vector2d{ SupportPolygone_Edges_Center(0,c),
                                                                         SupportPolygone_Edges_Center(1,c)}).x();

 
            }
        }

        void jarvis_march();

        std::vector<Eigen::Vector3d> Get_Polygone_Corners(){
            return SupportPolygone_Corners;
        }

        Rectangle & get_Rectangle(int indx)
        {
            if (indx < _Rectangles.size() - 1)
            {
                return _Rectangles[indx];
            }
            else
            {
                return _Rectangles[0];
            }
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