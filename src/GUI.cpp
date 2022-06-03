#include "../include/ismpc_walking/Walking_controller.h"

inline double floorn(double x, int n)
{
  return floor(pow(10, n) * x) / pow(10, n);
}

void Walking_controller::addToGUI()
{

  gui()->addElement({"Walking", "Action"},

                    mc_rtc::gui::Button("Start Move",
                                        [this]() {
                                          ComputeTrajectoryOnce = true;
                                          Stop = false;
                                        }),
                    mc_rtc::gui::Button("Stop",
                                        [this]() {
                                          Stop = true;
                                        }),
                    mc_rtc::gui::ComboInput(
                        "Velocity Tail", {"Periodic", "Truncated", "Anticipative", "None"}, [this]() { return Tail; },
                        [this](const std::string str) {
                          Tail = str;
                          ComputeTrajectoryOnce = true;
                        }),
                    mc_rtc::gui::Label("Velocity Tail used", [this]() { return this->MPCSolver.Tail(); }),
                    mc_rtc::gui::Label("Timing", [this]() { return this->t; }),
                    mc_rtc::gui::Label("Double support duration", [this]() { return this->Tds; }),
                    mc_rtc::gui::Label("Next Step Timing ",
                                       [this]() {
                                         if(this->mpc_state_.TimeStamps.size() != 0)
                                         {
                                           return this->mpc_state_.get_Ts(0);
                                         }
                                         else
                                         {
                                           return 0.0;
                                         }
                                       }),
                    mc_rtc::gui::Checkbox(
                        "Automatic FootStep Placement", [this]() { return AutoFootstepPlacement; },
                        [this]() { AutoFootstepPlacement = !AutoFootstepPlacement; }),
                    mc_rtc::gui::Checkbox(
                        "Real Robot Data feedback", [this]() { return UseRealRobot; },
                        [this]() {
                          UseRealRobot = !UseRealRobot;
                          if(UseMPCState) UseMPCState = false;
                          ComputeTrajectoryOnce = true;
                        }),
                    mc_rtc::gui::Checkbox(
                        "MPC state feedback", [this]() { return UseMPCState; },
                        [this]() {
                          UseMPCState = !UseMPCState;
                          if(UseRealRobot) UseRealRobot = false;
                          ComputeTrajectoryOnce = true;
                        }),
                    mc_rtc::gui::Checkbox(
                        "Distrubance", [this]() { return Use_w; },
                        [this]() {
                          Use_w = !Use_w;
                        }),
                    // mc_rtc::gui::Checkbox("ZMP_Corr", [this](){return ZMP_correction;}, [this](){ZMP_correction =
                    // !ZMP_correction;}),
                    mc_rtc::gui::Label("Stab Error (m)", [this]() { return this->MPCSolver.stability_error(); }),
                    mc_rtc::gui::Label("MPC Processing Time (ms)", [this]() { return this->ProcessTime; }),
                    mc_rtc::gui::Label("Run Loop Processing Time (ms)", [this]() { return this->ControllerLoopTime; })
                    // mc_rtc::gui::Label("ZMP box range x",[this](){return this->MPCSolver.ZMP_dx;}),
                    // mc_rtc::gui::Label("ZMP box range y",[this](){return this->MPCSolver.ZMP_dy;})
  );

  gui()->addElement(
      {"Walking", "Footsteps Parameters"},
      mc_rtc::gui::ArrayInput(
          "Reference velocity", {"x", "y", "omega"},
          [this]() -> const Eigen::Vector3d & {
            return reference_velocity;
          },
          [this](const Eigen::Vector3d & vel) {
            reference_velocity = vel;
          }),      
      mc_rtc::gui::NumberInput(
          "Ts", [this]() -> double { return T_Steps; }, [this](const double & t) { T_Steps = t; }),
      mc_rtc::gui::NumberInput(
          "Steps", [this]() -> int { return N_Steps_Desired; }, [this](const double & n) { N_Steps_Desired = n;  }),

      mc_rtc::gui::Point3D("Steps desired pose", mc_rtc::gui::PointConfig(mc_rtc::gui::Color(1, 0, 1)),
                           [this]() {
                             return Eigen::Vector3d{x, y, 0};
                           }),
      mc_rtc::gui::Button("Compute Trajectory", [this]() { ComputeTrajectoryOnce = true; }));


  gui()->addElement(
      {"Walking", "Visualization", "Feasibility"},
      mc_rtc::gui::Polygon(
          "Feasibility Region", mc_rtc::gui::Color(1., 0., 1.),
          [this]() -> std::vector<Eigen::Vector3d> {
            Eigen::Vector3d p0 = R_support_0 * this->MPCSolver.Puk_min();
            Eigen::Vector3d p2 = R_support_0 * this->MPCSolver.Puk_max();
            Eigen::Vector3d p1 =
                p0 + R_support_0 * Eigen::Vector3d{this->MPCSolver.Puk_max().x() - this->MPCSolver.Puk_min().x(), 0, 0};
            Eigen::Vector3d p3 =
                p0 + R_support_0 * Eigen::Vector3d{0, this->MPCSolver.Puk_max().y() - this->MPCSolver.Puk_min().y(), 0};
            return {p0, p1, p2, p3};
          }),
      mc_rtc::gui::Point3D("Pu", mc_rtc::gui::PointConfig(mc_rtc::gui::Color(1, 0.5, 0.25)),
                           [this]() {
                             Eigen::Vector3d vec(this->MPCSolver.Puk());
                             vec.z() = 0;
                             return vec;
                           }),
      mc_rtc::gui::Point3D("DCM", mc_rtc::gui::PointConfig(mc_rtc::gui::Color(1, 0.75, 0.25)), [this]() {
        Eigen::Vector3d vec(StabTask->measuredDCM());
        vec.z() = 0;
        return vec;
      }));

  gui()->addElement(
      {"Walking", "Visualization", "Trajectories"},
      mc_rtc::gui::Trajectory(
          "RealCoMTrajectory",
          mc_rtc::gui::LineConfig(mc_rtc::gui::Color(1., 0., 0.), 0.01, mc_rtc::gui::LineStyle::Dotted),
          [this]() {
            if(UseRealRobot)
            {
              return realRobot().com();
            }
            else
            {
              return robot().com();
            }
          }),
      mc_rtc::gui::Trajectory(
          "SwingFoot Trajectory",
          mc_rtc::gui::LineConfig(mc_rtc::gui::Color(0., 1., 0.), 0.01, mc_rtc::gui::LineStyle::Solid),
          [this]() { return robot().surfacePose(swingFootName).translation(); }),
      mc_rtc::gui::Trajectory(
          "ZMPMeasured", mc_rtc::gui::LineConfig(mc_rtc::gui::Color(0.5, 1., 0.), 0.01, mc_rtc::gui::LineStyle::Solid),
          [this]() { return StabTask->measuredZMP(); }),
      mc_rtc::gui::Trajectory(
          "Pzk", mc_rtc::gui::LineConfig(mc_rtc::gui::Color(0., 1., 0.), 0.01, mc_rtc::gui::LineStyle::Solid),
          [this]() {
            return mpc_state_.Pzk;
            ;
          }),
      mc_rtc::gui::Trajectory("Predicted ZMP Trajectory", mc_rtc::gui::Color(0., 0., 1.),
                              [this]() -> std::vector<Eigen::Vector3d> {
                                std::vector<Eigen::Vector3d> Output;
                                for(int k = 0; k < mpc_state_.X_MPC.size(); k++)
                                {
                                  Output.push_back(mpc_state_.Get_ZMP_planarTarget(k));
                                }
                                return Output;
                              }),
      mc_rtc::gui::Trajectory("Predicted CoM Trajectory", mc_rtc::gui::Color(1., 0., 0.),
                              [this]() -> std::vector<Eigen::Vector3d> {
                                std::vector<Eigen::Vector3d> Output;
                                for(int k = 0; k < mpc_state_.X_MPC.size(); k++)
                                {
                                  Output.push_back(mpc_state_.Get_CoM_planarTarget(k));
                                }
                                return Output;
                              }),

      mc_rtc::gui::Trajectory("Anticipative Trajectory", mc_rtc::gui::Color(1., 0., 1.),
                              [this]() -> std::vector<Eigen::Vector3d> {
                                std::vector<Eigen::Vector3d> Output;
                                Eigen::VectorXd Traj = MPCSolver.GetAfterTc_ZMP_trajectory();
                                int n = (int)(Traj.size() / 2);
                                for(int k = 0; k < n; k++)
                                {
                                  Output.push_back(Eigen::Vector3d{Traj(k), Traj(k + n), 0});
                                }
                                return Output;
                              }),

      mc_rtc::gui::Polygon(
          "AllPoly", mc_rtc::gui::Color(1., 0.3, 0.),
          [this]() -> const std::vector<std::vector<Eigen::Vector3d>> & { return this->MPCSolver.get_allpolys(); }),
      mc_rtc::gui::Polygon("SupportPolygon", mc_rtc::gui::Color(1., 1., 0.), [this]() -> const std::vector<Eigen::Vector3d> & { return mpc_state_.get_SupPolygon(); })

        );

 

  // gui()->addElement({"Walking ","Plots"}, mc_rtc::gui::ElementsStacking::Horizontal,
  //                Button("Plot CoM Tracking (x)",
  //                       [this]() {
  //                         gui()->addPlot("CoM Tracking (x)", plot::X("t", [this]() { return t_; }),
  //                                     mc_rtc::gui::plot::Y(
  //                                         "com_ref", [this]() { return comTarget_.x(); }, Color::Red),
  //                                     mc_rtc::gui::plot::Y(
  //                                         "com_mes", [this]() { return measuredCoM_.x(); }, Color::Magenta));
  //                       }),
  //                Button("Stop CoM (x)", [this]() { gui()->removePlot("CoM Tracking (x)"); }));
  // gui()->addElement({"Walking ","Plots"}, mc_rtc::gui::ElementsStacking::Horizontal,
  //                Button("Plot CoM Tracking (y)",
  //                       [this]() {
  //                         gui()->addPlot("CoM Tracking (y)", plot::X("t", [this]() { return t_; }),
  //                                     mc_rtc::gui::plot::Y(
  //                                         "com_ref", [this]() { return comTarget_.y(); }, Color::Red),
  //                                     mc_rtc::gui::plot::Y(
  //                                         "com_mes", [this]() { return measuredCoM_.y(); }, Color::Magenta));
  //                       }),
  //                Button("Stop CoM (y)", [this]() { gui()->removePlot("CoM Tracking (y)"); }));
};

void Walking_controller::add_FootSteps_GUI()
{

  auto footStepPolygon = [this](const sva::PTransformd & X_0_step,
                                const sva::PTransformd & X_0_support, std::string sup) {
    // double x = step.translation().x() ; 
    // double y = step.translation().y() ;
    // double theta = mc_rbdyn::rpyFromMat(step.rotation()).z();
    // Eigen::Vector3d footPos;
    // footPos << x, y, 0;
    // Eigen::Matrix3d footOri = mc_rbdyn::rpyToMat(0, 0, theta);
    // sva::PTransformd X_support_foot(footOri, footPos);
    const auto & surface = robot().surface(sup);
    const auto & points = surface.points();
    std::vector<Eigen::Vector3d> polygon;
    for(const auto & point : points)
    {
      const auto & X_foot_point = surface.X_b_s().inv() * point;
      Eigen::Vector3d p = (X_foot_point * X_0_step).translation();
      polygon.push_back(p);
    }
    return polygon;
  };

  gui()->addElement(
      {"Walking", "Visualization", "FootStep"},
      mc_rtc::gui::Polygon("Planned Footsteps", mc_rtc::gui::Color(0., 1., 0.),
                           [this, footStepPolygon]() -> std::vector<std::vector<Eigen::Vector3d>> {
                             
                             const std::vector<sva::PTransformd> & steps = mpc_state_.planned_steps();
                            //  mc_rtc::log::info("step {}",steps.size());
                             std::vector<std::vector<Eigen::Vector3d>> Output;
                             for(int k = 0; k < steps.size(); k++)
                             {
                               if(k % 2 == 1)
                               {
                                 Output.push_back(footStepPolygon(steps[k], this->X_0_support, this->supportFootName));
                               }
                               else
                               {
                                 Output.push_back(footStepPolygon(steps[k], this->X_0_support, this->swingFootName));
                               }
                             }
                             return Output;
                           }));
  gui()->addElement(
      {"Walking", "Visualization", "FootStep"},
      mc_rtc::gui::Polygon("Corrected Footsteps", mc_rtc::gui::Color(0., 0., 1.),
                           [this, footStepPolygon]() -> std::vector<std::vector<Eigen::Vector3d>> {
                             
                             const std::vector<sva::PTransformd> & steps_opti = mpc_state_.optimal_steps();

                             std::vector<std::vector<Eigen::Vector3d>> Output;
                            //  mc_rtc::log::info("step opti {}",steps_opti.size());

                             for(int k = 0; k < steps_opti.size(); k++)
                             {
    
                               if(k % 2 == 1)
                               {

                                 Output.push_back(footStepPolygon(steps_opti[k] , this->X_0_support, this->supportFootName));
                               }
                               else
                               {
                                 Output.push_back(footStepPolygon(steps_opti[k] , this->X_0_support, this->swingFootName));
                               }
                             }
                             return Output;
                           }));
}