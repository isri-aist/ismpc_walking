#include "Walking_controller.h"

inline double floorn(double x, int n)
{
  return floor(pow(10, n) * x) / pow(10, n);
}

void Walking_controller::addToGUI()
{
  auto footStepPolygon = [this](const Eigen::Vector3d & footstep, const sva::PTransformd & X_0_support,
                                std::string sup) {
    const auto & footStep = footstep;
    Eigen::Vector3d footPos;
    footPos << footStep.x(), footStep.y(), 0;
    Eigen::Matrix3d footOri = mc_rbdyn::rpyToMat(0, 0, footStep.z());
    sva::PTransformd X_support_foot(footOri, footPos);
    const auto & surface = robot().surface(sup);
    const auto & points = surface.points();
    std::vector<Eigen::Vector3d> polygon;
    for(const auto & point : points)
    {
      const auto & X_foot_point = surface.X_b_s().inv() * point;
      Eigen::Vector3d p = (X_foot_point * X_support_foot).translation();
      polygon.push_back(p);
    }
    return polygon;
  };

  gui()->addElement({"Walking", "Action"},

                    mc_rtc::gui::Button("Start Move",
                                        [this]() {
                                          t_k = 0;
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
                        "Real Robot Data", [this]() { return UseRealRobot; },
                        [this]() {
                          UseRealRobot = !UseRealRobot;
                          ComputeTrajectoryOnce = true;
                        }),
                    // mc_rtc::gui::Checkbox("ZMP_Corr", [this](){return ZMP_correction;}, [this](){ZMP_correction =
                    // !ZMP_correction;}),
                    mc_rtc::gui::Label("MPC Processing Time (ms)", [this]() { return this->ProcessTime; }),
                    mc_rtc::gui::Label("Run Loop Processing Time (ms)", [this]() { return this->ControllerLoopTime; })
                    // mc_rtc::gui::Label("ZMP box range x",[this](){return this->MPCSolver.ZMP_dx;}),
                    // mc_rtc::gui::Label("ZMP box range y",[this](){return this->MPCSolver.ZMP_dy;})
  );

  gui()->addElement(
      {"Walking", "Footsteps Parameters"},
      mc_rtc::gui::NumberInput(
          "Vx", [this]() -> double { return Vx_i; }, [this](const double & v) { Vx_i = v; }),
      mc_rtc::gui::NumberInput(
          "Vy", [this]() -> double { return Vy_i; }, [this](const double & v) { Vy_i = v; }),
      mc_rtc::gui::NumberInput(
          "Omega", [this]() -> double { return Omega_i; }, [this](const double & v) { Omega_i = v; }),
      mc_rtc::gui::NumberInput(
          "x", [this]() -> double { return x; }, [this](const double & v) { x = v; }),
      mc_rtc::gui::NumberInput(
          "y", [this]() -> double { return y; }, [this](const double & v) { y = v; }),
      mc_rtc::gui::NumberInput(
          "theta", [this]() -> double { return z; }, [this](const double & v) { z = v; }),

      mc_rtc::gui::NumberInput(
          "Ts", [this]() -> double { return T_Steps; }, [this](const double & v) { T_Steps = v; }),
      mc_rtc::gui::NumberInput(
          "Steps", [this]() -> double { return N_Steps_Desired; }, [this](const double & v) { N_Steps_Desired = v; }),

      mc_rtc::gui::Point3D("Steps desired", mc_rtc::gui::PointConfig(mc_rtc::gui::Color(1, 0, 1)),
                           [this]() {
                             return Eigen::Vector3d{x, y, 0};
                           }),
      mc_rtc::gui::Button("Compute Trajectory", [this]() { ComputeTrajectoryOnce = true; }));

  gui()->addElement({"Walking", "Visualization", "Reference Trajectory"},
                    mc_rtc::gui::Trajectory("Trajectory", mc_rtc::gui::Color(1., 1., 0.), [this]() -> const std::vector<Eigen::Vector3d> & { return mpc_state_.get_RefTraj(); }));

  gui()->addElement(
      {"Walking", "Visualization", "ZMP Range"},
      mc_rtc::gui::Point3D("P_z_k", mc_rtc::gui::PointConfig(mc_rtc::gui::Color(0.5, 1, 1)),
                           [this]() {
                             return Eigen::Vector3d{Pzk.x(), Pzk.y(), 0.05};
                           }),

      mc_rtc::gui::Point3D("ZMP_MPC", mc_rtc::gui::PointConfig(mc_rtc::gui::Color(1, 0.5, 0.25)), [this]() -> Eigen::Vector3d {
        return mpc_state_.Get_ZMP_planarTarget(Index);
      }));

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
          "CoMTrajectory",
          mc_rtc::gui::LineConfig(mc_rtc::gui::Color(1., 1., 0.), 0.01, mc_rtc::gui::LineStyle::Dotted),
          [this]() {
            return  mpc_state_.Get_CoM_planarTarget(Index);
          }),
      mc_rtc::gui::Trajectory(
          "ZMPMeasured", mc_rtc::gui::LineConfig(mc_rtc::gui::Color(0.5, 1., 0.), 0.01, mc_rtc::gui::LineStyle::Solid),
          [this]() { return StabTask->measuredZMP(); }),
      mc_rtc::gui::Trajectory(
          "Pzk", mc_rtc::gui::LineConfig(mc_rtc::gui::Color(0., 1., 0.), 0.01, mc_rtc::gui::LineStyle::Solid),
          [this]() {
            return Pzk;
            ;
          }),
      mc_rtc::gui::Trajectory("Predicted ZMP Trajectory", mc_rtc::gui::Color(0., 0., 1.),
                              [this]() -> const std::vector<Eigen::Vector3d> { return predictedZMPWorld; }),
      mc_rtc::gui::Polygon("SupportPolygon", mc_rtc::gui::Color(1., 1., 0.), [this]() -> const std::vector<Eigen::Vector3d> & { return mpc_state_.get_SupPolygon(); }),
      mc_rtc::gui::Trajectory("Predicted CoM Trajectory", mc_rtc::gui::Color(1., 0., 0.),
                              [this]() -> const std::vector<Eigen::Vector3d> { return predictedCoMWorld; })
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

  auto footStepPolygon = [this](const Eigen::Vector3d & footstep, const sva::PTransformd & X_0_support,
                                std::string sup) {
    const auto & footStep = footstep;
    Eigen::Vector3d footPos;
    footPos << footStep.x(), footStep.y(), 0;
    Eigen::Matrix3d footOri = mc_rbdyn::rpyToMat(0, 0, footStep.z());
    sva::PTransformd X_support_foot(footOri, footPos);
    const auto & surface = robot().surface(sup);
    const auto & points = surface.points();
    std::vector<Eigen::Vector3d> polygon;
    for(const auto & point : points)
    {
      const auto & X_foot_point = surface.X_b_s().inv() * point;
      Eigen::Vector3d p = (X_foot_point * X_support_foot).translation();
      polygon.push_back(p);
    }
    return polygon;
  };
  Eigen::VectorXd xf = mpc_state_.Xf;
  Eigen::VectorXd yf = mpc_state_.Yf;
  Eigen::VectorXd thetaf = mpc_state_.Thetaf;
  Eigen::VectorXd xf_Corr = mpc_state_.Xf_Corr;
  Eigen::VectorXd yf_Corr = mpc_state_.Yf_Corr;
  for(int k = 0; k < xf.size(); k++)
  {
    if(k % 2 == 1)
    {
      gui()->addElement(
          {"Walking", "Visualization", "FootStep"},
          mc_rtc::gui::Polygon(
              "NextFootStep" + std::to_string(k), mc_rtc::gui::Color(0., 1., 0.),
              [this, k, footStepPolygon, xf, yf, thetaf]() {
                return footStepPolygon(Eigen::Vector3d{xf(k), yf(k), thetaf(k)}, X_0_support, supportFootName);
              }));
    }
    else
    {
      gui()->addElement(
          {"Walking", "Visualization", "FootStep"},
          mc_rtc::gui::Polygon(
              "NextFootStep" + std::to_string(k), mc_rtc::gui::Color(0., 1., 0.),
              [this, k, footStepPolygon, xf, yf, thetaf]() {
                return footStepPolygon(Eigen::Vector3d{xf(k), yf(k), thetaf(k)}, X_0_support, swingFootName);
              }));
    }
  }
  for(int k = 0; k < 2; k++)
  {
    if(k % 2 == 1)
    {
      gui()->addElement({"Walking", "Visualization", "FootStep"},
                        mc_rtc::gui::Polygon("Corrected Footstep" + std::to_string(k), mc_rtc::gui::Color(0., 0., 1.),
                                             [this, k, footStepPolygon, xf_Corr, yf_Corr, thetaf]() {
                                               return footStepPolygon(
                                                   Eigen::Vector3d{xf_Corr(k), yf_Corr(k), thetaf(k)}, X_0_support,
                                                   supportFootName);
                                             }));
    }
    else
    {
      gui()->addElement(
          {"Walking", "Visualization", "FootStep"},
          mc_rtc::gui::Polygon(
              "Corrected Footstep" + std::to_string(k), mc_rtc::gui::Color(0., 0., 1.),
              [this, k, footStepPolygon, xf_Corr, yf_Corr, thetaf]() {
                return footStepPolygon(Eigen::Vector3d{xf_Corr(k), yf_Corr(k), thetaf(k)}, X_0_support, swingFootName);
              }));
    }
  }
}