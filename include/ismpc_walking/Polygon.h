#pragma once
#include <SpaceVecAlg/SpaceVecAlg>
#include <Eigen/StdVector>
#include <eigen3/Eigen/Dense>

// clang-format off
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
#include <boost/geometry/multi/geometries/register/multi_polygon.hpp>
// clang-format on

inline Eigen::Vector3d rpyFromMat(const Eigen::Matrix3d & E)
{
  double roll = atan2(E(1, 2), E(2, 2));
  double pitch = -asin(E(0, 2));
  double yaw = atan2(E(0, 1), E(0, 0));
  return Eigen::Vector3d(roll, pitch, yaw);
}

/**
 * @brief Rectangle class allows to generate easily a representation of a contact location with a defined size and
 * orientation
 *
 */
struct Rectangle
{

public:
  Rectangle(double ori, const Eigen::Vector2d size, const Eigen::Vector3d offset = Eigen::Vector3d::Zero())
  {
    _center = offset;
    _angle = ori;
    _size.segment(0, 2) = size;
    compute_rect();
  }

  Rectangle(const sva::PTransformd & pose,
            const Eigen::Vector2d & size,
            const Eigen::Vector3d offset = Eigen::Vector3d::Zero())
  {
    _center = pose.translation() + offset;
    _center.z() = 0;
    _angle = rpyFromMat(pose.rotation()).z();
    _size.segment(0, 2) = size;
    compute_rect();
  }

  Rectangle(const Eigen::Vector3d & center,
            const Eigen::Vector2d & size,
            const Eigen::Vector3d offset = Eigen::Vector3d::Zero())
  {
    _center = center + offset;
    _angle = _center.z();
    _size.segment(0, 2) = size;
    _center.z() = 0;
    compute_rect();
  }
  void compute_rect()
  {

    R << cos(_angle), -sin(_angle), 0, sin(_angle), cos(_angle), 0, 0, 0, 1;

    upper_left_corner = _center + R * Eigen::Vector3d{-_size.x() / 2, _size.y() / 2, 0};
    upper_right_corner = _center + R * Eigen::Vector3d{_size.x() / 2, _size.y() / 2, 0};
    lower_left_corner = _center + R * Eigen::Vector3d{-_size.x() / 2, -_size.y() / 2, 0};
    lower_right_corner = _center + R * Eigen::Vector3d{_size.x() / 2, -_size.y() / 2, 0};
    corners = {upper_left_corner, upper_right_corner, lower_right_corner, lower_left_corner};
  }
  void add_offset(const Eigen::Vector3d offset)
  {
    _center += Eigen::Vector3d{offset.x(), offset.y(), 0};
    compute_rect();
  }
  ~Rectangle()
  {
    corners.clear();
    _center.setZero();
    _size.setZero();
    _angle = 0.;
    R = Eigen::Matrix3d::Identity();
    upper_left_corner.setZero();
    upper_right_corner.setZero();
    lower_left_corner.setZero();
    lower_right_corner.setZero();
  }

  std::vector<Eigen::Vector3d> & Get_corners()
  {
    return corners;
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
  std::vector<Eigen::Vector3d> corners;
};

struct vec3d_x_comp
{
  inline bool operator()(const Eigen::Vector3d & struct1, const Eigen::Vector3d & struct2)
  {
    return (struct1.x() < struct2.x());
  }
};

/**
 * @brief SupportPolygon class allow to convert any generated rectangles or pair of rectangles into an inequality
 * constraint of the form N * p < O where N is a (n x 2) matrix
 */
struct SupportPolygon
{

public:
  SupportPolygon() = default;
  SupportPolygon(const std::vector<Eigen::Vector3d> & corners)
  {
    SupportPolygone_Corners = corners;
    Compute_polygone();
  }
  SupportPolygon(const Rectangle Rect1, const Rectangle Rect2)
  {
    _Rectangles = {Rect1, Rect2};
    Get_corners();
    Compute_polygone();
  }
  SupportPolygon(const Rectangle Rect1)
  {
    _Rectangles = {Rect1};
    Get_corners();
    Compute_polygone();
  }
  SupportPolygon(const Eigen::MatrixX2d normals, Eigen::VectorXd offsets)
  {
    SupportPolygone_Normals = normals;
    Offset = offsets;
    cstr_to_polygone();
  }

  ~SupportPolygon()
  {
    _Rectangles.clear();
    _corners.clear();
    SupportPolygone_Corners.clear();
  }

  void jarvis_march();
  void convex_hull();

  std::vector<Eigen::Vector3d> & Get_Polygone_Corners()
  {
    return SupportPolygone_Corners;
  }

  const Eigen::MatrixX2d & normals()
  {
    return SupportPolygone_Normals;
  }

  const Eigen::VectorXd & offsets()
  {
    return Offset;
  }

  Eigen::Vector3d get_center()
  {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double n = static_cast<double>(_Rectangles.size());
    for(auto & r : _Rectangles)
    {
      center += (1 / n) * r.get_center();
    }
    return center;
  }

  Rectangle & get_Rectangle(int indx)
  {
    if(indx < static_cast<int>(_Rectangles.size()) - 1)
    {
      return _Rectangles[static_cast<size_t>(indx)];
    }
    else
    {
      return _Rectangles[0];
    }
  }

private:
  void Get_corners()
  {
    if(_Rectangles.size() > 1)
    {
      // for (int r = 0 ; r < _Rectangles.size() ; r++)
      // {
      //     std::vector<Eigen::Vector3d> corners = _Rectangles[r].Get_corners();
      //     _corners.insert(_corners.end(),corners.begin(),corners.end());
      // }
      // jarvis_march();
      convex_hull();
    }
    else
    {
      SupportPolygone_Corners = _Rectangles[0].Get_corners();
    }
  }

  void Compute_polygone()
  {

    SupportPolygone_Normals.resize(SupportPolygone_Corners.size(), 2);
    SupportPolygone_Edges_Center.resize(SupportPolygone_Corners.size(), 2);
    SupportPolygone_Vertices.resize(SupportPolygone_Corners.size(), 2);
    Offset.resize(SupportPolygone_Corners.size());
    Eigen::Matrix2d R_Vertices_0;

    for(size_t c = 0; c < SupportPolygone_Corners.size(); c++)
    {
      const Eigen::Vector3d & point_1 = SupportPolygone_Corners[c];
      const Eigen::Vector3d & point_2 = SupportPolygone_Corners[(c + 1) % SupportPolygone_Corners.size()];
      const Eigen::Vector3d vertice = (point_2 - point_1).normalized();
      const Eigen::Vector3d normal = vertical_vec.cross(vertice);
      SupportPolygone_Normals(c, 0) = normal.x();
      SupportPolygone_Normals(c, 1) = normal.y();
      SupportPolygone_Vertices(c, 0) = vertice.x();
      SupportPolygone_Vertices(c, 1) = vertice.y();
      SupportPolygone_Edges_Center(c, 0) = (((point_2 + point_1) / 2)).x();
      SupportPolygone_Edges_Center(c, 1) = (((point_2 + point_1) / 2)).y();

      R_Vertices_0 << SupportPolygone_Normals(c, 0), SupportPolygone_Vertices(c, 0), SupportPolygone_Normals(c, 1),
          SupportPolygone_Vertices(c, 1);

      Offset(c) = (R_Vertices_0.transpose() * SupportPolygone_Edges_Center.block(c, 0, 1, 2).transpose())(0);
    }
  }

  void cstr_to_polygone()
  {

    std::vector<int> vertices_indx;
    for(Eigen::Index i = 0; i < SupportPolygone_Normals.rows(); i++)
    {
      Eigen::Index end_indx = (i + 1) % SupportPolygone_Normals.rows();
      Eigen::Vector2d ni = SupportPolygone_Normals.block(i, 0, 1, 2).normalized();
      Eigen::Vector2d nip1 = SupportPolygone_Normals.block(end_indx, 0, 1, 2).normalized();
      // mc_rtc::log::info("normal {}\n{}\nnext_normal{}\ndot prod {}",i,ni,nip1,ni.transpose() * nip1);
      if(std::abs(ni.transpose() * nip1 - 1) > 1e-4)
      {

        vertices_indx.push_back(static_cast<int>(i));
        // mc_rtc::log::info("selected");
      }
    }
    // mc_rtc::log::info("corner {} selected {}",SupportPolygone_Normals.rows(),normals.size());
    for(size_t i = 0; i < vertices_indx.size(); i++)
    {
      Eigen::Index start_indx = vertices_indx[i];
      Eigen::Index end_indx = vertices_indx[(i + 1) % vertices_indx.size()];
      Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
      Eigen::Vector2d o = Eigen::Vector2d::Zero();
      R.block(0, 0, 1, 2) = SupportPolygone_Normals.block(start_indx, 0, 1, 2);
      R.block(1, 0, 1, 2) = SupportPolygone_Normals.block(end_indx, 0, 1, 2);
      o.x() = Offset[start_indx];
      o.y() = Offset[end_indx];
      if(R.determinant() != 0)
      {
        Eigen::Vector2d p = R.inverse() * o;
        // mc_rtc::log::info("start {} ; end {} point {}",i,end_indx,p);
        // mc_rtc::log::info("R {}\no {}",R,o);

        SupportPolygone_Corners.push_back(Eigen::Vector3d{p.x(), p.y(), 0});
      }
    }
  }

  Eigen::Vector3d vertical_vec = Eigen::Vector3d{0, 0, 1};
  std::vector<Rectangle> _Rectangles;
  std::vector<Eigen::Vector3d> _corners;
  std::vector<Eigen::Vector3d> SupportPolygone_Corners;
  Eigen::MatrixX2d SupportPolygone_Vertices;
  Eigen::MatrixX2d SupportPolygone_Edges_Center;
  Eigen::MatrixX2d SupportPolygone_Normals;
  Eigen::VectorXd Offset;
};