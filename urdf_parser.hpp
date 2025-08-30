#ifndef URDF_PARSER_HPP
#define URDF_PARSER_HPP

#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>


#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace URDF {

// 基本几何形状
struct Box {
  Eigen::Vector3d size;
};

struct Cylinder {
  double radius;
  double length;
};

struct Sphere {
  double radius;
};

struct Mesh {
  std::string filename;
  std::optional<Eigen::Vector3d> scale;
};

// 使用 std::variant 优雅地处理不同几何类型
using Geometry = std::variant<Box, Cylinder, Sphere, Mesh>;

// 惯性属性
struct Inertial {
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  double mass = 0.0;
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Identity();
};

// 可视化属性
struct Visual {
  std::string name;
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  Geometry geometry;
};

// 碰撞属性
struct Collision {
  std::string name;
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  Geometry geometry;
};

// 关节限制
struct Limit {
  double lower = 0.0;
  double upper = 0.0;
  double effort = 0.0;
  double velocity = 0.0;
};

// 关节动力学
struct Dynamics {
  double damping = 0.0;
  double friction = 0.0;
};

// 连杆 (Link)
struct Link {
  std::string name;
  std::optional<Inertial> inertial;
  std::vector<Visual> visuals;
  std::vector<Collision> collisions;
};

// 关节 (Joint)
struct Joint {
  std::string name;
  std::string type;
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  std::string parent_link;
  std::string child_link;
  std::optional<Eigen::Vector3d> axis;
  std::optional<Limit> limit;
  std::optional<Dynamics> dynamics;
};

// 机器人模型
struct Robot {
  std::string name;
  std::map<std::string, std::shared_ptr<Link>> links;
  std::map<std::string, std::shared_ptr<Joint>> joints;
};

// 解析器类
class Parser {
public:
  // 主解析函数，返回一个 optional<Robot>，如果解析失败则返回 std::nullopt
  std::optional<Robot> parse(const std::filesystem::path &urdf_path);
};

// << 操作符重载，方便打印调试 (声明)
std::ostream &operator<<(std::ostream &os, const Robot &robot);

} // namespace URDF

#endif // URDF_PARSER_HPP