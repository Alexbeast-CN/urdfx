#include "urdf_parser.hpp"
#include "pugixml.hpp"
#include <sstream>

namespace URDF {

namespace { // 匿名命名空间，用于内部辅助函数

// 从 "x y z" 格式的字符串解析 Eigen::Vector3d
Eigen::Vector3d parse_vector3d(const std::string &str) {
  Eigen::Vector3d vec;
  std::stringstream ss(str);
  ss >> vec.x() >> vec.y() >> vec.z();
  return vec;
}

// 解析 <origin> 标签，返回 Eigen::Isometry3d
Eigen::Isometry3d parse_origin(const pugi::xml_node &node) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  if (node) {
    auto xyz_attr = node.attribute("xyz");
    if (xyz_attr) {
      transform.translation() = parse_vector3d(xyz_attr.as_string());
    }
    auto rpy_attr = node.attribute("rpy");
    if (rpy_attr) {
      Eigen::Vector3d rpy = parse_vector3d(rpy_attr.as_string());
      transform.rotate(Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()));
    }
  }
  return transform;
}

// 解析 <inertial> 标签
std::optional<Inertial> parse_inertial(const pugi::xml_node &node) {
  if (!node)
    return std::nullopt;

  Inertial inertial;
  inertial.origin = parse_origin(node.child("origin"));
  inertial.mass = node.child("mass").attribute("value").as_double();

  auto inertia_node = node.child("inertia");
  if (inertia_node) {
    inertial.inertia(0, 0) = inertia_node.attribute("ixx").as_double();
    inertial.inertia(0, 1) = inertia_node.attribute("ixy").as_double();
    inertial.inertia(0, 2) = inertia_node.attribute("ixz").as_double();
    inertial.inertia(1, 1) = inertia_node.attribute("iyy").as_double();
    inertial.inertia(1, 2) = inertia_node.attribute("iyz").as_double();
    inertial.inertia(2, 2) = inertia_node.attribute("izz").as_double();
    // 惯性矩阵是对称的
    inertial.inertia(1, 0) = inertial.inertia(0, 1);
    inertial.inertia(2, 0) = inertial.inertia(0, 2);
    inertial.inertia(2, 1) = inertial.inertia(1, 2);
  }
  return inertial;
}

// 解析 <geometry> 标签
Geometry parse_geometry(const pugi::xml_node &node) {
  if (auto box_node = node.child("box")) {
    return Box{parse_vector3d(box_node.attribute("size").as_string())};
  }
  if (auto cylinder_node = node.child("cylinder")) {
    return Cylinder{cylinder_node.attribute("radius").as_double(),
                    cylinder_node.attribute("length").as_double()};
  }
  if (auto sphere_node = node.child("sphere")) {
    return Sphere{sphere_node.attribute("radius").as_double()};
  }
  if (auto mesh_node = node.child("mesh")) {
    Mesh mesh;
    mesh.filename = mesh_node.attribute("filename").as_string();
    if (auto scale_attr = mesh_node.attribute("scale")) {
      mesh.scale = parse_vector3d(scale_attr.as_string());
    }
    return mesh;
  }
  // 默认或错误情况返回一个空的 Box
  return Box{Eigen::Vector3d::Zero()};
}

// 解析 <visual> 标签
Visual parse_visual(const pugi::xml_node &node) {
  Visual visual;
  visual.name = node.attribute("name").as_string();
  visual.origin = parse_origin(node.child("origin"));
  visual.geometry = parse_geometry(node.child("geometry"));
  return visual;
}

// 解析 <collision> 标签
Collision parse_collision(const pugi::xml_node &node) {
  Collision collision;
  collision.name = node.attribute("name").as_string();
  collision.origin = parse_origin(node.child("origin"));
  collision.geometry = parse_geometry(node.child("geometry"));
  return collision;
}

// 解析 <link> 标签
std::shared_ptr<Link> parse_link(const pugi::xml_node &node) {
  auto link = std::make_shared<Link>();
  link->name = node.attribute("name").as_string();
  link->inertial = parse_inertial(node.child("inertial"));
  for (const auto &visual_node : node.children("visual")) {
    link->visuals.push_back(parse_visual(visual_node));
  }
  for (const auto &collision_node : node.children("collision")) {
    link->collisions.push_back(parse_collision(collision_node));
  }
  return link;
}

// 解析 <joint> 标签
std::shared_ptr<Joint> parse_joint(const pugi::xml_node &node) {
  auto joint = std::make_shared<Joint>();
  joint->name = node.attribute("name").as_string();
  joint->type = node.attribute("type").as_string();
  joint->origin = parse_origin(node.child("origin"));
  joint->parent_link = node.child("parent").attribute("link").as_string();
  joint->child_link = node.child("child").attribute("link").as_string();

  if (auto axis_node = node.child("axis")) {
    joint->axis = parse_vector3d(axis_node.attribute("xyz").as_string());
  }
  if (auto limit_node = node.child("limit")) {
    joint->limit = Limit{limit_node.attribute("lower").as_double(),
                         limit_node.attribute("upper").as_double(),
                         limit_node.attribute("effort").as_double(),
                         limit_node.attribute("velocity").as_double()};
  }
  if (auto dynamics_node = node.child("dynamics")) {
    joint->dynamics =
        Dynamics{dynamics_node.attribute("damping").as_double(0.0),
                 dynamics_node.attribute("friction").as_double(0.0)};
  }
  return joint;
}

} // namespace

// Parser 类的主函数实现
std::optional<Robot> Parser::parse(const std::filesystem::path &urdf_path) {
  if (!std::filesystem::exists(urdf_path)) {
    std::cerr << "Error: URDF file not found at " << urdf_path << std::endl;
    return std::nullopt;
  }

  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_file(urdf_path.c_str());

  if (!result) {
    std::cerr << "Error parsing XML file: " << result.description()
              << std::endl;
    return std::nullopt;
  }

  pugi::xml_node robot_node = doc.child("robot");
  if (!robot_node) {
    std::cerr << "Error: <robot> tag not found in URDF file." << std::endl;
    return std::nullopt;
  }

  Robot robot;
  robot.name = robot_node.attribute("name").as_string();

  for (const auto &link_node : robot_node.children("link")) {
    auto link = parse_link(link_node);
    robot.links[link->name] = link;
  }

  for (const auto &joint_node : robot_node.children("joint")) {
    auto joint = parse_joint(joint_node);
    robot.joints[joint->name] = joint;
  }

  return robot;
}

// --- 打印函数实现 ---
std::ostream &operator<<(std::ostream &os, const Eigen::Vector3d &vec) {
  os << "[" << vec.x() << ", " << vec.y() << ", " << vec.z() << "]";
  return os;
}

std::ostream &operator<<(std::ostream &os, const Eigen::Isometry3d &T) {
  os << "T: " << T.translation().transpose()
     << ", RPY: " << T.rotation().eulerAngles(2, 1, 0).reverse().transpose();
  return os;
}

void print_geometry(std::ostream &os, const Geometry &geo) {
  std::visit(
      [&os](auto &&arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, Box>) {
          os << "Box(size: " << arg.size.transpose() << ")";
        } else if constexpr (std::is_same_v<T, Cylinder>) {
          os << "Cylinder(radius: " << arg.radius << ", length: " << arg.length
             << ")";
        } else if constexpr (std::is_same_v<T, Sphere>) {
          os << "Sphere(radius: " << arg.radius << ")";
        } else if constexpr (std::is_same_v<T, Mesh>) {
          os << "Mesh(filename: " << arg.filename;
          if (arg.scale) {
            os << ", scale: " << arg.scale->transpose();
          }
          os << ")";
        }
      },
      geo);
}

std::ostream &operator<<(std::ostream &os, const Robot &robot) {
  os << "=======================================\n";
  os << "Robot Name: " << robot.name << "\n";
  os << "=======================================\n\n";

  os << "---------- Links (" << robot.links.size() << ") ----------\n";
  for (const auto &[name, link] : robot.links) {
    os << "Link: " << name << "\n";
    if (link->inertial) {
      os << "  - Inertial:\n";
      os << "    - Origin: " << link->inertial->origin << "\n";
      os << "    - Mass: " << link->inertial->mass << "\n";
      os << "    - Inertia:\n" << link->inertial->inertia << "\n";
    }
    for (const auto &visual : link->visuals) {
      os << "  - Visual: " << visual.name << "\n";
      os << "    - Origin: " << visual.origin << "\n";
      os << "    - Geometry: ";
      print_geometry(os, visual.geometry);
      os << "\n";
    }
  }
  os << "\n---------- Joints (" << robot.joints.size() << ") ----------\n";
  for (const auto &[name, joint] : robot.joints) {
    os << "Joint: " << name << " (Type: " << joint->type << ")\n";
    os << "  - Parent: " << joint->parent_link
       << " -> Child: " << joint->child_link << "\n";
    os << "  - Origin: " << joint->origin << "\n";
    if (joint->axis) {
      os << "  - Axis: " << *joint->axis << "\n";
    }
    if (joint->limit) {
      os << "  - Limit: [lower=" << joint->limit->lower
         << ", upper=" << joint->limit->upper
         << ", effort=" << joint->limit->effort
         << ", velocity=" << joint->limit->velocity << "]\n";
    }
  }
  os << "=======================================\n";
  return os;
}

} // namespace URDF