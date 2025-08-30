#include "urdf_parser.hpp"
#include <iostream>

int main(const int argc, char* argv[]) {
  // 检查命令行参数
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <urdf_file_path>" << std::endl;
    return 1;
  }

  // 创建一个解析器实例
  URDF::Parser parser;

  // 从命令行参数获取 URDF 文件路径
  std::filesystem::path urdf_file = argv[1];

  // 调用解析函数

  // 检查解析是否成功
  if (const std::optional<URDF::Robot> robot_opt = parser.parse(urdf_file)) {
    // 如果成功，打印解析出的机器人模型信息
    const URDF::Robot &robot = *robot_opt;
    std::cout << "Successfully parsed URDF file: " << urdf_file << "\n\n";
    std::cout << robot << std::endl;
  } else {
    std::cerr << "Failed to parse URDF file: " << urdf_file << std::endl;
    return 1;
  }

  return 0;
}