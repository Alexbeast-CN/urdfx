cmake_minimum_required(VERSION 3.20)

project(urdfx VERSION 0.1.0 LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

FetchContent_Declare(
  pugixml
  GIT_REPOSITORY https://github.com/zeux/pugixml.git
  GIT_TAG v1.15
)
FetchContent_MakeAvailable(pugixml)

FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4 
)

# 下载并加入到构建系统
FetchContent_MakeAvailable(eigen)


# 添加可执行文件
add_executable(urdf_parser_app
    main.cpp
    urdf_parser.cpp
)

# 链接库
target_link_libraries(urdf_parser_app PRIVATE
    Eigen3::Eigen
    pugixml::pugixml
)

# 包含头文件目录
target_include_directories(urdf_parser_app PRIVATE
    ${EIGEN3_INCLUDE_DIR}
)