cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install directories (gộp urdf, meshes, launch, rviz vào một lệnh)
install(
  DIRECTORY
    urdf
    meshes
    launch
    rviz  # nếu bạn có thư mục rviz, nếu chưa có thì bỏ dòng này cũng được
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
