cmake_minimum_required(VERSION 3.8)
# Tệp này mô tả cách BIÊN DỊCH/CÀI ĐẶT. Nó không chạy P01 hay mô phỏng UAV.
# Tên package p01 thống nhất với <name> trong package.xml.
project(p01)

# C++17 cần cho std::clamp và std::void_t trong các header đang tái sử dụng.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ament_cmake: tích hợp CMake với ROS 2; rclcpp: API viết node bằng C++.
# px4_msgs cung cấp kiểu thông điệp, không tự tạo dữ liệu cảm biến.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)


# rcl_interfaces cung cấp mô tả tham số ROS; Eigen xử lý vector/ma trận P01.
# eigen3_cmake_module giúp ROS tìm Eigen3 qua CMake.
find_package(rcl_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)



if(BUILD_TESTING)
  # Kiểm tra logic cất cánh thuần C++: không phát lệnh tới PX4.
  enable_testing()
  add_executable(test_ground_takeoff test/test_ground_takeoff.cpp)
  target_include_directories(test_ground_takeoff PRIVATE include)
  target_link_libraries(test_ground_takeoff Eigen3::Eigen)
  add_test(NAME ground_takeoff_core COMMAND test_ground_takeoff)

  # Các công cụ lint có sẵn từ ros2 pkg create.
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Các header inline được include khi biên dịch; mỗi header không phải một node.
add_executable(p01_offboard_hover src/p01_offboard_hover.cpp)

# Cho phép tìm include/uav_research/*.hpp ngay trong package p01 này.
# Tên namespace/include giữ từ nguồn cũ; không phụ thuộc package ROS uav_research.
target_include_directories(p01_offboard_hover PRIVATE include)

# Khai báo phụ thuộc ROS cho executable; Eigen liên kết qua target riêng bên dưới.
ament_target_dependencies(p01_offboard_hover
  rclcpp
  rcl_interfaces
  px4_msgs
    geometry_msgs
  std_msgs
)

target_link_libraries(p01_offboard_hover Eigen3::Eigen)

# Cài vào lib/p01 để ROS tìm được khi dùng ros2 run p01 p01_offboard_hover.
install(TARGETS p01_offboard_hover
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(p01_path_visualizer src/p01_path_visualizer.cpp)

ament_target_dependencies(p01_path_visualizer
  rclcpp
  geometry_msgs
  nav_msgs
)

install(TARGETS p01_path_visualizer
  DESTINATION lib/${PROJECT_NAME}
)
# Node P01 đơn giản để học: cùng lõi toán, bỏ lớp an toàn. Chỉ dùng trong Gazebo.
add_executable(p01_simple src/p01_simple.cpp)
target_include_directories(p01_simple PRIVATE include)
ament_target_dependencies(p01_simple rclcpp px4_msgs geometry_msgs std_msgs)
target_link_libraries(p01_simple Eigen3::Eigen)
install(TARGETS p01_simple DESTINATION lib/${PROJECT_NAME})

# Node P02 đơn giản (PID hình học, arXiv:1304.6765v1): chép p01_simple, đổi lõi sang P02Controller. Chỉ dùng trong Gazebo.
add_executable(p02_simple src/p02_simple.cpp)
target_include_directories(p02_simple PRIVATE include)
ament_target_dependencies(p02_simple rclcpp px4_msgs geometry_msgs std_msgs)
target_link_libraries(p02_simple Eigen3::Eigen)
install(TARGETS p02_simple DESTINATION lib/${PROJECT_NAME})

# Đăng ký thông tin package với ament; giữ ở cuối cấu hình.
ament_package()
