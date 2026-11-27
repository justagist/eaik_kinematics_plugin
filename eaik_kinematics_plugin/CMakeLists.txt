cmake_minimum_required(VERSION 3.16)
project(eaik_kinematics_plugin LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eaik_vendor REQUIRED)
find_package(EAIK REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

generate_parameter_library(eaik_kinematics_parameters src/eaik_kinematics_parameters.yaml)

add_library(${PROJECT_NAME} SHARED src/eaik_kinematics_plugin.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} PRIVATE
  eaik_kinematics_parameters
  EAIK::EAIK
  Eigen3::Eigen
  moveit_core::moveit_kinematics_base
  moveit_core::moveit_robot_model
  moveit_core::moveit_robot_state
  pluginlib::pluginlib
  rclcpp::rclcpp
)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
pluginlib_export_plugin_description_file(moveit_core eaik_kinematics_plugin_description.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(srdfdom REQUIRED)
  find_package(tf2_eigen REQUIRED)

  ament_add_gtest(test_eaik_kinematics_plugin test/test_eaik_kinematics_plugin.cpp SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_eaik_kinematics_plugin
    moveit_core::moveit_kinematics_base
    moveit_core::moveit_robot_model
    moveit_core::moveit_robot_state
    moveit_core::moveit_test_utils
    pluginlib::pluginlib
    rclcpp::rclcpp
    srdfdom::srdfdom
    tf2_eigen::tf2_eigen
  )

  # Needs moveit_resources and the compared plugins at run time, so it is built with the tests.
  add_executable(benchmark_eaik_kinematics benchmark/benchmark_eaik_kinematics.cpp)
  target_link_libraries(benchmark_eaik_kinematics
    moveit_core::moveit_kinematics_base
    moveit_core::moveit_robot_model
    moveit_core::moveit_robot_state
    moveit_core::moveit_test_utils
    pluginlib::pluginlib
    rclcpp::rclcpp
    srdfdom::srdfdom
    tf2_eigen::tf2_eigen
  )
  install(TARGETS benchmark_eaik_kinematics RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
