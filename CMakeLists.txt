cmake_minimum_required(VERSION 3.8)
project(fake_sensors)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio)



add_executable(fake_lidar src/fake_lidar.cpp src/lidar_config.cpp)
add_executable(fake_camera src/fake_camera.cpp src/camera_config.cpp)

target_include_directories(fake_lidar PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_include_directories(fake_camera PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(fake_lidar PUBLIC c_std_99 cxx_std_20)
target_compile_features(fake_camera PUBLIC c_std_99 cxx_std_20)

ament_target_dependencies(fake_lidar
    rclcpp
    std_msgs
    sensor_msgs
    cv_bridge
    image_transport
)

ament_target_dependencies(fake_camera
    rclcpp
    std_msgs
    sensor_msgs
    cv_bridge
    image_transport
)

target_link_libraries(fake_lidar ${OpenCV_LIBS})
target_link_libraries(fake_camera ${OpenCV_LIBS})

install(TARGETS fake_lidar
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS fake_camera
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()

