cmake_minimum_required(VERSION 3.0)
project(Class_line_follower LANGUAGES CXX)

add_executable(${PROJECT_NAME} src/main.cpp src/image_processing.cpp src/motor_driver.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_14)
target_include_directories(${PROJECT_NAME} PRIVATE include)

# OpenCV setup
find_package(OpenCV REQUIRED)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

