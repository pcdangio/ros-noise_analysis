cmake_minimum_required(VERSION 3.0.2)
project(noise_analysis)

# Find catkin and package dependencies.
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_introspection)

# Find ALGLIB
find_package(ALGLIB REQUIRED)

# Set up Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
find_package(Qt5 REQUIRED COMPONENTS
    Widgets
    Charts)

# Generate catkin package.
catkin_package()

# Set up build include directories.
include_directories(
  src
  ${catkin_INCLUDE_DIRS})

# Build noise_analysis executable.
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/gui/form_main.cpp
    src/gui/form_main.ui
    src/gui/form_array.cpp
    src/gui/form_array.ui
    src/data/data_interface.cpp
    src/data/dataset.cpp
    src/data/candidate_field.cpp
    src/graph/chart.cpp)

# Link noise_analysis executable.
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Widgets
  Qt5::Charts
  ${ALGLIB_LIB}
)

# Set up install for noise_analysis.
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
