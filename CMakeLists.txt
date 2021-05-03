cmake_minimum_required(VERSION 3.5)
project(astra_camera)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    #  add_compile_options(-Wall -Wextra)
    add_compile_options(-Wno-deprecated)
    add_compile_options(-Wno-error)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(libuvc REQUIRED)

option(FILTER "OB Filter library" ON)

#MACHINE = $(shell uname -m)
execute_process(COMMAND uname -m OUTPUT_VARIABLE MACHINES)
execute_process(COMMAND getconf LONG_BIT OUTPUT_VARIABLE MACHINES_BIT)
MESSAGE(STATUS "ORRBEC Machine : ${MACHINES}")
MESSAGE(STATUS "ORRBEC Machine Bits : ${MACHINES_BIT}")
IF ((${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "64"))
    set(HOST_PLATFORM "x64")
    # for schroot enviroment
ELSEIF ((${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "32"))
    set(HOST_PLATFORM "x86")
ELSEIF (${MACHINES} MATCHES "x86")
ELSEIF (${MACHINES} MATCHES "x86")
    set(HOST_PLATFORM "x86")
ELSEIF (${MACHINES} MATCHES "i686")
    set(HOST_PLATFORM "x86")
ELSEIF (${MACHINES} MATCHES "i386")
    set(HOST_PLATFORM "x86")
ELSEIF (${MACHINES} MATCHES "arm")
    set(HOST_PLATFORM "arm")
ELSEIF ((${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "64"))
    set(HOST_PLATFORM "arm64")
ELSEIF ((${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "32"))
    set(HOST_PLATFORM "arm")
ENDIF ()

include_directories(${CMAKE_CURRENT_BINARY_DIR}/openni2)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/openni2_redist/${HOST_PLATFORM})

include_directories(include
        ${Boost_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include/openni2
        ${CMAKE_CURRENT_BINARY_DIR}/openni2_redist/${HOST_PLATFORM}
        )


add_library(astra_wrapper
        src/astra_convert.cpp
        src/astra_device.cpp
        src/astra_device_info.cpp
        src/astra_timer_filter.cpp
        src/astra_frame_listener.cpp
        src/astra_device_manager.cpp
        src/astra_exception.cpp
        src/astra_video_mode.cpp
        src/astra_device_type.cpp
        )
ament_target_dependencies(astra_wrapper
        "builtin_interfaces"
        "rclcpp"
        "sensor_msgs"
        "image_transport"
        "camera_info_manager")


target_link_libraries(astra_wrapper -lOpenNI2 ${Boost_LIBRARIES})

add_library(astra_driver_lib src/astra_driver.cpp src/astra_device_type.cpp)
ament_target_dependencies(astra_driver_lib
        "builtin_interfaces"
        "rclcpp"
        "sensor_msgs"
        "image_transport"
        "camera_info_manager")

add_executable(astra_camera_node
        ros/astra_camera_node.cpp
        )
target_link_libraries(astra_camera_node astra_driver_lib astra_wrapper ${Boost_LIBRARIES})
ament_target_dependencies(astra_camera_node
        "builtin_interfaces"
        "rclcpp"
        "sensor_msgs"
        "image_transport"
        "camera_info_manager")


install(TARGETS astra_camera_node
        DESTINATION lib/${PROJECT_NAME}
        )

install(TARGETS astra_wrapper astra_driver_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

ament_export_include_directories(include)
ament_export_libraries(astra_driver_lib astra_wrapper)
ament_package()