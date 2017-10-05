#ifndef ROS12_SHIM_H_
#define ROS12_SHIM_H_

#include <iostream>

#include <rcutils/logging_macros.h>

#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_DEBUG RCUTILS_LOG_DEBUG
#define ROS_WARN RCUTILS_LOG_WARN
#define ROS_ERROR RCUTILS_LOG_ERROR
#define ROS_FATAL RCUTILS_LOG_FATAL

#define ROS_INFO_STREAM(str) std::cout << str << std::endl
#define ROS_DEBUG_STREAM(str)
#define ROS_WARN_STREAM(str) std::cout << str << std::endl
#define ROS_ERROR_STREAM(str) std::cout << str << std::endl
#define ROS_FATAL_STREAM(str) std::cout << str << std::endl

#endif
