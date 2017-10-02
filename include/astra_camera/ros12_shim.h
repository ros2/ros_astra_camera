#ifndef ROS12_SHIM_H_
#define ROS12_SHIM_H_

#include <iostream>

#include <rcutils/logging_macros.h>

#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_DEBUG RCUTILS_LOG_DEBUG
#define ROS_WARN RCUTILS_LOG_WARN
#define ROS_ERROR RCUTILS_LOG_ERROR
#define ROS_FATAL RCUTILS_LOG_FATAL

#ifndef ROS_INFO_STREAM
  #define ROS_INFO_STREAM(str) std::cout << str << std::endl
#endif
#ifndef ROS_DEBUG_STREAM
  #define ROS_DEBUG_STREAM(str)
#endif
#ifndef ROS_WARN_STREAM
  #define ROS_WARN_STREAM(str) std::cout << str << std::endl
#endif
#ifndef ROS_ERROR_STREAM
  #define ROS_ERROR_STREAM(str) std::cout << str << std::endl
#endif
#ifndef ROS_FATAL_STREAM
  #define ROS_FATAL_STREAM(str) std::cout << str << std::endl
#endif

#endif
