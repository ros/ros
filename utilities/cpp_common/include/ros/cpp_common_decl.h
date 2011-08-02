#ifndef ROS_CPP_COMMON_DECL_H_INCLUDED
#define ROS_CPP_COMMON_DECL_H_INCLUDED

#include <ros/macros.h>

#ifdef ROS_BUILD_SHARED_LIBS // ros is being built around shared libraries
  #ifdef cpp_common_EXPORTS // we are building a shared lib/dll
    #define CPP_COMMON_DECL ROS_HELPER_EXPORT
  #else // we are using shared lib/dll
    #define CPP_COMMON_DECL ROS_HELPER_IMPORT
  #endif 
#else // ros is being built around static libraries
  #define CPP_COMMON_DECL
#endif

#endif
