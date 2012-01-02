cmake_minimum_required(VERSION 2.8)
project(XmlRpc)

find_package(catkin)

find_package(cpp_common)
include_directories(${cpp_common_INCLUDE_DIRS})

include_directories(src)

if(WIN32)
  add_definitions(-D_WINDOWS)
endif()

add_library(XmlRpc SHARED
  src/XmlRpcClient.cpp 
  src/XmlRpcDispatch.cpp 
  src/XmlRpcServer.cpp 
  src/XmlRpcServerConnection.cpp 
  src/XmlRpcServerMethod.cpp 
  src/XmlRpcSocket.cpp 
  src/XmlRpcSource.cpp 
  src/XmlRpcUtil.cpp 
  src/XmlRpcValue.cpp
  )

if(WIN32)
  target_link_libraries(XmlRpc ws2_32)
endif()

foreach(header
    src/base64.h	      
    src/XmlRpcException.h	    
    src/XmlRpcServer.h	      
    src/XmlRpcSource.h
    src/XmlRpcClient.h    
    src/XmlRpc.h		    
    src/XmlRpcServerMethod.h  
    src/XmlRpcUtil.h
    src/XmlRpcDispatch.h  
    src/XmlRpcServerConnection.h  
    src/XmlRpcSocket.h	      
    src/XmlRpcValue.h)
  get_filename_component(base ${header} NAME)

  install(FILES ${header}
    DESTINATION include
    RENAME ${base}
    )
endforeach()

#explicitly install library and includes
install(TARGETS XmlRpc
  DESTINATION lib
 )

catkin_project(XmlRpc
  VERSION 0.0.1
  INCLUDE_DIRS src
  LIBRARIES XmlRpc
  )
