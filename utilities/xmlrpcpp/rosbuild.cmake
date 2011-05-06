include(${CMAKE_CURRENT_BINARY_DIR}/package.cmake)

if(WIN32)
  add_definitions(-D_WINDOWS)
endif()

rosbuild_add_library(XmlRpc 
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

