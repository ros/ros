# Microsoft Developer Studio Project File - Name="xmlrpc" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Static Library" 0x0104

CFG=xmlrpc - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "xmlrpc.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "xmlrpc.mak" CFG="xmlrpc - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "xmlrpc - Win32 Release" (based on "Win32 (x86) Static Library")
!MESSAGE "xmlrpc - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
RSC=rc.exe

!IF  "$(CFG)" == "xmlrpc - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Target_Dir ""
MTL=midl.exe
# ADD BASE MTL /nologo /D "NDEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "NDEBUG" /mktyplib203 /win32
# ADD BASE CPP /nologo /MT /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /D "_MBCS" /D "_LIB" /YX /FD /c
# ADD CPP /nologo /MD /W3 /GX /Zd /O2 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /D "_MBCS" /D "_LIB" /FR /YX /FD /c
# ADD BASE RSC /l 0x409 /d "NDEBUG"
# ADD RSC /l 0x409 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo

!ELSEIF  "$(CFG)" == "xmlrpc - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Target_Dir ""
MTL=midl.exe
# ADD BASE MTL /nologo /D "_DEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "_DEBUG" /mktyplib203 /win32
# ADD BASE CPP /nologo /MTd /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /D "_MBCS" /D "_LIB" /YX /FD /GZ /c
# ADD CPP /MDd /W3 /GX /Zi /Od /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /D "_MBCS" /D "_LIB" /FR /FD /c
# SUBTRACT CPP /nologo /YX
# ADD BASE RSC /l 0x409 /d "_DEBUG"
# ADD RSC /l 0x409 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo

!ENDIF 

# Begin Target

# Name "xmlrpc - Win32 Release"
# Name "xmlrpc - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat"
# Begin Source File

SOURCE=.\src\XmlRpcClient.cpp
# End Source File
# Begin Source File

SOURCE=.\src\XmlRpcDispatch.cpp
# End Source File
# Begin Source File

SOURCE=.\src\XmlRpcServer.cpp
# End Source File
# Begin Source File

SOURCE=.\src\XmlRpcServerConnection.cpp
# End Source File
# Begin Source File

SOURCE=.\src\XmlRpcServerMethod.cpp
# End Source File
# Begin Source File

SOURCE=.\src\XmlRpcSocket.cpp
# End Source File
# Begin Source File

SOURCE=.\src\XmlRpcSource.cpp
# End Source File
# Begin Source File

SOURCE=.\src\XmlRpcUtil.cpp
# End Source File
# Begin Source File

SOURCE=.\src\XmlRpcValue.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl"
# Begin Source File

SOURCE=.\src\base64.h
# End Source File
# Begin Source File

SOURCE=.\src\XmlRpc.h
# End Source File
# Begin Source File

SOURCE=.\src\XmlRpcClient.h
# End Source File
# Begin Source File

SOURCE=.\src\XmlRpcDispatch.h
# End Source File
# Begin Source File

SOURCE=.\src\XmlRpcServer.h
# End Source File
# Begin Source File

SOURCE=.\src\XmlRpcServerConnection.h
# End Source File
# Begin Source File

SOURCE=.\src\XmlRpcServerMethod.h
# End Source File
# Begin Source File

SOURCE=.\src\XmlRpcSocket.h
# End Source File
# Begin Source File

SOURCE=.\src\XmlRpcSource.h
# End Source File
# Begin Source File

SOURCE=.\src\XmlRpcUtil.h
# End Source File
# Begin Source File

SOURCE=.\src\XmlRpcValue.h
# End Source File
# End Group
# Begin Source File

SOURCE=.\README.html
# End Source File
# End Target
# End Project
