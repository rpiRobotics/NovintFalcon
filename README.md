# NovintFalcon
A Robot Raconteur Service for the Novint Falcon Joystick

Written by Gregory Grebe for use in the CATS lab at Rensselaer Polytechnic Institute 

# System Requirments
32 bit system

Windows (Tested with Windows 7)

Microsoft Visual Studio (Tested with 2012 and 2013)

# Required Dependencies
Robot Raconteur C++ SDK 

Boost Version 1.55.0 (You will need to build several of the boost libraries. Follow the RR C++ Guide instructions on building Boost.)

Novint Falcon Drivers

Novint Falcon SDK (Note: Because of limitations in the Novint Falcon SDK the project will only build in 'Release' Configuration)


# Configuring VS Properties
C++ > General > Additional Include Directories:
\<RobotRaconteurDir\>\include; <BoostDir>; \<Path to HDAL_SDK_2.1.3\>\include;


Linker > General > Additional Library Directories:
\<BoostDir\>\stage32-msvc-12.0\lib; \<BoostDir\>\libs; \<RobotRaconteurDir\>\lib\Release; \<Path to HDAL_SDK_2.1.3\>\lib;


Linker > Input > Additional Dependencies:
  
  Rpcrt4.lib; ws2_32.lib; IPHLPAPI.lib; RobotRaconteur2.lib; hdl.lib;

 
# Project Files Descriptions
Main.cpp
  
  The main program. Creates the Robot Raconteur Service.

falcon_impl.h and falcon_impl.cpp
  
  The implementation for the Robot Raconteur Service. Implements the stubs defined in falcon_service and falcon_service_stubskel. 

Haptics.h and Haptics.cpp
  
  The layer to directly communicate with the Falcon. Defines the HapticsClass class.
