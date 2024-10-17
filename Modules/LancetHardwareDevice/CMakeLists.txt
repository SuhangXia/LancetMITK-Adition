set(AIMOOE_INCLUDE "")
set(AIMOOE_LIB_RELEASE "")
set(AIMOOE_DLLS_RELEASE "")

set(AIMOOE_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/AimPosDll/include")
file(GLOB AIMOOE_LIB_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/AimPosDll/Release/*.lib")
#file(GLOB AIMOOE_DLL_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/AimPosDll/Release/*.dll")
###############################################################################################

set(Diana_INCLUDE "")
set(Diana_LIB_RELEASE "")
set(Diana_DLLS_RELEASE "")

set(Diana_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/DianaApi/include")
file(GLOB Diana_LIB_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/DianaApi/Release/*.lib")
file(GLOB Diana_DLL_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/DianaApi/Release/*.dll")
######################################################################################
set(Hans_INCLUDE "")
set(Hans_LIB_RELEASE "")
set(Hans_DLLS_RELEASE "")

set(Hans_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/HansRobotDll/include")
file(GLOB Hans_LIB_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/HansRobotDll/Release/*.lib")
file(GLOB Hans_DLL_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/HansRobotDll/Release/*.dll")
###############################################################################
set(Jaka_INCLUDE "")
set(Jaka_LIB_RELEASE "")
set(Jaka_DLLS_RELEASE "")

set(Jaka_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/JakaRobotDll/include")
file(GLOB Jaka_LIB_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/JakaRobotDll/Release/*.lib")
file(GLOB Jaka_DLL_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/JakaRobotDll/Release/*.dll")

######################################################################################
# set(ARIEMEDI_INCLUDE "")
# set(ARIEMEDI_LIB_RELEASE "")

# set(ARIEMEDI_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/AriemediDll/include")
# file(GLOB ARIEMEDI_LIB_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/AriemediDll/Release/*.lib")
# file(GLOB ARIEMEDI_LIB_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/AriemediDll/Release/*.dll")

add_definitions(-DARMDCOMBINEDAPI_EXPORTS)

mitk_create_module(
  INCLUDE_DIRS
     #PUBLIC ${ARIEMEDI_INCLUDE}
     PUBLIC ${AIMOOE_INCLUDE}
     PUBLIC ${Diana_INCLUDE}
     PUBLIC ${Hans_INCLUDE}
     PUBLIC ${ARIEMEDI_INCLUDE}
     PUBLIC ${Jaka_INCLUDE}
  DEPENDS PUBLIC MitkCore 
  DEPENDS MitkLancetPrintDataHelper
  PACKAGE_DEPENDS VTK
  PACKAGE_DEPENDS Qt5|Core+Widgets
  #ADDITIONAL_LIBS PUBLIC "${ARIEMEDI_LIB_RELEASE}"
  ADDITIONAL_LIBS PUBLIC "${AIMOOE_LIB_RELEASE}"
  ADDITIONAL_LIBS PUBLIC "${Diana_LIB_RELEASE}"
  ADDITIONAL_LIBS PUBLIC "${Hans_LIB_RELEASE}"
  ADDITIONAL_LIBS PUBLIC "${ARIEMEDI_LIB_RELEASE}"
  ADDITIONAL_LIBS PUBLIC "${Jaka_LIB_RELEASE}"

   # Do not automatically create CppMicroServices initialization code.
  # Because the VTK "auto-init" functionality injects file-local static
  # initialization code in every cpp file which includes a VTK header,
  # static initialization order becomes an issue again. For the Mitk
  # core library, we need to ensure that the VTK static initialization stuff
  # happens before the CppMicroServices initialization, since the latter
  # might already use VTK code which needs to access VTK object factories.
  # Hence, CppMicroServices initialization code is placed manually within
  # the mitkCoreActivator.cpp file.
  NO_INIT
)
add_subdirectory(cmdapps)
