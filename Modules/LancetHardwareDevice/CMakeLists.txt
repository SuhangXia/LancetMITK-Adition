set(AIMOOE_INCLUDE "")
set(AIMOOE_LIB_RELEASE "")
set(AIMOOE_DLLS_RELEASE "")

set(AIMOOE_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/AimPosDll/include")
file(GLOB AIMOOE_LIB_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/AimPosDll/Release/*.lib")

###############################################################################################

set(Diana_INCLUDE "")
set(Diana_LIB_RELEASE "")
set(Diana_DLLS_RELEASE "")

set(Diana_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/DianaApi/include")
file(GLOB Diana_LIB_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/DianaApi/Release/*.lib")

######################################################################################
set(Hans_INCLUDE "")
set(Hans_LIB_RELEASE "")
set(Hans_DLLS_RELEASE "")

set(Hans_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/HansRobotDll/include")
file(GLOB Hans_LIB_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/HansRobotDll/Release/*.lib")
#file(GLOB Hans_DLL_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/HansRobotDll/Release/*.dll")
###############################################################################
set(Jaka_INCLUDE "")
set(Jaka_LIB_RELEASE "")
set(Jaka_DLLS_RELEASE "")

set(Jaka_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/JakaRobotDll/include")
file(GLOB Jaka_LIB_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/JakaRobotDll/Release/*.lib")
#file(GLOB Jaka_DLL_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/JakaRobotDll/Release/*.dll")

######################################################################################
set(ARIEMEDI_INCLUDE "")
set(ARIEMEDI_LIB_RELEASE "")

set(ARIEMEDI_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/AriemediDll/include")
file(GLOB ARIEMEDI_LIB_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/AriemediDll/Release/*.lib")

add_definitions(-DARMDCOMBINEDAPI_EXPORTS)
############################################################################


mitk_create_module(
  INCLUDE_DIRS
     #PUBLIC ${ARIEMEDI_INCLUDE}
     PUBLIC ${AIMOOE_INCLUDE}
     PUBLIC ${Diana_INCLUDE}
     PUBLIC ${Hans_INCLUDE}
     PUBLIC ${Jaka_INCLUDE}
  DEPENDS PUBLIC MitkCore 
  DEPENDS MitkLancetPrintDataHelper
  PACKAGE_DEPENDS VTK
  PACKAGE_DEPENDS Qt5|Core+Widgets
  #ADDITIONAL_LIBS PUBLIC "${ARIEMEDI_LIB_RELEASE}"
  ADDITIONAL_LIBS PUBLIC "${AIMOOE_LIB_RELEASE}"
  ADDITIONAL_LIBS PUBLIC "${Diana_LIB_RELEASE}"
  ADDITIONAL_LIBS PUBLIC "${Hans_LIB_RELEASE}"
  ADDITIONAL_LIBS PUBLIC "${Jaka_LIB_RELEASE}"
)

# 添加 Hans 的 DLL
#foreach(DLL_FILE ${Hans_DLL_RELEASE})
  #  add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    #    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${DLL_FILE}" $<TARGET_FILE_DIR:${PROJECT_NAME}>)
#endforeach()

# 添加 Jaka 的 DLL
#foreach(DLL_FILE ${Jaka_DLL_RELEASE})
  #  add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    #    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${DLL_FILE}" $<TARGET_FILE_DIR:${PROJECT_NAME}>)
#endforeach()

add_subdirectory(cmdapps)
