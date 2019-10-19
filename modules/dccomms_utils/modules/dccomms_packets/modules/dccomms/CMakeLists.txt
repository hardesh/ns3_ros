cmake_minimum_required (VERSION 3.5.1)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_COLOR_MAKEFILE   ON)

#SET PROJECT NAME
SET(PROJECT_NAME_ORIG "dccomms" )

#SET DEPENDENCIES DIRECTORY
SET(MODULES_DIR "deps" )
 
SET(PROJECT_NAME_ORIG dccomms)
message("Building ${PROJECT_NAME_ORIG}...")

#BEGIN RESOLVE GIT REVISION
#https://bravenewmethod.com/2017/07/14/git-revision-as-compiler-definition-in-build-with-cmake/
message(STATUS "Resolving GIT Version...")
set(_build_version "unknown")
find_package(Git)
if(GIT_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    OUTPUT_VARIABLE _build_version
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  message( STATUS "GIT hash: ${_build_version}")
else()
  message(STATUS "GIT not found")
endif()
string(TIMESTAMP _time_stamp)
#END RESOLVE GIT REVISION

#http://stackoverflow.com/questions/7787823/cmake-how-to-get-the-name-of-all-subdirectories-of-a-directory
function(GETPROJECTS result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*/CMakeLists.txt)
  SET(dirlist "")
  FOREACH(file ${children})
        get_filename_component(subdir ${file} DIRECTORY)
        LIST(APPEND dirlist ${subdir})
  ENDFOREACH()
  SET(${result} ${dirlist} PARENT_SCOPE)
endfunction()

#DECLARE PROJECT
SET(PROJECT_NAME_GIT "${PROJECT_NAME_ORIG}_${_build_version}")
string(TOUPPER ${PROJECT_NAME_ORIG} TMP0)
SET(${TMP0}_GIT "${PROJECT_NAME_GIT}" PARENT_SCOPE)

if (TARGET ${PROJECT_NAME_GIT})
    message(WARNING "${PROJECT_NAME_GIT} has already been built!")
    return()
endif()

project (${PROJECT_NAME_GIT} CXX)
message(STATUS "PROJECT NAME: ${PROJECT_NAME}")

##################################
####### BUILD DEPENDENCIES #######
##################################

SET(DEPS_DIR ${MODULES_DIR})
SET(TMP0 ${CMAKE_CURRENT_SOURCE_DIR}/${DEPS_DIR})
GETPROJECTS(DEPS ${TMP0})
message(STATUS "Dependencies found in ${TMP0}: ${DEPS}")
FOREACH(subdir ${DEPS})
if(GIT_FOUND)
  SET(SUBPROJECT_PATH ${DEPS_DIR}/${subdir})
  SET(SUBPROJECT_FULL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/${SUBPROJECT_PATH})
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
    WORKING_DIRECTORY ${SUBPROJECT_FULL_PATH}
    OUTPUT_VARIABLE _sub_build_version
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  SET(SUBPROJECT_GIT_NAME ${subdir}_${_sub_build_version})
  message(STATUS "Subproject generated git name: " ${SUBPROJECT_GIT_NAME})
  if (TARGET ${subdir})
      message(WARNING "Subproject ${SUBPROJECT_GIT_NAME} (${subdir}) has already been built!")
  else()
      add_subdirectory(${SUBPROJECT_PATH})
  endif()
else()
  message(STATUS "GIT not found")
endif()
ENDFOREACH()

###### 

SET(${PROJECT_NAME}_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${${CPPLOGGING_GIT}_INCLUDE_DIRS}
    ${${CLASS_LOADER_GIT}_INCLUDE_DIRS}
    ${${CPPUTILS_GIT}_INCLUDE_DIRS}
   )

SET(SRCPATH src/)
file(GLOB ${PROJECT_NAME}_CPP_FILES
    ${SRCPATH}Arduino.cpp
    ${SRCPATH}BlockRadioTransmitter.cpp
    ${SRCPATH}Checksum.cpp
    ${SRCPATH}CommsBridge.cpp
    ${SRCPATH}CommsDeviceService.cpp
    ${SRCPATH}CommsException.cpp
    ${SRCPATH}DataLinkFrame.cpp
    ${SRCPATH}CommsDevice.cpp
    ${SRCPATH}Stream.cpp
    ${SRCPATH}StreamCommsDevice.cpp
    ${SRCPATH}NamedPipeStream.cpp
    ${SRCPATH}Packet.cpp
    ${SRCPATH}Radio.cpp
    ${SRCPATH}SerialPortStream.cpp
    ${SRCPATH}TCPStream.cpp
    ${SRCPATH}TransportPDU.cpp
    ${SRCPATH}Utils.cpp
    ${SRCPATH}CommsDeviceSocket.cpp
    )

SET(PROJECT_LIBRARIES m pthread rt crypto ${CPPLOGGING_GIT} ${${CLASS_LOADER_GIT}_LIBRARIES} ${CPPUTILS_GIT})

SET(${PROJECT_NAME}_CXX_COMPILE_FLAGS "-fPIC -std=c++14")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${${PROJECT_NAME}_CXX_COMPILE_FLAGS}")

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_CPP_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${${PROJECT_NAME}_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${PROJECT_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
	LIBRARY DESTINATION "lib"
	)
install(DIRECTORY "include/dccomms" DESTINATION "include")

SET(${PROJECT_NAME}_INCLUDE_DIRS ${${PROJECT_NAME}_INCLUDE_DIRS} CACHE STRING "${PROJECT_NAME} include dirs")
SET(${PROJECT_NAME}_LIBRARIES ${PROJECT_NAME} ${PROJECT_LIBRARIES} CACHE STRING "${PROJECT_NAME} libraries")

message("End ${PROJECT_NAME_ORIG}")
