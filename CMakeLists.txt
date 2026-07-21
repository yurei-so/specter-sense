cmake_minimum_required(VERSION 3.20)
project(specter_sense VERSION 0.1.0 LANGUAGES CXX)

option(SPECTER_SENSE_WITH_KINECT_V1 "Build the libfreenect Kinect V1 source when available" ON)
option(SPECTER_SENSE_WITH_KINECT_V2 "Build the libfreenect2 Kinect V2 source when available" ON)
option(SPECTER_SENSE_BUILD_CALIBRATOR "Build the interactive OpenGL calibration utility" ON)
set(FREENECT_ROOT "" CACHE PATH "Path to a libfreenect install prefix")
set(FREENECT2_ROOT "" CACHE PATH "Path to a libfreenect2 source/build or install prefix")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(specter_sense_core
  src/boost_json.cpp
  src/config.cpp
  src/environment.cpp
  src/pipeline.cpp
  src/socket_publisher.cpp
  src/state_writer.cpp
  src/synthetic_source.cpp
)
target_include_directories(specter_sense_core PUBLIC include)
target_compile_options(specter_sense_core PRIVATE -Wall -Wextra -Wpedantic -Wconversion)

if(SPECTER_SENSE_WITH_KINECT_V1)
  find_path(FREENECT_INCLUDE_DIR libfreenect.h HINTS ${FREENECT_ROOT} PATH_SUFFIXES include usr/include)
  find_library(FREENECT_LIBRARY freenect HINTS ${FREENECT_ROOT}
    PATH_SUFFIXES lib lib64 lib/x86_64-linux-gnu usr/lib usr/lib/x86_64-linux-gnu)
  if(FREENECT_INCLUDE_DIR AND FREENECT_LIBRARY)
    target_sources(specter_sense_core PRIVATE src/kinect_v1_source.cpp)
    target_include_directories(specter_sense_core PRIVATE ${FREENECT_INCLUDE_DIR})
    target_link_libraries(specter_sense_core PRIVATE ${FREENECT_LIBRARY})
    target_compile_definitions(specter_sense_core PUBLIC SPECTER_SENSE_HAS_KINECT_V1=1)
  else()
    message(STATUS "libfreenect not found; Kinect V1 source disabled")
  endif()
endif()

if(SPECTER_SENSE_WITH_KINECT_V2)
  find_path(FREENECT2_INCLUDE_DIR libfreenect2/libfreenect2.hpp
    HINTS ${FREENECT2_ROOT} PATH_SUFFIXES include)
  find_library(FREENECT2_LIBRARY freenect2
    HINTS ${FREENECT2_ROOT} PATH_SUFFIXES lib build/lib)
  if(FREENECT2_INCLUDE_DIR AND FREENECT2_LIBRARY)
    target_sources(specter_sense_core PRIVATE src/kinect_source.cpp)
    target_include_directories(specter_sense_core PRIVATE ${FREENECT2_INCLUDE_DIR})
    if(EXISTS "${FREENECT2_ROOT}/build/libfreenect2/config.h")
      target_include_directories(specter_sense_core PRIVATE "${FREENECT2_ROOT}/build")
    endif()
    target_link_libraries(specter_sense_core PRIVATE ${FREENECT2_LIBRARY})
    target_compile_definitions(specter_sense_core PUBLIC SPECTER_SENSE_HAS_KINECT_V2=1)
  else()
    message(STATUS "libfreenect2 not found; Kinect V2 source disabled")
  endif()
endif()

add_executable(specter-sense src/main.cpp)
target_link_libraries(specter-sense PRIVATE specter_sense_core)
target_compile_options(specter-sense PRIVATE -Wall -Wextra -Wpedantic -Wconversion)

if(SPECTER_SENSE_BUILD_CALIBRATOR)
  find_package(OpenGL QUIET)
  find_package(glfw3 QUIET CONFIG)
  if(OpenGL_FOUND AND TARGET glfw)
    add_executable(specter-sense-calibrate src/calibrate.cpp)
    target_link_libraries(specter-sense-calibrate PRIVATE specter_sense_core OpenGL::GL glfw)
    target_compile_options(specter-sense-calibrate PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
  else()
    message(STATUS "OpenGL or GLFW not found; calibration utility will not be built")
  endif()
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(specter-sense-tests tests/core_tests.cpp)
  target_link_libraries(specter-sense-tests PRIVATE specter_sense_core)
  target_compile_options(specter-sense-tests PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
  add_test(NAME core COMMAND specter-sense-tests)
endif()

install(TARGETS specter-sense RUNTIME DESTINATION bin)
if(TARGET specter-sense-calibrate)
  install(TARGETS specter-sense-calibrate RUNTIME DESTINATION bin)
endif()
install(FILES config/specter-sense.example.json DESTINATION share/specter-sense)
