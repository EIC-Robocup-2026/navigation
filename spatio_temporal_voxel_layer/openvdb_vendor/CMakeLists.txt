cmake_minimum_required(VERSION 3.18)
project(openvdb_vendor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

foreach(prefix ${CMAKE_PREFIX_PATH} $ENV{CONDA_PREFIX})
  if(prefix AND EXISTS "${prefix}/lib/cmake/OpenVDB")
    list(APPEND CMAKE_MODULE_PATH "${prefix}/lib/cmake/OpenVDB")
  endif()
endforeach()
list(APPEND CMAKE_MODULE_PATH "/usr/lib/x86_64-linux-gnu/cmake/OpenVDB")
list(APPEND CMAKE_MODULE_PATH "/usr/lib/aarch64-linux-gnu/cmake/OpenVDB")
find_package(OpenVDB QUIET)

ament_vendor(${PROJECT_NAME}
  SATISFIED ${OpenVDB_FOUND}
  VCS_URL https://github.com/AcademySoftwareFoundation/openvdb.git
  VCS_VERSION v10.0.1
  VCS_TYPE git
  CMAKE_ARGS
    -DCONCURRENT_MALLOC=Tbbmalloc
    -DOPENVDB_BUILD_BINARIES=OFF
)

# When this package is found, make sure to find the real OpenVDB
ament_package(
  CONFIG_EXTRAS "openvdb_vendor-extras.cmake"
)
