cmake_minimum_required(VERSION 3.16)
project(eaik_vendor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

find_package(EAIK 1.2.2 QUIET)

# The patch makes CPP/src an installable library with a CMake package config. It
# is the change proposed upstream; remove it when upstream contains it.
ament_vendor(eaik_upstream
  SATISFIED ${EAIK_FOUND}
  VCS_URL https://github.com/OstermD/EAIK.git
  VCS_VERSION 3d973f6b7e7c87ca32928ec5b7eb5e50d956a7f5
  SOURCE_SUBDIR CPP/src
  PATCHES patches/0001-cmake-installable-library.patch
)

if(NOT EAIK_FOUND OR FORCE_BUILD_VENDOR_PKG)
  set(_eaik_src ${CMAKE_CURRENT_BINARY_DIR}/eaik_upstream-prefix/src/eaik_upstream)
  install(FILES ${_eaik_src}/LICENSE DESTINATION share/${PROJECT_NAME}/licenses RENAME EAIK-LICENSE)
  install(FILES ${_eaik_src}/CPP/external/ik-geo/LICENSE DESTINATION share/${PROJECT_NAME}/licenses
          RENAME ik-geo-LICENSE)
endif()

ament_package()
