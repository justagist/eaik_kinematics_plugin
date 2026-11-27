cmake_minimum_required(VERSION 3.16)
project(eaik_vendor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

find_package(EAIK 1.2.2 QUIET)

# The fork is EAIK v1.2.2 with an installable library, a fix for joint 6 at the wrist singularity
# of spherical-wrist arms, and a fix for parallel axes that point in opposite directions.
ament_vendor(eaik_upstream
  SATISFIED ${EAIK_FOUND}
  VCS_URL https://github.com/justagist/EAIK.git
  VCS_VERSION d3887e1145b86c4aa59b752e07788cc90ffd771a
  SOURCE_SUBDIR CPP/src
)

if(NOT EAIK_FOUND OR FORCE_BUILD_VENDOR_PKG)
  set(_eaik_src ${CMAKE_CURRENT_BINARY_DIR}/eaik_upstream-prefix/src/eaik_upstream)
  install(FILES ${_eaik_src}/LICENSE DESTINATION share/${PROJECT_NAME}/licenses RENAME EAIK-LICENSE)
  install(FILES ${_eaik_src}/CPP/external/ik-geo/LICENSE DESTINATION share/${PROJECT_NAME}/licenses
          RENAME ik-geo-LICENSE)
endif()

ament_package()
