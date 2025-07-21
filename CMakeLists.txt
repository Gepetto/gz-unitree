cmake_minimum_required(VERSION 3.8)
project(gz-unitree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
find_package(gz-sensors8 REQUIRED)
set(GZ_SENSORS_VER ${gz-sensors8_VERSION_MAJOR})
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)


add_library(gz-unitree
  SHARED
  system.cc
)

target_include_directories(
  gz-unitree PRIVATE include
)

target_link_libraries(gz-unitree
  unitree_sdk2
  Threads::Threads
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
  gz-common${GZ_COMMON_VER}::profiler
)

target_compile_definitions(gz-unitree
  PUBLIC "GZ_PROFILER_ENABLE=1"
)


# somehow cmake install generates a binary lib that doesnt link to unitre_sdk2's libddsc, doing a dumb cp instead works.
# install(
#   TARGETS gz-unitree
#   DESTINATION lib/${PROJECT_NAME}
# )

# ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
# ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")


# ament_package()
