cmake_minimum_required(VERSION 3.8)
project(gz-unitree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(gz-cmake3 REQUIRED)
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
find_package(gz-gui9 REQUIRED)
set(GZ_GUI_VER ${gz-gui9_VERSION_MAJOR})
find_package(gz-rendering9 REQUIRED)
set(GZ_RENDERING_VER ${gz-rendering9_VERSION_MAJOR})
find_package(gz-transport14 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport14_VERSION_MAJOR})

add_library(gz-video-recorder SHARED RemoteVideoRecorder.cc)
target_link_libraries(
  gz-video-recorder
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-common${GZ_COMMON_VER}::profiler
  gz-gui${GZ_GUI_VER}::gz-gui${GZ_GUI_VER}
  gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
  gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
)
