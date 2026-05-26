# ============================================================
# DIPLOMOVÁ PRÁCE: Návrh systému pro autonomně bezkolizní řízení UAV
# Příloha C
# Autor: rtm. Richard Línek
# Fyzická platforma (sim-to-real přenos)
#
# Konfigurace sestavení projektu STC Drone Nav
# Hardware: Jetson Nano (JetPack 4.6 / Ubuntu 18.04)
#           Velodyne VLP-16, Pixhawk (PX4)
# ============================================================

cmake_minimum_required(VERSION 3.12)
project(stc_drone_nav VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Optimalizace pro ARM Cortex-A57 (Jetson Nano)
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -march=armv8-a -mtune=cortex-a57 -ffast-math -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g -Wall -Wextra")

message(STATUS "Typ sestavení: ${CMAKE_BUILD_TYPE}")
message(STATUS "Cílová architektura: ARM Cortex-A57 (Jetson Nano)")

# ============================================================
# Závislosti
# ============================================================

# PCL – Point Cloud Library (verze 1.8 dostupná v Ubuntu 18.04)
find_package(PCL 1.8 REQUIRED COMPONENTS
    common
    io
    filters
    registration
    kdtree
    features
)
if(PCL_FOUND)
    message(STATUS "PCL nalezen: ${PCL_VERSION}")
else()
    message(FATAL_ERROR "PCL nenalezen! Nainstalujte: sudo apt-get install libpcl-dev")
endif()

# Eigen3 – lineární algebra
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
if(EIGEN3_FOUND)
    message(STATUS "Eigen3 nalezen: ${EIGEN3_VERSION}")
endif()

# yaml-cpp – načítání konfigurace
find_package(yaml-cpp REQUIRED)
if(yaml-cpp_FOUND)
    message(STATUS "yaml-cpp nalezen")
else()
    message(FATAL_ERROR "yaml-cpp nenalezen! Nainstalujte: sudo apt-get install libyaml-cpp-dev")
endif()

# MAVSDK – komunikace s Pixhawkem přes MAVLink
find_package(MAVSDK REQUIRED)
if(MAVSDK_FOUND)
    message(STATUS "MAVSDK nalezen")
else()
    message(FATAL_ERROR "MAVSDK nenalezen! Sestavte ze zdroje nebo nainstalujte deb balíček.")
endif()

# Vlákna (pthreads)
find_package(Threads REQUIRED)

# ============================================================
# Hlavičkové soubory
# ============================================================
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${YAML_CPP_INCLUDE_DIR}
)

add_definitions(${PCL_DEFINITIONS})

# ============================================================
# Spustitelný soubor
# ============================================================
set(SRC_FILES
    src/main.cpp
    src/lidar_masking.cpp
    src/slam_odometry.cpp
    src/mavlink_bridge.cpp
    src/tf_utils.cpp
    src/watchdog.cpp
)

add_executable(stc_drone_nav ${SRC_FILES})

target_link_libraries(stc_drone_nav
    PRIVATE
        ${PCL_LIBRARIES}
        Eigen3::Eigen
        yaml-cpp
        MAVSDK::mavsdk
        Threads::Threads
)

# ============================================================
# Kopírování konfigurace do build adresáře
# ============================================================
configure_file(
    ${CMAKE_SOURCE_DIR}/config/config.yaml
    ${CMAKE_BINARY_DIR}/config/config.yaml
    COPYONLY
)

# ============================================================
# Instalace
# ============================================================
install(TARGETS stc_drone_nav DESTINATION bin)
install(DIRECTORY config/ DESTINATION share/stc_drone_nav/config)

# ============================================================
# Informace o sestavení
# ============================================================
message(STATUS "")
message(STATUS "=== STC Drone Nav – konfigurace sestavení ===")
message(STATUS "  Projekt:       ${PROJECT_NAME} v${PROJECT_VERSION}")
message(STATUS "  Sestavení:     ${CMAKE_BUILD_TYPE}")
message(STATUS "  Kompilátor:    ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  Instalace do:  ${CMAKE_INSTALL_PREFIX}")
message(STATUS "==============================================")
message(STATUS "")
message(STATUS "Příkaz pro sestavení na Jetson Nano:")
message(STATUS "  mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j4")
message(STATUS "")
