DIPLOMOVÁ PRÁCE: Návrh systému pro autonomně bezkolizní řízení UAV
Příloha C – Fyzická platforma (sim-to-real přenos)
Autor: rtm. Richard Línek
Univerzita obrany, FVT, K-211 Katedra vojenské robotiky, 2026

========================================================================
OBSAH PŘÍLOHY
========================================================================

CMakeLists.txt
    Konfigurační soubor systému CMake pro sestavení projektu na platformě
    Jetson Nano (ARM Cortex-A57); definuje závislosti (PCL, Eigen3,
    yaml-cpp, MAVSDK) a parametry kompilace.

install_deps_jetson.sh
    Bash skript pro automatickou instalaci všech závislostí projektu
    na Jetson Nano (JetPack 4.6 / Ubuntu 18.04 ARM64).

config/config.yaml
    Centrální konfigurační soubor systému; obsahuje parametry LiDAR
    masky, ICP odometrie, MAVLink připojení, watchdogu, Theta* plánovače
    a transformací souřadnic.

src/main.cpp
    Vstupní bod aplikace; inicializuje všechny moduly, spouští vlákna
    (LiDAR příjem, SLAM, watchdog) a řídí průběh mise od vzletu
    po přistání.

src/mavlink_bridge.cpp
    Implementace komunikace s řídicí jednotkou letu Pixhawk přes MAVSDK;
    zajišťuje armování, vzlet, Offboard waypoint navigaci, přistání
    a failsafe akce.

src/slam_odometry.cpp
    Implementace frame-to-keyframe ICP odometrie pomocí knihovny PCL;
    odhaduje polohu UAV v lokálním NED frame z dat Velodyne VLP-16.

src/lidar_masking.cpp
    Implementace třívrstvé filtrace mračna bodů: CropBox maska konstrukce
    UAV, PassThrough výškový filtr a VoxelGrid downsample pro snížení
    výpočetní zátěže před ICP.

src/watchdog.cpp
    Implementace bezpečnostního watchdogu; monitoruje dostupnost LiDAR dat
    a SLAM odometrie a při výpadku spouští failsafe akci (LOITER nebo LAND).

src/tf_utils.cpp
    Implementace transformací souřadnicových rámců LiDAR → BODY → NED
    pomocí ZYX Euler úhlů a IMU dat z Pixhawku.

include/mavlink_bridge.hpp
    Rozhraní třídy MavlinkBridge; deklaruje stavy mise, konfiguraci,
    strukturu Waypoint a metody pro řízení letu přes MAVSDK.

include/slam_odometry.hpp
    Rozhraní třídy SlamOdometry a struktury Pose; deklaruje metody
    pro aktualizaci ICP odometrie a vláknově bezpečné čtení polohy.

include/lidar_masking.hpp
    Rozhraní třídy LidarMasking; deklaruje konfiguraci filtrů a metody
    pro zpracování mračna bodů z VLP-16.

include/watchdog.hpp
    Rozhraní třídy Watchdog; deklaruje stavy, konfiguraci a metody
    pro monitorování senzorů a spouštění failsafe akcí.

include/tf_utils.hpp
    Rozhraní třídy TfUtils; deklaruje pomocné metody pro transformace
    souřadnic a konverzi mezi NED a ENU rámcem.

========================================================================
SESTAVENÍ
========================================================================

    mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make -j4
    ./stc_drone_nav ../config/config.yaml
