/*
 * DIPLOMOVÁ PRÁCE: Návrh systému pro autonomně bezkolizní řízení UAV
 * Příloha C
 * Autor: rtm. Richard Línek
 * Fyzická platforma (sim-to-real přenos)
 *
 * Soubor obsahuje vstupní bod aplikace STC Drone Nav:
 *   - Inicializace modulů systému (LiDAR, SLAM, MAVLink, TF, Watchdog)
 *   - Vlákno příjmu a zpracování dat z LiDARu (PCL HDL Grabber)
 *   - Řídicí smyčka mise (Theta* plánování + Offboard MAVLink)
 *   - Obsluha signálů SIGINT/SIGTERM pro čisté ukončení
 *
 * Architektura vláken:
 *   - Vlákno 1 (main):     inicializace, mise, Theta* plánování
 *   - Vlákno 2 (lidar):    příjem a filtrování mračna bodů
 *   - Vlákno 3 (slam):     ICP odometrie na filtrovaných datech
 *   - Vlákno 4 (watchdog): monitorování senzorických dat
 */

#include <boost/function.hpp>
#include <atomic>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <pcl/io/hdl_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lidar_masking.hpp"
#include "mavlink_bridge.hpp"
#include "slam_odometry.hpp"
#include "tf_utils.hpp"
#include "watchdog.hpp"

#include <yaml-cpp/yaml.h>

// ============================================================
// Globální příznak běhu (reakce na signál ukončení)
// ============================================================
namespace {
    std::atomic<bool> g_running{true};

    void signalHandler(int signum)
    {
        std::cout << "\n[Main] Přijat signál " << signum
                  << " – ukončuji bezpečně..." << std::endl;
        g_running.store(false);
    }
}

// ============================================================
// SharedData – sdílená data mezi vlákny (chráněna mutexy)
// ============================================================
struct SharedData {
    stc::CloudTPtr        latest_cloud;
    std::mutex            cloud_mutex;
    std::atomic<bool>     new_cloud_available{false};
};

// ============================================================
// lidarThread – příjem mračen bodů z VLP-16 (PCL HDL Grabber)
// ============================================================
void lidarThread(SharedData&         shared,
                 stc::LidarMasking&  masking,
                 stc::TfUtils&       tf,
                 stc::SlamOdometry&  slam,
                 stc::Watchdog&      watchdog,
                 const YAML::Node&   cfg)
{
    std::cout << "[LiDAR] Vlákno spuštěno." << std::endl;

    std::string device_ip = cfg["velodyne"]["device_ip"].as<std::string>("192.168.1.200");
    int port = cfg["velodyne"]["port"].as<int>(2368);

    try {
        pcl::HDLGrabber grabber(boost::asio::ip::address::from_string(device_ip), port);

        boost::function<void(const stc::CloudTConstPtr&)> cloud_cb =
            [&](const stc::CloudTConstPtr& raw_cloud) {
                if (!g_running.load()) return;

                watchdog.feedLidar();

                auto filtered = masking.process(raw_cloud);

                if (filtered && !filtered->empty()) {
                    // ICP odometrie pracuje v LiDAR frame
                    bool slam_ok = slam.update(filtered);
                    if (slam_ok) {
                        watchdog.feedSlam();
                    }

                    {
                        std::lock_guard<std::mutex> lock(shared.cloud_mutex);
                        shared.latest_cloud = filtered;
                        shared.new_cloud_available.store(true);
                    }
                }
            };

        auto connection = grabber.registerCallback(cloud_cb);
        grabber.start();

        std::cout << "[LiDAR] HDL Grabber spuštěn. Čekám na data z "
                  << device_ip << ":" << port << std::endl;

        while (g_running.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        grabber.stop();
        std::cout << "[LiDAR] HDL Grabber zastaven." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[LiDAR] CHYBA: " << e.what() << std::endl;
        g_running.store(false);
    }
}

// ============================================================
// missionController – hlavní navigační smyčka
// Theta* plánování + řízení přes Offboard MAVLink
// ============================================================
void missionController(SharedData&         shared,
                       stc::MavlinkBridge& bridge,
                       stc::SlamOdometry&  slam,
                       stc::TfUtils&       tf,
                       const YAML::Node&   cfg)
{
    std::cout << "[Mission] Kontrolér mise spuštěn." << std::endl;

    const float replan_interval_ms =
        cfg["planner"]["replan_interval_ms"].as<float>(500.0f);
    const float wp_tolerance_m =
        cfg["planner"]["wp_tolerance_m"].as<float>(0.5f);

    // Cílová poloha mise v lokálním NED frame
    stc::MavlinkBridge::Waypoint mission_goal;
    mission_goal.x   = cfg["mission"]["goal_x"].as<float>(10.0f);
    mission_goal.y   = cfg["mission"]["goal_y"].as<float>(0.0f);
    mission_goal.z   = -cfg["mavlink"]["takeoff_altitude_m"].as<float>(3.0f);
    mission_goal.yaw = std::numeric_limits<float>::quiet_NaN();

    if (!bridge.startOffboard()) {
        std::cerr << "[Mission] Nelze spustit Offboard mód!" << std::endl;
        return;
    }

    // -------------------------------------------------------
    // Hlavní navigační smyčka
    // V každé iteraci:
    //   1. Získá aktuální polohu z ICP odometrie (SLAM)
    //   2. Zkontroluje dostupnost nových LiDAR dat
    //   3. Spustí Theta* plánovač → seznam waypointů
    //   4. Odešle následující waypoint přes Offboard MAVLink
    // -------------------------------------------------------
    std::vector<stc::MavlinkBridge::Waypoint> path;
    size_t current_wp_idx = 0;
    auto last_replan = std::chrono::steady_clock::now();

    std::cout << "[Mission] Navigace ke cíli ["
              << mission_goal.x << ", " << mission_goal.y << ", "
              << mission_goal.z << "] m NED." << std::endl;

    while (g_running.load()) {

        if (!slam.isInitialized()) {
            std::cout << "[Mission] Čekám na inicializaci SLAM..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        stc::Pose current_pose = slam.getCurrentPose();

        // Kontrola dosažení cíle
        float dx = mission_goal.x - current_pose.x;
        float dy = mission_goal.y - current_pose.y;
        float dist_to_goal = std::sqrt(dx*dx + dy*dy);

        if (dist_to_goal < wp_tolerance_m) {
            std::cout << "[Mission] CÍL DOSAŽEN! dist=" << dist_to_goal << " m" << std::endl;
            break;
        }

        // -------------------------------------------------------
        // Přeplánování – integrace Theta* plánovače
        //
        // Vstup:  current_pose (x,y,z,yaw), occupancy grid z LiDAR
        // Výstup: path (seznam Waypoint v NED frame)
        //
        // Příklad volání po integraci TheStar3D:
        //   auto grid = buildOccupancyGrid(shared, cfg);
        //   path = theta_planner.plan(current_pose, mission_goal, grid);
        //   current_wp_idx = 0;
        // -------------------------------------------------------
        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_replan).count();

        if (elapsed_ms > replan_interval_ms || path.empty()) {
            // Přímá cesta k cíli – zástupný kód pro Theta* plánovač
            path.clear();
            path.push_back(mission_goal);
            current_wp_idx = 0;
            last_replan = now;

            std::cout << "[Mission] [Re]plánování: "
                      << path.size() << " WP, dist=" << dist_to_goal
                      << " m, yaw=" << current_pose.yaw << " rad" << std::endl;
        }

        // Kontrola dosažení aktuálního waypointu
        if (!path.empty() && current_wp_idx < path.size()) {
            const auto& wp = path[current_wp_idx];
            float wp_dx = wp.x - current_pose.x;
            float wp_dy = wp.y - current_pose.y;
            float wp_dist = std::sqrt(wp_dx*wp_dx + wp_dy*wp_dy);

            if (wp_dist < wp_tolerance_m && current_wp_idx + 1 < path.size()) {
                ++current_wp_idx;
                std::cout << "[Mission] WP " << current_wp_idx
                          << " dosažen, přecházím na WP " << (current_wp_idx+1)
                          << "/" << path.size() << std::endl;
            }

            bridge.sendWaypoint(path[current_wp_idx]);
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>(replan_interval_ms / 5.0f)));
    }

    std::cout << "[Mission] Navigační smyčka ukončena." << std::endl;
}

// ============================================================
// main – vstupní bod aplikace
// ============================================================
int main(int argc, char* argv[])
{
    std::signal(SIGINT,  signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::cout << "============================================" << std::endl;
    std::cout << " STC Drone Nav v1.0" << std::endl;
    std::cout << " Theta* navigace – fyzická platforma" << std::endl;
    std::cout << "============================================" << std::endl;

    // Cesta ke konfiguračnímu souboru (první argument nebo výchozí)
    std::string config_path = "config/config.yaml";
    if (argc > 1) {
        config_path = argv[1];
    }

    {
        std::ifstream f(config_path);
        if (!f.good()) {
            std::cerr << "[Main] CHYBA: Konfigurační soubor nenalezen: "
                      << config_path << std::endl;
            std::cerr << "  Použití: " << argv[0] << " [cesta/ke/config.yaml]" << std::endl;
            return 1;
        }
    }

    // Načtení konfigurace ze YAML souboru
    YAML::Node cfg;
    try {
        cfg = YAML::LoadFile(config_path);
        std::cout << "[Main] Konfigurace načtena: " << config_path << std::endl;
    } catch (const YAML::Exception& e) {
        std::cerr << "[Main] CHYBA při načítání konfigurace: " << e.what() << std::endl;
        return 1;
    }

    // Inicializace modulů systému
    std::cout << "[Main] Inicializuji moduly..." << std::endl;

    std::unique_ptr<stc::LidarMasking>  masking;
    std::unique_ptr<stc::SlamOdometry>  slam;
    std::unique_ptr<stc::MavlinkBridge> bridge;
    std::unique_ptr<stc::TfUtils>       tf;
    std::unique_ptr<stc::Watchdog>      watchdog;

    try {
        masking  = std::make_unique<stc::LidarMasking>(cfg);
        slam     = std::make_unique<stc::SlamOdometry>(cfg);
        bridge   = std::make_unique<stc::MavlinkBridge>(cfg);
        tf       = std::make_unique<stc::TfUtils>(cfg);
        watchdog = std::make_unique<stc::Watchdog>(cfg);
    } catch (const std::exception& e) {
        std::cerr << "[Main] CHYBA při inicializaci: " << e.what() << std::endl;
        return 1;
    }

    watchdog->setAlertCallback([](stc::Watchdog::Status status,
                                   const std::string& reason) {
        std::cerr << "\033[31m[WATCHDOG ALERT] " << reason << "\033[0m" << std::endl;
    });

    // Připojení k řídicí jednotce letu (Pixhawk)
    std::cout << "[Main] Připojuji se k Pixhawku..." << std::endl;
    if (!bridge->connect()) {
        std::cerr << "[Main] CHYBA: Nelze se připojit k FC!" << std::endl;
        return 1;
    }

    watchdog->startMonitoring(*bridge);

    SharedData shared;

    // Spuštění vlákna pro příjem LiDAR dat
    std::thread lidar_t([&]() {
        lidarThread(shared, *masking, *tf, *slam, *watchdog, cfg);
    });

    // Čekání na první LiDAR data a inicializaci SLAM
    std::cout << "[Main] Čekám na první LiDAR data..." << std::endl;
    for (int i = 0; i < 50; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (slam->isInitialized()) {
            std::cout << "[Main] LiDAR inicializován." << std::endl;
            break;
        }
        if (!g_running.load()) break;
    }

    // Armování UAV
    if (g_running.load()) {
        std::cout << "[Main] Odjišťuji motory..." << std::endl;
        if (!bridge->arm()) {
            std::cerr << "[Main] ARM selhal!" << std::endl;
            g_running.store(false);
        }
    }

    // Vzlet
    if (g_running.load()) {
        std::cout << "[Main] Vzlétám..." << std::endl;
        if (!bridge->takeoff()) {
            std::cerr << "[Main] TAKEOFF selhal!" << std::endl;
            g_running.store(false);
        }
    }

    // Spuštění navigační smyčky
    if (g_running.load()) {
        missionController(shared, *bridge, *slam, *tf, cfg);
    }

    // Přistání a čisté ukončení
    std::cout << "[Main] Mise dokončena – přistávám." << std::endl;
    g_running.store(false);

    if (bridge->getState() != stc::MavlinkBridge::State::LANDED &&
        bridge->getState() != stc::MavlinkBridge::State::DISCONNECTED) {
        bridge->land();
    }

    watchdog->stopMonitoring();

    if (lidar_t.joinable()) {
        lidar_t.join();
    }

    std::cout << "[Main] Aplikace ukončena." << std::endl;
    return 0;
}
