/*
 * DIPLOMOVÁ PRÁCE: Návrh systému pro autonomně bezkolizní řízení UAV
 * Příloha C
 * Autor: rtm. Richard Línek
 * Fyzická platforma (sim-to-real přenos)
 *
 * Soubor obsahuje rozhraní bezpečnostního watchdogu (Watchdog):
 *   - Monitorování dostupnosti LiDAR dat a SLAM odometrie
 *   - Failsafe akce při výpadku dat: přechod do LOITER nebo LAND
 *   - Eskalace: automatické přistání po uplynutí LOITER timeoutu
 *
 * Použití:
 *   watchdog.feedLidar();             // volat při každém přijatém mračnu bodů
 *   watchdog.feedSlam();              // volat při každé úspěšné ICP aktualizaci
 *   watchdog.startMonitoring(bridge); // spustí monitorovací vlákno
 */

#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <yaml-cpp/yaml.h>

namespace stc {

class MavlinkBridge;

// ============================================================
// Watchdog – bezpečnostní monitorování senzorických dat
//
// Stav "ztráta dat" nastane pokud uplyne více než timeout_ms
// bez aktualizace příslušného zdroje dat.
// ============================================================
class Watchdog {
public:
    enum class Status {
        OK,
        LIDAR_LOST,
        SLAM_LOST,
        CRITICAL
    };

    // Konfigurace (z config.yaml, sekce watchdog)
    struct Config {
        int         lidar_timeout_ms          = 500;
        int         slam_timeout_ms           = 500;
        std::string action_on_loss            = "LOITER";
        int         loiter_to_land_timeout_ms = 5000;
    };

    using AlertCallback = std::function<void(Status, const std::string& reason)>;

    explicit Watchdog(const YAML::Node& cfg);
    ~Watchdog();

    Watchdog(const Watchdog&) = delete;
    Watchdog& operator=(const Watchdog&) = delete;

    // Signalizuje příjem LiDAR mračna bodů
    void feedLidar();

    // Signalizuje úspěšnou aktualizaci SLAM odometrie
    void feedSlam();

    // Spustí monitorovací vlákno
    void startMonitoring(MavlinkBridge& bridge);

    // Zastaví monitorovací vlákno
    void stopMonitoring();

    // Nastaví callback volaný při detekci výpadku dat
    void setAlertCallback(AlertCallback cb) { alert_cb_ = std::move(cb); }

    Status getStatus() const { return status_.load(); }
    bool isLidarAlive() const;
    bool isSlamAlive() const;
    const Config& getConfig() const { return cfg_; }

private:
    Config         cfg_;
    MavlinkBridge* bridge_ptr_{nullptr};

    std::atomic<std::chrono::steady_clock::time_point::rep>
        last_lidar_feed_ns_{0};
    std::atomic<std::chrono::steady_clock::time_point::rep>
        last_slam_feed_ns_{0};

    std::atomic<Status> status_{Status::OK};
    std::atomic<bool>   running_{false};
    std::thread         monitor_thread_;
    AlertCallback       alert_cb_;

    std::chrono::steady_clock::time_point failsafe_start_;
    bool                                  in_failsafe_{false};

    void monitorLoop();
    void triggerFailsafe(const std::string& reason);
    long long msSinceLastLidar() const;
    long long msSinceLastSlam()  const;

    static void log(const std::string& msg);
    static void logWarn(const std::string& msg);
};

} // namespace stc
