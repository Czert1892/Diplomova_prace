/*
 * DIPLOMOVÁ PRÁCE: Návrh systému pro autonomně bezkolizní řízení UAV
 * Příloha C
 * Autor: rtm. Richard Línek
 * Fyzická platforma (sim-to-real přenos)
 *
 * Soubor obsahuje implementaci bezpečnostního watchdogu:
 *   - Monitorovací vlákno s periodou 100 ms
 *   - Detekce výpadku LiDAR dat nebo SLAM odometrie
 *   - Failsafe: LOITER při výpadku, LAND po uplynutí LOITER timeoutu
 */

#include "watchdog.hpp"
#include "mavlink_bridge.hpp"

#include <iostream>

namespace stc {

static inline std::chrono::steady_clock::time_point::rep nowNs()
{
    return std::chrono::steady_clock::now().time_since_epoch().count();
}

// ============================================================
// Konstruktor – načte konfiguraci z YAML
// ============================================================
Watchdog::Watchdog(const YAML::Node& yaml)
{
    try {
        const auto& wd = yaml["watchdog"];
        cfg_.lidar_timeout_ms          = wd["lidar_timeout_ms"].as<int>(500);
        cfg_.slam_timeout_ms           = wd["slam_timeout_ms"].as<int>(500);
        cfg_.action_on_loss            = wd["action_on_loss"].as<std::string>("LOITER");
        cfg_.loiter_to_land_timeout_ms = wd["loiter_to_land_timeout_ms"].as<int>(5000);

        log("[Watchdog] Konfigurace načtena.");
        log("  LiDAR timeout: " + std::to_string(cfg_.lidar_timeout_ms) + " ms");
        log("  SLAM  timeout: " + std::to_string(cfg_.slam_timeout_ms)  + " ms");
        log("  Akce při výpadku: " + cfg_.action_on_loss);
    } catch (const YAML::Exception& e) {
        std::cerr << "[Watchdog] Chyba při čtení YAML: " << e.what()
                  << " – používám výchozí hodnoty." << std::endl;
    }

    // Inicializace časovačů na aktuální čas (zabraňuje falešnému alarmu při startu)
    last_lidar_feed_ns_.store(nowNs());
    last_slam_feed_ns_.store(nowNs());
}

// ============================================================
// Destruktor – zastaví monitorovací vlákno
// ============================================================
Watchdog::~Watchdog()
{
    stopMonitoring();
}

// ============================================================
// feedLidar / feedSlam – potvrzení příjmu dat ze senzorů
// ============================================================
void Watchdog::feedLidar()
{
    last_lidar_feed_ns_.store(nowNs());
}

void Watchdog::feedSlam()
{
    last_slam_feed_ns_.store(nowNs());
}

// ============================================================
// startMonitoring – spustí monitorovací vlákno
// ============================================================
void Watchdog::startMonitoring(MavlinkBridge& bridge)
{
    bridge_ptr_ = &bridge;
    running_.store(true);
    monitor_thread_ = std::thread(&Watchdog::monitorLoop, this);
    log("[Watchdog] Monitorovací vlákno spuštěno.");
}

// ============================================================
// stopMonitoring – zastaví monitorovací vlákno
// ============================================================
void Watchdog::stopMonitoring()
{
    running_.store(false);
    if (monitor_thread_.joinable()) {
        monitor_thread_.join();
        log("[Watchdog] Monitorovací vlákno zastaveno.");
    }
}

// ============================================================
// monitorLoop – hlavní smyčka monitorování (perioda 100 ms)
// ============================================================
void Watchdog::monitorLoop()
{
    while (running_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        bool lidar_ok = isLidarAlive();
        bool slam_ok  = isSlamAlive();

        Status new_status = Status::OK;
        std::string reason;

        if (!lidar_ok && !slam_ok) {
            new_status = Status::CRITICAL;
            reason = "Výpadek LiDAR i SLAM dat!";
        } else if (!lidar_ok) {
            new_status = Status::LIDAR_LOST;
            reason = "Výpadek LiDAR dat (>"
                     + std::to_string(cfg_.lidar_timeout_ms) + " ms)";
        } else if (!slam_ok) {
            new_status = Status::SLAM_LOST;
            reason = "Výpadek SLAM lokalizace (>"
                     + std::to_string(cfg_.slam_timeout_ms) + " ms)";
        }

        Status prev_status = status_.exchange(new_status);

        // Přechod do failsafe při detekci nového výpadku
        if (new_status != Status::OK && prev_status == Status::OK) {
            logWarn("[Watchdog] VÝPADEK: " + reason);
            if (alert_cb_) {
                alert_cb_(new_status, reason);
            }
            triggerFailsafe(reason);
            failsafe_start_ = std::chrono::steady_clock::now();
            in_failsafe_ = true;
        }

        // Obnova dat – ukončení failsafe stavu
        if (new_status == Status::OK && in_failsafe_) {
            log("[Watchdog] Data obnovena – FAILSAFE zrušen.");
            in_failsafe_ = false;
        }

        // Eskalace: LOITER → LAND po uplynutí timeoutu
        if (in_failsafe_ && cfg_.action_on_loss == "LOITER") {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - failsafe_start_).count();
            if (elapsed > cfg_.loiter_to_land_timeout_ms) {
                logWarn("[Watchdog] LOITER timeout překročen – iniciuji přistání.");
                if (bridge_ptr_) {
                    bridge_ptr_->land();
                }
                in_failsafe_ = false;
            }
        }
    }
}

// ============================================================
// triggerFailsafe – spustí nakonfigurovanou failsafe akci
// ============================================================
void Watchdog::triggerFailsafe(const std::string& reason)
{
    if (!bridge_ptr_) {
        logWarn("[Watchdog] Bridge není nastaven – failsafe nelze provést!");
        return;
    }

    logWarn("[Watchdog] FAILSAFE: " + reason);

    if (cfg_.action_on_loss == "LAND") {
        logWarn("[Watchdog] Přistávám (LAND failsafe).");
        bridge_ptr_->land();
    } else {
        logWarn("[Watchdog] Přecházím do LOITER.");
        bridge_ptr_->loiter();
    }
}

// ============================================================
// isLidarAlive / isSlamAlive – kontrola časové dostupnosti dat
// ============================================================
bool Watchdog::isLidarAlive() const
{
    return msSinceLastLidar() <= cfg_.lidar_timeout_ms;
}

bool Watchdog::isSlamAlive() const
{
    return msSinceLastSlam() <= cfg_.slam_timeout_ms;
}

long long Watchdog::msSinceLastLidar() const
{
    auto now_ns  = nowNs();
    auto last_ns = last_lidar_feed_ns_.load();
    return static_cast<long long>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::nanoseconds(now_ns - last_ns)).count());
}

long long Watchdog::msSinceLastSlam() const
{
    auto now_ns  = nowNs();
    auto last_ns = last_slam_feed_ns_.load();
    return static_cast<long long>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::nanoseconds(now_ns - last_ns)).count());
}

// ============================================================
void Watchdog::log(const std::string& msg)
{
    std::cout << msg << std::endl;
}

void Watchdog::logWarn(const std::string& msg)
{
    std::cerr << "\033[33m" << msg << "\033[0m" << std::endl;
}

} // namespace stc
