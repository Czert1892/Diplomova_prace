/*
 * DIPLOMOVÁ PRÁCE: Návrh systému pro autonomně bezkolizní řízení UAV
 * Příloha C
 * Autor: rtm. Richard Línek
 * Fyzická platforma (sim-to-real přenos)
 *
 * Soubor obsahuje rozhraní MAVLink mostu (MavlinkBridge):
 *   - Připojení k řídicí jednotce letu Pixhawk přes MAVSDK
 *   - Řízení mise: armování, vzlet, Offboard waypoint navigace, přistání
 *   - Failsafe akce při výpadku dat: přechod do LOITER nebo LAND
 *   - Asynchronní příjem telemetrických dat (poloha, výška, orientace IMU)
 *
 * Sekvence mise:
 *   1. connect()       – připojení a čekání na heartbeat
 *   2. arm()           – armování motorů
 *   3. takeoff()       – vzlet do cílové výšky
 *   4. startOffboard() – aktivace Offboard módu
 *   5. sendWaypoint()  – opakované odesílání SET_POSITION_TARGET_LOCAL_NED
 *   6. land()          – přistání na aktuální pozici
 *   7. disarm()        – odarmování motorů
 *
 * Závislosti: MAVSDK >= 1.4
 */

#pragma once

#include <mutex>
#include <string>
#include <atomic>
#include <chrono>
#include <memory>
#include <functional>
#include <optional>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <yaml-cpp/yaml.h>

namespace stc {

// ============================================================
// MavlinkBridge – komunikace s Pixhawkem přes MAVLink (MAVSDK)
// ============================================================
class MavlinkBridge {
public:
    // Stavy mise
    enum class State {
        DISCONNECTED,
        CONNECTED,
        ARMED,
        TAKING_OFF,
        IN_FLIGHT,
        LOITERING,
        LANDING,
        LANDED,
        ERROR
    };

    // Konfigurace (z config.yaml, sekce mavlink)
    struct Config {
        std::string connection_url               = "serial:///dev/ttyTHS1:921600";
        float       takeoff_altitude_m           = 3.0f;
        float       waypoint_acceptance_radius_m = 0.5f;
        float       max_horizontal_speed_ms      = 2.0f;
        float       max_vertical_speed_ms        = 1.0f;
        float       yaw_deg                      = 0.0f;
        double      connection_timeout_s         = 10.0;
    };

    // Waypoint v lokálním NED frame (relativní k bodu armování)
    struct Waypoint {
        float x;    // [m] North (dopředu)
        float y;    // [m] East  (doprava)
        float z;    // [m] Down  (záporné = výše)
        float yaw;  // [rad] heading (NaN = zachovat aktuální)
    };

    explicit MavlinkBridge(const YAML::Node& cfg);
    ~MavlinkBridge();

    MavlinkBridge(const MavlinkBridge&) = delete;
    MavlinkBridge& operator=(const MavlinkBridge&) = delete;

    // Připojí se k řídicí jednotce letu a čeká na heartbeat
    bool connect();

    // Armuje motory UAV
    bool arm();

    // Vzlet do výšky definované v konfiguraci
    bool takeoff();

    // Aktivuje Offboard mód (nutné před odesíláním waypointů)
    bool startOffboard();

    // Odešle cílový waypoint do řídicí jednotky
    // Volat opakovaně (min. 2 Hz, timeout Offboard módu ≈ 500 ms)
    bool sendWaypoint(const Waypoint& wp);

    // Přistání na aktuální pozici
    bool land();

    // Odarmování motorů
    bool disarm();

    // Failsafe: udržení aktuální pozice UAV
    bool loiter();

    // Vrátí aktuální výšku v NED frame [m]
    std::optional<float> getAltitudeNed() const;

    // Vrátí aktuální GPS polohu
    std::optional<mavsdk::Telemetry::Position> getGpsPosition() const;

    // Vrátí aktuální orientaci UAV (Euler RPY) z IMU
    std::optional<mavsdk::Telemetry::EulerAngle> getAttitude() const;

    // Vrátí true pokud je aktivní Offboard mód
    bool isOffboard() const;

    // Vrátí aktuální stav mise
    State getState() const { return state_.load(); }
    std::string getStateString() const;

    // Vrátí true pokud jsou motory armovány
    bool isArmed() const;

private:
    Config cfg_;

    std::unique_ptr<mavsdk::Mavsdk>     mavsdk_;
    std::shared_ptr<mavsdk::System>     system_;
    std::unique_ptr<mavsdk::Action>     action_;
    std::unique_ptr<mavsdk::Offboard>   offboard_;
    std::unique_ptr<mavsdk::Telemetry>  telemetry_;

    std::atomic<State> state_{State::DISCONNECTED};

    mutable std::mutex               telemetry_mutex_;
    mavsdk::Telemetry::Position      last_gps_pos_{};
    mavsdk::Telemetry::EulerAngle    last_attitude_{};
    float                            last_altitude_ned_{0.0f};

    // Zaregistruje asynchronní telemetrické callbacky
    void registerTelemetryCallbacks();

    static void log(const std::string& msg);
    static void logError(const std::string& msg);
};

} // namespace stc
