/*
 * DIPLOMOVÁ PRÁCE: Návrh systému pro autonomně bezkolizní řízení UAV
 * Příloha C
 * Autor: rtm. Richard Línek
 * Fyzická platforma (sim-to-real přenos)
 *
 * Soubor obsahuje implementaci MAVLink mostu (MavlinkBridge):
 *   - Správa připojení přes MAVSDK (sériový port UART nebo UDP)
 *   - Řízení mise: armování, vzlet, Offboard navigace, přistání
 *   - Asynchronní příjem telemetrie (poloha GPS, výška, orientace IMU)
 *   - Failsafe logika (LOITER nebo LAND při výpadku dat)
 */

#include "mavlink_bridge.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <mutex>

namespace stc {

// ============================================================
// Konstruktor – načte konfiguraci z YAML
// ============================================================
MavlinkBridge::MavlinkBridge(const YAML::Node& yaml)
{
    try {
        const auto& mv = yaml["mavlink"];
        cfg_.connection_url               = mv["connection_url"].as<std::string>(
                                                "serial:///dev/ttyTHS1:921600");
        cfg_.takeoff_altitude_m           = mv["takeoff_altitude_m"].as<float>(3.0f);
        cfg_.waypoint_acceptance_radius_m = mv["waypoint_acceptance_radius_m"].as<float>(0.5f);
        cfg_.max_horizontal_speed_ms      = mv["max_horizontal_speed_ms"].as<float>(2.0f);
        cfg_.max_vertical_speed_ms        = mv["max_vertical_speed_ms"].as<float>(1.0f);
        cfg_.yaw_deg                      = mv["yaw_deg"].as<float>(0.0f);
        cfg_.connection_timeout_s         = mv["connection_timeout_s"].as<double>(10.0);

        log("[MavlinkBridge] Konfigurace načtena.");
        log("  Připojení: " + cfg_.connection_url);
        log("  Vzletová výška: " + std::to_string(cfg_.takeoff_altitude_m) + " m");
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(
            std::string("[MavlinkBridge] Chyba při čtení YAML: ") + e.what());
    }
}

MavlinkBridge::~MavlinkBridge()
{
    State s = state_.load();
    if (s == State::IN_FLIGHT || s == State::TAKING_OFF || s == State::LOITERING) {
        logError("[MavlinkBridge] Destruktor – iniciuji nouzové přistání.");
        land();
    }
}

// ============================================================
// connect – připojení k řídicí jednotce a čekání na heartbeat
// ============================================================
bool MavlinkBridge::connect()
{
    log("[MavlinkBridge] Připojuji se k FC: " + cfg_.connection_url);

    mavsdk_ = std::make_unique<mavsdk::Mavsdk>();

    mavsdk::ConnectionResult conn_result =
        mavsdk_->add_any_connection(cfg_.connection_url);

    if (conn_result != mavsdk::ConnectionResult::Success) {
        logError("[MavlinkBridge] Selhalo připojení: "
                 + std::to_string(static_cast<int>(conn_result)));
        state_.store(State::ERROR);
        return false;
    }

    log("[MavlinkBridge] Čekám na heartbeat z FC...");
    auto timeout = std::chrono::seconds(static_cast<int>(cfg_.connection_timeout_s));
    auto start   = std::chrono::steady_clock::now();

    while (mavsdk_->systems().empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (std::chrono::steady_clock::now() - start > timeout) {
            logError("[MavlinkBridge] Timeout čekání na heartbeat!");
            state_.store(State::ERROR);
            return false;
        }
    }

    system_ = mavsdk_->systems().at(0);
    if (!system_->is_connected()) {
        logError("[MavlinkBridge] Systém není připojen.");
        state_.store(State::ERROR);
        return false;
    }

    action_    = std::make_unique<mavsdk::Action>(system_);
    offboard_  = std::make_unique<mavsdk::Offboard>(system_);
    telemetry_ = std::make_unique<mavsdk::Telemetry>(system_);

    registerTelemetryCallbacks();

    state_.store(State::CONNECTED);
    log("[MavlinkBridge] Připojeno k FC. Systém ID: "
        + std::to_string(system_->get_system_id()));
    return true;
}

// ============================================================
// arm – armování motorů UAV
// ============================================================
bool MavlinkBridge::arm()
{
    if (state_.load() != State::CONNECTED) {
        logError("[MavlinkBridge] arm(): Není připojeno.");
        return false;
    }

    log("[MavlinkBridge] Odjišťuji motory...");
    auto result = action_->arm();
    if (result != mavsdk::Action::Result::Success) {
        logError("[MavlinkBridge] arm() selhalo: "
                 + std::to_string(static_cast<int>(result)));
        return false;
    }

    state_.store(State::ARMED);
    log("[MavlinkBridge] Motory odjištěny.");
    return true;
}

// ============================================================
// takeoff – vzlet do nakonfigurované výšky
// ============================================================
bool MavlinkBridge::takeoff()
{
    if (state_.load() != State::ARMED) {
        logError("[MavlinkBridge] takeoff(): Motory nejsou odjištěny.");
        return false;
    }

    log("[MavlinkBridge] Vzlétám do " +
        std::to_string(cfg_.takeoff_altitude_m) + " m...");

    action_->set_takeoff_altitude(cfg_.takeoff_altitude_m);
    auto result = action_->takeoff();

    if (result != mavsdk::Action::Result::Success) {
        logError("[MavlinkBridge] takeoff() selhalo: "
                 + std::to_string(static_cast<int>(result)));
        return false;
    }

    state_.store(State::TAKING_OFF);

    // Čekání na dosažení cílové výšky (polling telemetrie, max 30 s)
    log("[MavlinkBridge] Čekám na dosažení výšky...");
    for (int i = 0; i < 60; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        auto alt = getAltitudeNed();
        if (alt.has_value()) {
            float alt_m = -alt.value();  // NED: záporné = výše
            if (alt_m >= cfg_.takeoff_altitude_m * 0.85f) {
                log("[MavlinkBridge] Výška dosažena: " +
                    std::to_string(alt_m) + " m.");
                break;
            }
        }
    }

    state_.store(State::IN_FLIGHT);
    log("[MavlinkBridge] Vzlet dokončen.");
    return true;
}

// ============================================================
// startOffboard – aktivace Offboard módu
// Před aktivací je nutné odeslat alespoň jeden setpoint
// ============================================================
bool MavlinkBridge::startOffboard()
{
    if (state_.load() != State::IN_FLIGHT) {
        logError("[MavlinkBridge] startOffboard(): UAV není ve vzduchu.");
        return false;
    }

    mavsdk::Offboard::PositionNedYaw hold{};
    hold.north_m = 0.0f;
    hold.east_m  = 0.0f;
    hold.down_m  = -cfg_.takeoff_altitude_m;
    hold.yaw_deg = cfg_.yaw_deg;
    offboard_->set_position_ned(hold);

    auto result = offboard_->start();
    if (result != mavsdk::Offboard::Result::Success) {
        logError("[MavlinkBridge] startOffboard() selhalo: "
                 + std::to_string(static_cast<int>(result)));
        return false;
    }

    log("[MavlinkBridge] Offboard mód aktivován.");
    return true;
}

// ============================================================
// sendWaypoint – odeslání cílového NED setpointu
// Volat min. 2 Hz (timeout Offboard módu ≈ 500 ms)
// ============================================================
bool MavlinkBridge::sendWaypoint(const Waypoint& wp)
{
    State s = state_.load();
    if (s != State::IN_FLIGHT && s != State::LOITERING) {
        logError("[MavlinkBridge] sendWaypoint(): Neplatný stav: "
                 + getStateString());
        return false;
    }

    mavsdk::Offboard::PositionNedYaw setpoint{};
    setpoint.north_m = wp.x;
    setpoint.east_m  = wp.y;
    setpoint.down_m  = wp.z;

    // NaN → zachovat aktuální yaw; jinak převod z radiánů na stupně
    if (std::isnan(wp.yaw)) {
        setpoint.yaw_deg = std::numeric_limits<float>::quiet_NaN();
    } else {
        setpoint.yaw_deg = wp.yaw * 180.0f / static_cast<float>(M_PI);
    }

    auto result = offboard_->set_position_ned(setpoint);
    if (result != mavsdk::Offboard::Result::Success) {
        logError("[MavlinkBridge] sendWaypoint() selhalo.");
        return false;
    }

    if (s == State::LOITERING) {
        state_.store(State::IN_FLIGHT);
    }
    return true;
}

// ============================================================
// land – přistání na aktuální pozici
// ============================================================
bool MavlinkBridge::land()
{
    log("[MavlinkBridge] Přistávám...");

    if (isOffboard()) {
        offboard_->stop();
    }

    auto result = action_->land();
    if (result != mavsdk::Action::Result::Success) {
        logError("[MavlinkBridge] land() selhalo: "
                 + std::to_string(static_cast<int>(result)));
        state_.store(State::ERROR);
        return false;
    }

    state_.store(State::LANDING);

    // FC automaticky odarmuje motory po přistání
    for (int i = 0; i < 120; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (!isArmed()) {
            break;
        }
    }

    state_.store(State::LANDED);
    log("[MavlinkBridge] Přistání dokončeno.");
    return true;
}

// ============================================================
// disarm – odarmování motorů
// ============================================================
bool MavlinkBridge::disarm()
{
    if (state_.load() != State::LANDED) {
        logError("[MavlinkBridge] disarm(): UAV není na zemi!");
        return false;
    }
    auto result = action_->disarm();
    if (result != mavsdk::Action::Result::Success) {
        logError("[MavlinkBridge] disarm() selhalo.");
        return false;
    }
    log("[MavlinkBridge] Motory zajištěny.");
    return true;
}

// ============================================================
// loiter – failsafe: udržení aktuální pozice UAV
// ============================================================
bool MavlinkBridge::loiter()
{
    log("[MavlinkBridge] FAILSAFE: Přecházím do LOITER.");

    auto result = action_->hold();
    if (result == mavsdk::Action::Result::Success) {
        state_.store(State::LOITERING);
        log("[MavlinkBridge] LOITER aktivován.");
        return true;
    }

    // Záložní řešení: Offboard setpoint s nulovou rychlostí
    mavsdk::Offboard::PositionNedYaw hold{};
    hold.north_m = 0.0f;
    hold.east_m  = 0.0f;
    hold.down_m  = -cfg_.takeoff_altitude_m;
    hold.yaw_deg = cfg_.yaw_deg;
    offboard_->set_position_ned(hold);
    state_.store(State::LOITERING);
    return true;
}

// ============================================================
// Gettery telemetrie (vláknově bezpečné)
// ============================================================
std::optional<float> MavlinkBridge::getAltitudeNed() const
{
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    return last_altitude_ned_;
}

std::optional<mavsdk::Telemetry::Position> MavlinkBridge::getGpsPosition() const
{
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    return last_gps_pos_;
}

std::optional<mavsdk::Telemetry::EulerAngle> MavlinkBridge::getAttitude() const
{
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    return last_attitude_;
}

bool MavlinkBridge::isOffboard() const
{
    if (!offboard_) return false;
    return offboard_->is_active();
}

bool MavlinkBridge::isArmed() const
{
    if (!telemetry_) return false;
    return telemetry_->armed();
}

std::string MavlinkBridge::getStateString() const
{
    switch (state_.load()) {
        case State::DISCONNECTED: return "DISCONNECTED";
        case State::CONNECTED:    return "CONNECTED";
        case State::ARMED:        return "ARMED";
        case State::TAKING_OFF:   return "TAKING_OFF";
        case State::IN_FLIGHT:    return "IN_FLIGHT";
        case State::LOITERING:    return "LOITERING";
        case State::LANDING:      return "LANDING";
        case State::LANDED:       return "LANDED";
        case State::ERROR:        return "ERROR";
        default:                  return "UNKNOWN";
    }
}

// ============================================================
// registerTelemetryCallbacks – asynchronní příjem dat z FC
// ============================================================
void MavlinkBridge::registerTelemetryCallbacks()
{
    // Výška relativní k bodu armování (NED)
    telemetry_->subscribe_position([this](mavsdk::Telemetry::Position pos) {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        last_gps_pos_      = pos;
        last_altitude_ned_ = -pos.relative_altitude_m;  // NED: kladné = dolů
    });

    // Orientace UAV (Euler RPY z IMU)
    telemetry_->subscribe_attitude_euler([this](mavsdk::Telemetry::EulerAngle euler) {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        last_attitude_ = euler;
    });

    // Frekvence telemetrických aktualizací
    telemetry_->set_rate_position(5.0);     // 5 Hz
    telemetry_->set_rate_attitude_async(10.0, [](mavsdk::Telemetry::Result result) {}); // 10 Hz
}

// ============================================================
void MavlinkBridge::log(const std::string& msg)
{
    std::cout << msg << std::endl;
}

void MavlinkBridge::logError(const std::string& msg)
{
    std::cerr << msg << std::endl;
}

} // namespace stc
