/*
 * DIPLOMOVÁ PRÁCE: Návrh systému pro autonomně bezkolizní řízení UAV
 * Příloha C
 * Autor: rtm. Richard Línek
 * Fyzická platforma (sim-to-real přenos)
 *
 * Soubor obsahuje rozhraní pro transformace souřadnicových rámců (TfUtils):
 *   - LiDAR frame → BODY frame (statická transformace z konfigurace)
 *   - BODY frame → NED frame (dynamická transformace z IMU dat Pixhawku)
 *   - Pomocné utility: ZYX Euler → rotační matice, NED ↔ ENU konverze
 *
 * Souřadnicové rámce:
 *   LIDAR frame – střed LiDARu (osa X dopředu, Z nahoru)
 *   BODY frame  – těžiště UAV (NED: X dopředu, Y doprava, Z dolů)
 *   NED frame   – lokální North-East-Down (origin = bod vzletu)
 */

#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <mutex>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

namespace stc {

using PointT    = pcl::PointXYZI;
using CloudT    = pcl::PointCloud<PointT>;
using CloudTPtr = CloudT::Ptr;

// ============================================================
// TfUtils – transformace souřadnicových rámců
// ============================================================
class TfUtils {
public:
    // Statické transformační parametry (z config.yaml, sekce tf)
    struct Config {
        Eigen::Vector3f lidar_to_body_translation{0.0f, 0.0f, -0.10f};
        Eigen::Vector3f lidar_to_body_rotation_rpy{0.0f, 0.0f, 0.0f};
    };

    // Dynamická orientace UAV z IMU (aktualizuje se za běhu)
    struct ImuOrientation {
        float roll  = 0.0f;  // [rad]
        float pitch = 0.0f;  // [rad]
        float yaw   = 0.0f;  // [rad]
    };

    explicit TfUtils(const YAML::Node& cfg);

    // Aktualizuje orientaci UAV z IMU dat (volat po každém IMU snímku)
    void updateImuOrientation(float roll_rad, float pitch_rad, float yaw_rad);

    // Transformuje mračno bodů z LiDAR frame do NED frame
    CloudTPtr lidarToNed(const CloudTPtr& lidar_cloud);

    // Transformuje mračno bodů z LiDAR frame do BODY frame
    CloudTPtr lidarToBody(const CloudTPtr& lidar_cloud);

    // Vytvoří rotační matici ze ZYX Euler úhlů (roll, pitch, yaw)
    static Eigen::Matrix3f rpyToRotationMatrix(float roll, float pitch, float yaw);

    // Sestaví transformační matici 4×4 z rotace a translace
    static Eigen::Matrix4f makeTransform(const Eigen::Matrix3f& rot,
                                         const Eigen::Vector3f& trans);

    // Transformuje jeden bod transformační maticí
    static Eigen::Vector3f transformPoint(const Eigen::Matrix4f& T,
                                          const Eigen::Vector3f& point);

    // Extrahuje yaw úhel z transformační matice 4×4
    static float extractYaw(const Eigen::Matrix4f& T);

    // Konverze NED → ENU (North-East-Down → East-North-Up)
    static Eigen::Vector3f nedToEnu(const Eigen::Vector3f& ned);

    // Konverze ENU → NED
    static Eigen::Vector3f enuToNed(const Eigen::Vector3f& enu);

    const Eigen::Matrix4f& getLidarToBodyTransform() const { return T_lidar_to_body_; }
    Eigen::Matrix4f getBodyToNedTransform() const;

private:
    Config          cfg_;
    ImuOrientation  imu_;
    Eigen::Matrix4f T_lidar_to_body_;
    mutable std::mutex imu_mutex_;

    void initStaticTransform();
};

} // namespace stc
