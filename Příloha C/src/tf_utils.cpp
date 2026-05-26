/*
 * DIPLOMOVÁ PRÁCE: Návrh systému pro autonomně bezkolizní řízení UAV
 * Příloha C
 * Autor: rtm. Richard Línek
 * Fyzická platforma (sim-to-real přenos)
 *
 * Soubor obsahuje implementaci transformací souřadnicových rámců:
 *   - Statická transformace LiDAR → BODY (montážní konfigurace)
 *   - Dynamická transformace BODY → NED (orientace UAV z IMU Pixhawku)
 *   - Pomocné utility pro práci s Euler úhly a PCL mračny bodů
 */

#include "tf_utils.hpp"

#include <iostream>
#include <pcl/common/transforms.h>
#include <mutex>

namespace stc {

// ============================================================
// Konstruktor – načte konfiguraci a sestaví statickou transformaci
// ============================================================
TfUtils::TfUtils(const YAML::Node& yaml)
    : T_lidar_to_body_(Eigen::Matrix4f::Identity())
{
    try {
        const auto& tf = yaml["tf"];

        auto trans_vec = tf["lidar_to_body_translation"].as<std::vector<float>>();
        if (trans_vec.size() >= 3) {
            cfg_.lidar_to_body_translation = {
                trans_vec[0], trans_vec[1], trans_vec[2]};
        }

        auto rpy_vec = tf["lidar_to_body_rotation_rpy"].as<std::vector<float>>();
        if (rpy_vec.size() >= 3) {
            cfg_.lidar_to_body_rotation_rpy = {
                rpy_vec[0], rpy_vec[1], rpy_vec[2]};
        }

        std::cout << "[TfUtils] Konfigurace načtena z YAML." << std::endl;
    } catch (const YAML::Exception& e) {
        std::cerr << "[TfUtils] Chyba při čtení YAML: " << e.what()
                  << " – používám výchozí hodnoty." << std::endl;
    }

    initStaticTransform();
}

// ============================================================
// initStaticTransform – sestaví transformaci LiDAR → BODY
// ============================================================
void TfUtils::initStaticTransform()
{
    const auto& rpy   = cfg_.lidar_to_body_rotation_rpy;
    const auto& trans = cfg_.lidar_to_body_translation;

    Eigen::Matrix3f R = rpyToRotationMatrix(rpy[0], rpy[1], rpy[2]);
    T_lidar_to_body_ = makeTransform(R, trans);

    std::cout << "[TfUtils] Statická transformace LiDAR→BODY inicializována." << std::endl;
    std::cout << "  Translace: [" << trans[0] << ", " << trans[1] << ", "
              << trans[2] << "] m" << std::endl;
    std::cout << "  Rotace RPY: [" << rpy[0] << ", " << rpy[1] << ", "
              << rpy[2] << "] rad" << std::endl;
}

// ============================================================
// updateImuOrientation – aktualizuje dynamickou orientaci UAV
// ============================================================
void TfUtils::updateImuOrientation(float roll_rad, float pitch_rad, float yaw_rad)
{
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_.roll  = roll_rad;
    imu_.pitch = pitch_rad;
    imu_.yaw   = yaw_rad;
}

// ============================================================
// lidarToBody – transformace mračna bodů LiDAR → BODY frame
// ============================================================
CloudTPtr TfUtils::lidarToBody(const CloudTPtr& lidar_cloud)
{
    CloudTPtr result(new CloudT());
    pcl::transformPointCloud(*lidar_cloud, *result, T_lidar_to_body_);
    return result;
}

// ============================================================
// lidarToNed – transformace mračna bodů LiDAR → NED frame
// Kombinuje statickou transformaci LiDAR→BODY s dynamickou BODY→NED
// ============================================================
CloudTPtr TfUtils::lidarToNed(const CloudTPtr& lidar_cloud)
{
    auto body_cloud = lidarToBody(lidar_cloud);

    Eigen::Matrix4f T_body_to_ned = getBodyToNedTransform();
    CloudTPtr result(new CloudT());
    pcl::transformPointCloud(*body_cloud, *result, T_body_to_ned);

    return result;
}

// ============================================================
// getBodyToNedTransform – sestaví BODY → NED z aktuální IMU orientace
// ============================================================
Eigen::Matrix4f TfUtils::getBodyToNedTransform() const
{
    std::lock_guard<std::mutex> lock(imu_mutex_);
    Eigen::Matrix3f R = rpyToRotationMatrix(imu_.roll, imu_.pitch, imu_.yaw);
    return makeTransform(R, Eigen::Vector3f::Zero());
}

// ============================================================
// rpyToRotationMatrix – ZYX Euler úhly → rotační matice 3×3
// Konvence: yaw (Z), pitch (Y), roll (X)
// ============================================================
Eigen::Matrix3f TfUtils::rpyToRotationMatrix(float roll, float pitch, float yaw)
{
    Eigen::AngleAxisf R_roll (roll,  Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf R_pitch(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf R_yaw  (yaw,   Eigen::Vector3f::UnitZ());

    return (R_yaw * R_pitch * R_roll).toRotationMatrix();
}

// ============================================================
// makeTransform – sestaví transformační matici 4×4 z R a t
// ============================================================
Eigen::Matrix4f TfUtils::makeTransform(const Eigen::Matrix3f& rot,
                                        const Eigen::Vector3f& trans)
{
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = rot;
    T.block<3, 1>(0, 3) = trans;
    return T;
}

// ============================================================
// transformPoint – transformuje jeden 3D bod
// ============================================================
Eigen::Vector3f TfUtils::transformPoint(const Eigen::Matrix4f& T,
                                         const Eigen::Vector3f& point)
{
    Eigen::Vector4f p4(point[0], point[1], point[2], 1.0f);
    Eigen::Vector4f result = T * p4;
    return result.head<3>();
}

// ============================================================
// extractYaw – extrahuje yaw úhel z rotační matice
// ============================================================
float TfUtils::extractYaw(const Eigen::Matrix4f& T)
{
    return std::atan2(T(1, 0), T(0, 0));
}

// ============================================================
// nedToEnu / enuToNed – konverze mezi NED a ENU
// NED: X=North, Y=East,  Z=Down
// ENU: X=East,  Y=North, Z=Up
// ============================================================
Eigen::Vector3f TfUtils::nedToEnu(const Eigen::Vector3f& ned)
{
    return {ned[1], ned[0], -ned[2]};
}

Eigen::Vector3f TfUtils::enuToNed(const Eigen::Vector3f& enu)
{
    return {enu[1], enu[0], -enu[2]};
}

} // namespace stc
