/*
 * DIPLOMOVÁ PRÁCE: Návrh systému pro autonomně bezkolizní řízení UAV
 * Příloha C
 * Autor: rtm. Richard Línek
 * Fyzická platforma (sim-to-real přenos)
 *
 * Soubor obsahuje implementaci ICP-based LiDAR odometrie:
 *   - Frame-to-keyframe ICP (PCL::IterativeClosestPoint)
 *   - VoxelGrid downsample pro snížení výpočetní zátěže na Jetson Nano
 *   - Keyframe strategie pro omezení akumulovaného driftu
 *   - Odmítnutí divergentních ICP odhadů (fitness threshold 0.25)
 */

#include "slam_odometry.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace stc {

// ============================================================
// Pose – konverze na Eigen Matrix4f a zpět
// ============================================================
Eigen::Matrix4f Pose::toMatrix() const
{
    // Rotace pouze kolem osy Z (yaw); roll a pitch kompenzuje IMU
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    float cy = std::cos(yaw);
    float sy = std::sin(yaw);
    T(0, 0) =  cy;  T(0, 1) = sy;
    T(1, 0) = -sy;  T(1, 1) = cy;
    T(0, 3) = x;
    T(1, 3) = y;
    T(2, 3) = z;
    return T;
}

Pose Pose::fromMatrix(const Eigen::Matrix4f& mat)
{
    Pose p;
    p.x   = mat(0, 3);
    p.y   = mat(1, 3);
    p.z   = mat(2, 3);
    p.yaw = std::atan2(mat(1, 0), mat(0, 0));
    return p;
}

// ============================================================
// Konstruktor – načte konfiguraci z YAML a inicializuje ICP
// ============================================================
SlamOdometry::SlamOdometry(const YAML::Node& yaml)
    : accumulated_transform_(Eigen::Matrix4f::Identity())
    , last_update_time_(std::chrono::steady_clock::now())
{
    try {
        const auto& s = yaml["slam"];
        cfg_.voxel_leaf_size             = s["voxel_leaf_size"].as<float>(0.15f);
        cfg_.icp_max_iterations          = s["icp_max_iterations"].as<int>(30);
        cfg_.icp_max_correspondence_dist = s["icp_max_correspondence_dist"].as<float>(0.5f);
        cfg_.icp_transformation_eps      = s["icp_transformation_eps"].as<float>(1e-6f);
        cfg_.icp_euclidean_fitness_eps   = s["icp_euclidean_fitness_eps"].as<float>(0.01f);
        cfg_.icp_fitness_threshold       = s["icp_fitness_threshold"].as<float>(0.25f);
        cfg_.keyframe_min_dist_m         = s["keyframe_min_dist_m"].as<float>(0.3f);
        cfg_.keyframe_min_angle_rad      = s["keyframe_min_angle_rad"].as<float>(0.087f);

        if (s["initial_pose"]) {
            auto ip = s["initial_pose"].as<std::vector<float>>();
            if (ip.size() >= 4) {
                cfg_.initial_pose = {ip[0], ip[1], ip[2], ip[3]};
            }
        }
        log("[SlamOdometry] Konfigurace načtena z YAML.");
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(
            std::string("[SlamOdometry] Chyba při čtení YAML: ") + e.what());
    }

    // Parametry ICP
    icp_.setMaximumIterations(cfg_.icp_max_iterations);
    icp_.setMaxCorrespondenceDistance(cfg_.icp_max_correspondence_dist);
    icp_.setTransformationEpsilon(cfg_.icp_transformation_eps);
    icp_.setEuclideanFitnessEpsilon(cfg_.icp_euclidean_fitness_eps);

    // VoxelGrid downsample
    voxel_.setLeafSize(cfg_.voxel_leaf_size,
                       cfg_.voxel_leaf_size,
                       cfg_.voxel_leaf_size);

    current_pose_ = cfg_.initial_pose;
    accumulated_transform_ = current_pose_.toMatrix();

    log("[SlamOdometry] ICP odometrie inicializována.");
    log("  Voxel leaf:  " + std::to_string(cfg_.voxel_leaf_size) + " m");
    log("  ICP iter:    " + std::to_string(cfg_.icp_max_iterations));
    log("  Fitness thr: " + std::to_string(cfg_.icp_fitness_threshold));
}

// ============================================================
// update – zpracuje nový cloud a aktualizuje odhad polohy
// ============================================================
bool SlamOdometry::update(const CloudTPtr& raw_cloud)
{
    if (!raw_cloud || raw_cloud->empty()) {
        log("[SlamOdometry] WARN: Prázdný cloud, přeskočeno.");
        return false;
    }

    CloudTPtr current_ds = downsample(raw_cloud);
    if (current_ds->size() < 50) {
        log("[SlamOdometry] WARN: Příliš málo bodů po downsamplování ("
            + std::to_string(current_ds->size()) + "), přeskočeno.");
        return false;
    }

    // První snímek – inicializace keyframe, ICP se neprovádí
    if (!initialized_.load()) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        keyframe_cloud_ = current_ds;
        last_update_time_ = std::chrono::steady_clock::now();
        initialized_.store(true);
        log("[SlamOdometry] Inicializace keyframe (" +
            std::to_string(current_ds->size()) + " bodů).");
        return true;
    }

    icp_.setInputSource(current_ds);
    icp_.setInputTarget(keyframe_cloud_);

    CloudTPtr aligned(new CloudT());
    icp_.align(*aligned, accumulated_transform_);

    if (!icp_.hasConverged()) {
        log("[SlamOdometry] WARN: ICP nekonvergoval.");
        return false;
    }

    float fitness = static_cast<float>(icp_.getFitnessScore());
    if (fitness > cfg_.icp_fitness_threshold) {
        log("[SlamOdometry] WARN: ICP fitness příliš vysoký ("
            + std::to_string(fitness) + " > "
            + std::to_string(cfg_.icp_fitness_threshold)
            + "), transformace odmítnuta.");
        return false;
    }

    Eigen::Matrix4f T = icp_.getFinalTransformation();

    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        accumulated_transform_ = T;
        current_pose_          = Pose::fromMatrix(T);
        last_fitness_          = fitness;
        last_update_time_      = std::chrono::steady_clock::now();
    }

    // Aktualizace keyframe při dostatečném pohybu
    if (shouldUpdateKeyframe(T)) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        keyframe_cloud_ = current_ds;
        log("[SlamOdometry] Keyframe aktualizován. Pose: x="
            + std::to_string(current_pose_.x) + " y="
            + std::to_string(current_pose_.y) + " z="
            + std::to_string(current_pose_.z) + " yaw="
            + std::to_string(current_pose_.yaw));
    }

    return true;
}

// ============================================================
// shouldUpdateKeyframe – rozhodnutí o aktualizaci keyframe
// ============================================================
bool SlamOdometry::shouldUpdateKeyframe(const Eigen::Matrix4f& T)
{
    float dx   = T(0, 3);
    float dy   = T(1, 3);
    float dz   = T(2, 3);
    float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    float dyaw = std::abs(std::atan2(T(1, 0), T(0, 0)));

    return (dist > cfg_.keyframe_min_dist_m) ||
           (dyaw > cfg_.keyframe_min_angle_rad);
}

// ============================================================
// downsample – VoxelGrid downsample mračna bodů
// ============================================================
CloudTPtr SlamOdometry::downsample(const CloudTPtr& cloud)
{
    CloudTPtr out(new CloudT());
    voxel_.setInputCloud(cloud);
    voxel_.filter(*out);
    return out;
}

// ============================================================
// getCurrentPose – vláknově bezpečný getter
// ============================================================
Pose SlamOdometry::getCurrentPose() const
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return current_pose_;
}

// ============================================================
// getLastUpdateTime – vláknově bezpečný getter
// ============================================================
std::chrono::steady_clock::time_point SlamOdometry::getLastUpdateTime() const
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return last_update_time_;
}

// ============================================================
// reset – resetuje odometrii na zadanou polohu
// ============================================================
void SlamOdometry::reset(const Pose& pose)
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_pose_          = pose;
    accumulated_transform_ = pose.toMatrix();
    keyframe_cloud_.reset();
    initialized_.store(false);
    log("[SlamOdometry] Odometrie resetována.");
}

// ============================================================
void SlamOdometry::log(const std::string& msg)
{
    std::cout << msg << std::endl;
}

} // namespace stc
