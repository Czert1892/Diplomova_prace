/*
 * DIPLOMOVÁ PRÁCE: Návrh systému pro autonomně bezkolizní řízení UAV
 * Příloha C
 * Autor: rtm. Richard Línek
 * Fyzická platforma (sim-to-real přenos)
 *
 * Soubor obsahuje rozhraní ICP-based LiDAR odometrie (SlamOdometry):
 *   - Frame-to-keyframe ICP pomocí PCL::IterativeClosestPoint
 *   - Odhad polohy UAV v lokálním NED frame (x, y, z, yaw)
 *   - Keyframe strategie pro omezení akumulovaného driftu
 *   - Vláknově bezpečné rozhraní pro multivláknovou architekturu
 */

#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

namespace stc {

using PointT    = pcl::PointXYZI;
using CloudT    = pcl::PointCloud<PointT>;
using CloudTPtr = CloudT::Ptr;

// ============================================================
// Pose – poloha a orientace UAV v lokálním NED frame
// ============================================================
struct Pose {
    float x   = 0.0f;   // [m] North (NED)
    float y   = 0.0f;   // [m] East  (NED)
    float z   = 0.0f;   // [m] Down  (NED, kladné = níže)
    float yaw = 0.0f;   // [rad] heading (0 = North, kladný = CW)

    // Převod na transformační matici 4×4 (rotace pouze yaw)
    Eigen::Matrix4f toMatrix() const;

    // Vytvoření Pose z transformační matice 4×4
    static Pose fromMatrix(const Eigen::Matrix4f& mat);
};

// ============================================================
// SlamOdometry – vláknově bezpečná ICP odometrie
//
// Princip:
//   1. Příchozí mračno bodů → VoxelGrid downsample
//   2. ICP: aktuální cloud vs. keyframe cloud
//   3. Akumulace transformací → aktualizace polohy
//   4. Při dostatečném pohybu → aktualizace keyframe
//
// Omezení:
//   - Bez uzavření smyčky (loop closure) – vhodné pro krátké mise
//   - Maximálně 30 iterací ICP na jeden snímek
//   - ICP s fitness skóre > threshold je odmítnuto
// ============================================================
class SlamOdometry {
public:
    // Konfigurace (z config.yaml, sekce slam)
    struct Config {
        float voxel_leaf_size             = 0.15f;
        int   icp_max_iterations          = 30;
        float icp_max_correspondence_dist = 0.5f;
        float icp_transformation_eps      = 1e-6f;
        float icp_euclidean_fitness_eps   = 0.01f;
        float icp_fitness_threshold       = 0.25f;
        float keyframe_min_dist_m         = 0.3f;
        float keyframe_min_angle_rad      = 0.087f; // ≈ 5°
        Pose  initial_pose;
    };

    explicit SlamOdometry(const YAML::Node& cfg);

    // Zpracuje nový cloud a aktualizuje odhad polohy
    // Vrátí true při úspěšné konvergenci ICP
    bool update(const CloudTPtr& filtered_cloud);

    // Vrátí aktuální odhadovanou polohu (vláknově bezpečné)
    Pose getCurrentPose() const;

    // Vrátí čas poslední úspěšné aktualizace
    std::chrono::steady_clock::time_point getLastUpdateTime() const;

    // Resetuje odometrii na zadanou polohu
    void reset(const Pose& pose = Pose{});

    // Vrátí true pokud proběhla alespoň jedna úspěšná aktualizace
    bool isInitialized() const { return initialized_.load(); }

    // Vrátí fitness skóre posledního ICP výpočtu (pro diagnostiku)
    float getLastFitnessScore() const { return last_fitness_; }

private:
    Config cfg_;

    Pose                                          current_pose_;
    Eigen::Matrix4f                               accumulated_transform_;
    CloudTPtr                                     keyframe_cloud_;
    std::atomic<bool>                             initialized_{false};
    float                                         last_fitness_{0.0f};
    std::chrono::steady_clock::time_point         last_update_time_;

    mutable std::mutex                            pose_mutex_;

    pcl::IterativeClosestPoint<PointT, PointT>    icp_;
    pcl::VoxelGrid<PointT>                        voxel_;

    // Vrátí true pokud je pohyb od posledního keyframe dostatečný
    bool shouldUpdateKeyframe(const Eigen::Matrix4f& delta);

    // Downsample mračna bodů pomocí VoxelGrid filtru
    CloudTPtr downsample(const CloudTPtr& cloud);

    static void log(const std::string& msg);
};

} // namespace stc
