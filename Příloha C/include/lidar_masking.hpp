/*
 * DIPLOMOVÁ PRÁCE: Návrh systému pro autonomně bezkolizní řízení UAV
 * Příloha C
 * Autor: rtm. Richard Línek
 * Fyzická platforma (sim-to-real přenos)
 *
 * Soubor obsahuje rozhraní filtrace mračna bodů (LidarMasking):
 *   - CropBox maska – odstranění bodů odpovídajících konstrukci UAV
 *   - VoxelGrid downsample – redukce hustoty mračna pro ICP odometrii
 *   - PassThrough výškový filtr – omezení na relevantní výškové pásmo
 */

#pragma once

#include <string>
#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>

namespace stc {

using PointT         = pcl::PointXYZI;
using CloudT         = pcl::PointCloud<PointT>;
using CloudTPtr      = CloudT::Ptr;
using CloudTConstPtr = CloudT::ConstPtr;

// ============================================================
// LidarMasking – aplikace PCL filtrů na příchozí mračno bodů
// ============================================================
class LidarMasking {
public:
    // Konfigurace (z config.yaml, sekce lidar_mask a height_filter)
    struct Config {
        // CropBox maska UAV
        bool   mask_enabled  = true;
        float  mask_min_x    = -0.38f;
        float  mask_max_x    =  0.38f;
        float  mask_min_y    = -0.38f;
        float  mask_max_y    =  0.38f;
        float  mask_min_z    = -0.25f;
        float  mask_max_z    =  0.12f;

        // Výškový filtr
        bool   height_enabled = true;
        float  height_min_z   = -0.3f;
        float  height_max_z   =  4.0f;

        // VoxelGrid downsample
        float  voxel_leaf_size = 0.15f;
    };

    explicit LidarMasking(const YAML::Node& cfg);
    LidarMasking() = default;

    // Aplikuje kompletní pipeline filtrů (maska → výška → downsample)
    CloudTPtr process(const CloudTConstPtr& raw_cloud);

    // Aplikuje pouze masku UAV bez downsamplování (pro diagnostiku)
    CloudTPtr applyDroneMask(const CloudTConstPtr& cloud);

    const Config& getConfig() const { return cfg_; }

    // Aktualizuje konfiguraci filtrů za běhu
    void updateConfig(const Config& new_cfg);

private:
    Config cfg_;

    mutable pcl::CropBox<PointT>      crop_box_;
    mutable pcl::VoxelGrid<PointT>    voxel_grid_;
    mutable pcl::PassThrough<PointT>  pass_through_z_;

    void initFilters();
    static void log(const std::string& msg);
};

} // namespace stc
