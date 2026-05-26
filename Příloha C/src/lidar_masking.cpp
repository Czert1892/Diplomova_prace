/*
 * DIPLOMOVÁ PRÁCE: Návrh systému pro autonomně bezkolizní řízení UAV
 * Příloha C
 * Autor: rtm. Richard Línek
 * Fyzická platforma (sim-to-real přenos)
 *
 * Soubor obsahuje implementaci filtrace mračna bodů (LidarMasking):
 *   - CropBox v negativním módu odstraní body uvnitř boxu (konstrukce UAV)
 *   - PassThrough výškový filtr (osa Z v LiDAR frame)
 *   - VoxelGrid downsample pro redukci počtu bodů před ICP
 */

#include "lidar_masking.hpp"

#include <iostream>
#include <stdexcept>

namespace stc {

// ============================================================
// Konstruktor – načte konfiguraci z YAML
// ============================================================
LidarMasking::LidarMasking(const YAML::Node& yaml)
{
    try {
        const auto& mask = yaml["lidar_mask"];
        cfg_.mask_enabled = mask["enabled"].as<bool>(true);
        cfg_.mask_min_x   = mask["min_x"].as<float>(-0.38f);
        cfg_.mask_max_x   = mask["max_x"].as<float>( 0.38f);
        cfg_.mask_min_y   = mask["min_y"].as<float>(-0.38f);
        cfg_.mask_max_y   = mask["max_y"].as<float>( 0.38f);
        cfg_.mask_min_z   = mask["min_z"].as<float>(-0.25f);
        cfg_.mask_max_z   = mask["max_z"].as<float>( 0.12f);

        const auto& hf = yaml["height_filter"];
        cfg_.height_enabled = hf["enabled"].as<bool>(true);
        cfg_.height_min_z   = hf["min_z"].as<float>(-0.3f);
        cfg_.height_max_z   = hf["max_z"].as<float>( 4.0f);

        cfg_.voxel_leaf_size = yaml["slam"]["voxel_leaf_size"].as<float>(0.15f);

        log("[LidarMasking] Konfigurace načtena z YAML.");
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(
            std::string("[LidarMasking] Chyba při čtení YAML: ") + e.what());
    }

    initFilters();
}

// ============================================================
// initFilters – nastaví PCL filtry podle aktuální konfigurace
// ============================================================
void LidarMasking::initFilters()
{
    // CropBox v negativním módu odstraní body uvnitř boxu (tělo UAV)
    crop_box_.setNegative(true);
    crop_box_.setMin(Eigen::Vector4f(
        cfg_.mask_min_x, cfg_.mask_min_y, cfg_.mask_min_z, 1.0f));
    crop_box_.setMax(Eigen::Vector4f(
        cfg_.mask_max_x, cfg_.mask_max_y, cfg_.mask_max_z, 1.0f));

    voxel_grid_.setLeafSize(
        cfg_.voxel_leaf_size, cfg_.voxel_leaf_size, cfg_.voxel_leaf_size);

    pass_through_z_.setFilterFieldName("z");
    pass_through_z_.setFilterLimits(cfg_.height_min_z, cfg_.height_max_z);

    log("[LidarMasking] PCL filtry inicializovány:");
    log("  CropBox (maska UAV): X[" + std::to_string(cfg_.mask_min_x) + ", "
        + std::to_string(cfg_.mask_max_x) + "]  Y["
        + std::to_string(cfg_.mask_min_y) + ", "
        + std::to_string(cfg_.mask_max_y) + "]  Z["
        + std::to_string(cfg_.mask_min_z) + ", "
        + std::to_string(cfg_.mask_max_z) + "]");
    log("  VoxelGrid leaf: " + std::to_string(cfg_.voxel_leaf_size) + " m");
    log("  PassThrough Z: [" + std::to_string(cfg_.height_min_z) + ", "
        + std::to_string(cfg_.height_max_z) + "] m");
}

// ============================================================
// process – pipeline filtrů: maska UAV → výška → downsample
// ============================================================
CloudTPtr LidarMasking::process(const CloudTConstPtr& raw_cloud)
{
    if (!raw_cloud || raw_cloud->empty()) {
        return CloudTPtr(new CloudT());
    }

    CloudTPtr masked(new CloudT());
    CloudTPtr filtered(new CloudT());
    CloudTPtr result(new CloudT());

    if (cfg_.mask_enabled) {
        crop_box_.setInputCloud(raw_cloud);
        crop_box_.filter(*masked);
    } else {
        *masked = *raw_cloud;
    }

    if (cfg_.height_enabled) {
        pass_through_z_.setInputCloud(masked);
        pass_through_z_.filter(*filtered);
    } else {
        *filtered = *masked;
    }

    voxel_grid_.setInputCloud(filtered);
    voxel_grid_.filter(*result);

    return result;
}

// ============================================================
// applyDroneMask – pouze CropBox maska (pro diagnostiku)
// ============================================================
CloudTPtr LidarMasking::applyDroneMask(const CloudTConstPtr& cloud)
{
    CloudTPtr result(new CloudT());
    if (!cfg_.mask_enabled) {
        *result = *cloud;
        return result;
    }
    crop_box_.setInputCloud(cloud);
    crop_box_.filter(*result);
    return result;
}

// ============================================================
// updateConfig – aktualizace konfigurace filtrů za běhu
// ============================================================
void LidarMasking::updateConfig(const Config& new_cfg)
{
    cfg_ = new_cfg;
    initFilters();
    log("[LidarMasking] Konfigurace aktualizována.");
}

// ============================================================
void LidarMasking::log(const std::string& msg)
{
    std::cout << msg << std::endl;
}

} // namespace stc
