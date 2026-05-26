#!/bin/bash
# ============================================================
# DIPLOMOVÁ PRÁCE: Návrh systému pro autonomně bezkolizní řízení UAV
# Příloha C
# Autor: rtm. Richard Línek
# Fyzická platforma (sim-to-real přenos)
#
# Instalace závislostí projektu STC Drone Nav
# Cílová platforma: Jetson Nano, JetPack 4.6 / Ubuntu 18.04 (ARM64)
#
# Spuštění: chmod +x install_deps_jetson.sh && ./install_deps_jetson.sh
# ============================================================

set -e

echo "============================================"
echo " STC Drone Nav – instalace závislostí"
echo " Jetson Nano / Ubuntu 18.04"
echo "============================================"

echo "[1/6] Aktualizace balíčků..."
sudo apt-get update -y

echo "[2/6] Instalace PCL 1.8, Eigen3, yaml-cpp..."
sudo apt-get install -y \
    libpcl-dev \
    libeigen3-dev \
    libyaml-cpp-dev \
    cmake \
    build-essential \
    git \
    libusb-1.0-0-dev \
    libflann-dev \
    libboost-all-dev

echo "[3/6] Instalace MAVSDK..."
MAVSDK_VERSION="1.4.11"
MAVSDK_DEB="mavsdk_${MAVSDK_VERSION}_ubuntu18.04_arm64.deb"
MAVSDK_URL="https://github.com/mavlink/MAVSDK/releases/download/v${MAVSDK_VERSION}/${MAVSDK_DEB}"

if [ ! -f "/tmp/${MAVSDK_DEB}" ]; then
    echo "  Stahuji MAVSDK ${MAVSDK_VERSION}..."
    wget -O "/tmp/${MAVSDK_DEB}" "${MAVSDK_URL}"
fi

sudo dpkg -i "/tmp/${MAVSDK_DEB}" || sudo apt-get install -f -y
echo "  MAVSDK nainstalován."

echo "[4/6] Velodyne PCL Grabber – součást PCL (již nainstalován)."

echo "[5/6] Přidávám uživatele do skupiny dialout (přístup k UART)..."
sudo usermod -a -G dialout $USER
echo "  Odhlaste se a přihlaste pro aktivaci oprávnění."

echo "[6/6] Konfigurace síťového rozhraní pro VLP-16..."
echo "  Přidejte do /etc/network/interfaces:"
echo ""
echo "  auto eth0"
echo "  iface eth0 inet static"
echo "    address 192.168.1.201"
echo "    netmask 255.255.255.0"
echo ""
echo "  Nebo použijte nmcli:"
echo "  nmcli con add type ethernet ifname eth0 ip4 192.168.1.201/24"

echo ""
echo "============================================"
echo " Závislosti nainstalovány."
echo " Postup sestavení:"
echo "   cd STC"
echo "   mkdir build && cd build"
echo "   cmake .. -DCMAKE_BUILD_TYPE=Release"
echo "   make -j4"
echo "   ./stc_drone_nav ../config/config.yaml"
echo "============================================"
