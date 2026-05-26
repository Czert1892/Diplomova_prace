DIPLOMOVÁ PRÁCE: Návrh systému pro autonomně bezkolizní řízení UAV
Autor: rtm. Richard Línek
Univerzita obrany, FVT, K-211 Katedra vojenské robotiky, 2026

========================================================================
PŘEHLED PŘÍLOH
========================================================================

Práce obsahuje tři softwarové přílohy dokumentující postupný vývoj
navigačního systému UAV — od simulovaného 2D prototypu přes 3D rozšíření
až po implementaci na fyzické platformě.

------------------------------------------------------------------------
PŘÍLOHA A — 2D řešení (simulátor Webots)
------------------------------------------------------------------------
Jeden soubor C++ pro simulátor Webots (DJI Mavic 2).

Architektura (2 vrstvy):
  MavicAutopilot  — nízkoúrovňová stabilizace polohy, výšky a orientace
                    čtyřrotorového UAV pomocí PID regulátorů.
  Navigator       — 2D dynamické mapování okolí (mřížka obsazenosti 200×200
                    buněk, rozlišení 0,5 m) z dat LiDARu a plánování trasy
                    algoritmem Theta* (Any-Angle Path Planning) s Bresenhamovou
                    kontrolou přímé viditelnosti.

Klíčové vlastnosti:
  - Mapa pevné velikosti (100×100 m, fixovaná na origin)
  - Sledování trasy regulátorem Pure Pursuit s adaptivní rychlostí
  - Zapomínání starých detekcí (postupná degradace buněk)

------------------------------------------------------------------------
PŘÍLOHA B — 3D řešení (simulátor Webots)
------------------------------------------------------------------------
Jeden soubor C++ pro simulátor Webots (DJI Mavic 2).

Architektura (3 vrstvy):
  MavicAutopilot  — stejná stabilizační vrstva jako v Příloze A.
  Navigator3D     — 3D dynamické mapování (mřížka obsazenosti s výškovými
                    vrstvami, rozlišení 0,5 m) a plánování trasy Theta* 3D
                    se 26-směrným okolím a 3D kontrolou viditelnosti.
  Hlavní smyčka   — řízení mise, čtení senzorů, volání mapování a navigace.

Klíčové vlastnosti oproti Příloze A:
  - Dynamická velikost mapy přizpůsobená vzdálenosti k waypointu
  - Podpora více sekvenčních waypointů (absolutní nebo relativní GPS)
  - Stavový automat mise: TAKEOFF → NAVIGATING → LANDING → LANDED
  - PD regulátor výšky s tlumením vertikálního přestřelení
  - Dvoufázová přistávací sekvence (stabilizace XY + řízené klesání)
  - Asymetrická inflace překážek v ose Z (detekce horizontálních ploch)
  - Dopředné skenování trasy pro přeskakování mezibodů (forward path scan)

------------------------------------------------------------------------
PŘÍLOHA C — Fyzická platforma (sim-to-real přenos)
------------------------------------------------------------------------
Projekt STC Drone Nav — C++ aplikace pro reálné UAV.

Hardware:
  Jetson Nano (JetPack 4.6 / Ubuntu 18.04) + Velodyne VLP-16 + Pixhawk (PX4)

Architektura (5 modulů + konfigurace):
  main.cpp             — vstupní bod; inicializace, řízení mise, správa vláken.
  mavlink_bridge       — komunikace s Pixhawkem přes MAVSDK (armování, vzlet,
                         Offboard navigace, přistání, failsafe).
  slam_odometry        — frame-to-keyframe ICP odometrie z LiDARových dat
                         (PCL::IterativeClosestPoint, keyframe strategie).
  lidar_masking        — filtrace mračna bodů: CropBox maska UAV, PassThrough
                         výškový filtr, VoxelGrid downsample.
  watchdog             — bezpečnostní monitorování (LOITER při výpadku dat,
                         eskalace na LAND po timeoutu).
  tf_utils             — transformace souřadnic LiDAR → BODY → NED.
  config/config.yaml   — centrální konfigurace všech parametrů systému.

Klíčový rozdíl oproti Přílohám A a B:
  Simulační vrstvy (Webots API, pevná mapa, přímý přístup k senzorům)
  jsou nahrazeny reálnými hardwarovými rozhraními (MAVSDK, PCL HDL Grabber,
  UART). Theta* plánovač je připraven k integraci jako samostatný modul.

========================================================================
VZTAH MEZI PŘÍLOHAMI
========================================================================

  Příloha A  →  ověření algoritmu Theta* v 2D simulaci
  Příloha B  →  rozšíření na 3D prostor a vícebodovou misi v simulaci
  Příloha C  →  přenos ověřené architektury na fyzickou platformu (Jetson Nano)

Stabilizační vrstva MavicAutopilot (Přílohy A a B) odpovídá funkci modulu
mavlink_bridge v Příloze C — obě vrstvy přijímají rychlostní příkazy
a zajišťují nízkoúrovňové řízení UAV.
