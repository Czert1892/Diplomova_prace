/*
 * DIPLOMOVÁ PRÁCE: Návrh systému pro autonomně bezkolizní řízení UAV
 * Příloha B
 * Autor: rtm. Richard Línek
 * 3D řešení
 */

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Lidar.hpp>

#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>
#include <queue>
#include <map>
#include <tuple>
#include <cstdint>

using namespace webots;
using namespace std;

// ============================================================
//  Základní datové typy
// ============================================================
struct Point3D { double x, y, z; };

struct Node3D {
    int r, c, l;
    bool operator==(const Node3D& o) const { return r==o.r && c==o.c && l==o.l; }
    bool operator!=(const Node3D& o) const { return !(*this==o); }
    bool operator<(const Node3D& o) const {
        if (r != o.r) return r < o.r;
        if (c != o.c) return c < o.c;
        return l < o.l;
    }
};

// Euklidovská vzdálenost mezi dvěma uzly v 3D mřížce
static double dist3d(Node3D a, Node3D b) {
    return sqrt(pow(a.r-b.r,2) + pow(a.c-b.c,2) + pow(a.l-b.l,2));
}

// Směrový vektor pro expanzi sousedů v Theta* prohledávání
struct Dir3D { int dr, dc, dl; };

// ============================================================
//  MISE — Absolutní GPS souřadnice waypointů
//
//  Zadávají se GPS souřadnice (X, Y, Z) — NE souřadnice ze Scene Tree.
//
//  GPS souřadnice se v daném světě liší od Scene Tree o konstantní offset,
//  který lze zjistit umístěním UAV do známé pozice a přečtením GPS hodnoty
//  z prvního řádku logu ("[INIT] Startovní GPS: [X, Y]").
//
//  Z = letová výška pro navigaci (doporučeno 5 m pro 3D vyhýbání překážkám).
//    Po dosažení posledního waypointu se aktivuje klesání a přistání.
//
//  USE_RELATIVE_WAYPOINTS = false → absolutní GPS souřadnice
//  USE_RELATIVE_WAYPOINTS = true  → souřadnice relativní vůči startovní pozici
// ============================================================
static const bool USE_RELATIVE_WAYPOINTS = false;

static const vector<Point3D> MISSION_WAYPOINTS = {
    //      X         Y      Z   — absolutní GPS souřadnice
    { -213.0,  -96.0,  5.0 },   // WP 1: cíl mise
};

// ============================================================
//  1. VRSTVA: MAVIC AUTOPILOT (stabilizace letu)
// ============================================================
// Realizuje nízkoúrovňové řízení čtyř rotorů na základě požadovaných rychlostí
// v tělovém souřadnicovém systému (vx, vy, vyaw) a požadované výšky (target_z).
// Stejná implementace jako v ThetaCpp (2D verze) — autopilot je v této práci
// uvažován jako "black box" stabilizační vrstva.
class MavicAutopilot {
    Motor *motors[4];
    const double K_VERTICAL_THRUST = 68.5, K_VERTICAL_OFFSET = 0.6, K_VERTICAL_P = 3.0;
    const double K_ROLL_P = 50.0, K_PITCH_P = 30.0, MAX_TILT = 0.2;
    double cmd_pitch = 0, cmd_roll = 0;

public:
    MavicAutopilot(Robot *r) {
        const char* names[4] = {
            "front left propeller", "front right propeller",
            "rear left propeller",  "rear right propeller"
        };
        for (int i = 0; i < 4; i++) {
            motors[i] = r->getMotor(names[i]);
            motors[i]->setPosition(INFINITY);
            motors[i]->setVelocity(1.0);
        }
    }

    // Hlavní řídicí krok — přepočet požadovaných rychlostí na rychlosti motorů
    void update(double z, double roll, double pitch, double yaw,
                double vx, double vy, double target_z, double vyaw,
                const double *gyro) {
        // Mapování požadované rychlosti na cílový náklon (saturace v rozsahu ±MAX_TILT)
        double t_pitch =  max(-1.0, min(vx / 5.0, 1.0)) * MAX_TILT;
        double t_roll  = -(max(-1.0, min(vy / 5.0, 1.0))) * MAX_TILT;

        // Filtr typu low-pass pro plynulou změnu požadovaného náklonu
        cmd_pitch = 0.9 * cmd_pitch + 0.1 * t_pitch;
        cmd_roll  = 0.9 * cmd_roll  + 0.1 * t_roll;

        // PID složky pro náklony a výšku
        double r_in    = K_ROLL_P  * (roll  - cmd_roll)  + gyro[0];
        double p_in    = K_PITCH_P * (pitch - cmd_pitch) + gyro[1];
        double alt_err = max(-1.0, min(target_z - z + K_VERTICAL_OFFSET, 1.0));
        double v_in    = K_VERTICAL_P * pow(alt_err, 3.0);
        double t       = (K_VERTICAL_THRUST + v_in) / max(0.9, cos(pitch) * cos(roll));

        // Distribuce tahu na jednotlivé motory (kříž 4-rotorové konfigurace)
        motors[0]->setVelocity(  t - vyaw + p_in - r_in );
        motors[1]->setVelocity(-(t + vyaw + p_in + r_in));
        motors[2]->setVelocity(-(t + vyaw - p_in - r_in));
        motors[3]->setVelocity(  t - vyaw - p_in + r_in );
    }

    // Vypnutí všech motorů (po dokončení mise)
    void stop() {
        for (int i = 0; i < 4; i++) motors[i]->setVelocity(0.0);
    }
};

// ============================================================
//  2. VRSTVA: NAVIGATOR 3D (Theta* Any-Angle Path Planning ve 3D)
// ============================================================
// Realizuje obě fáze navigační smyčky v trojrozměrném prostoru:
//   Fáze 1 — dynamická aktualizace 3D mřížky obsazenosti z dat LiDARu
//   Fáze 2 — plánování trasy algoritmem Theta* s 26-směrným okolím
//
// Navíc obsahuje stavový automat mise (TAKEOFF/NAVIGATING/LANDING/LANDED)
// a PD regulátor výšky pro plynulou vertikální navigaci.
class Navigator3D {
private:
    // Pevné parametry mapy
    const double GRID_RES     = 0.5;   // Rozlišení buňky [m]
    const int    MAP_DIM_Z    = 50;    // Počet výškových vrstev (rozsah 0–25 m)
    const double Z_ORIGIN     = 0.0;   // Spodní hranice výškové osy
    const double MAP_MARGIN   = 25.0;  // Okraj mapy za UAV a WP [m]
    const int    MIN_MAP_HALF = 60;    // Min. polovina mapy [buňky] = 30 m každou stranu
    const int    MAX_MAP_HALF = 500;   // Max. polovina mapy [buňky] = 250 m každou stranu

    // Dynamické parametry mapy (přepočítány při každé změně waypointu)
    int    map_dim_xy     = 0;
    double map_origin_x   = 0.0;
    double map_origin_y   = 0.0;
    int    map_wp_idx_last = -1;       // Poslední WP, pro který byla mapa přepočítána

    vector<int8_t>  grid3d;            // 3D mřížka obsazenosti, hodnota 0–100
    vector<Point3D> path3d;            // Aktuální naplánovaná trasa
    int             path_idx = -1;     // Index aktuálního waypointu na trase

    // Mise
    vector<Point3D> waypoints;
    int             wp_idx = 0;

    // Stavový automat mise:
    //   TAKEOFF    — UAV stoupá na místě na výšku prvního waypointu
    //   NAVIGATING — horizontální 3D navigace mezi waypointy
    //   LANDING    — dvoufázová přistávací sekvence (stabilizace + klesání)
    //   LANDED     — mise dokončena, motory zastaveny
    enum State { TAKEOFF, NAVIGATING, LANDING, LANDED } state = TAKEOFF;
    double land_target_z   = 0.0;
    double land_x          = 0.0;      // Cílové XY pro přistání
    double land_y          = 0.0;
    int    land_stab_cnt   = 0;        // Počítadlo stabilizační fáze přistání
    bool   land_descending = false;    // false = stabilizace nad cílem, true = klesání

    // PD regulátor výšky:
    //   Plánovač vrací path3d body s 3D souřadnicemi. Autopilotovi se předává
    //   Z-souřadnice aktuálního path bodu jako cílová výška. PD regulátor přidává
    //   tlumení na základě aktuální vertikální rychlosti, čímž zabraňuje přestřelení.
    //
    //   Vnější smyčka: pos_err → desired_vel (saturace ±ALT_VMAX)
    //   Vnitřní smyčka: vel_err → korekce → target_alt
    const double K_ALT_OFFSET = 0.6;   // Musí odpovídat K_VERTICAL_OFFSET v autopilotu
    const double ALT_VMAX     = 0.5;   // Max. vertikální rychlost [m/s]
    const double ALT_KP       = 1.0;   // Zesílení pos_err → desired_vel
    const double ALT_KV_EFF   = 1.6;   // Zesílení vel_err (Kv * dt)
    const double ALT_MAX_STEP = 2.0;   // Max. korekce target_alt za krok [m]
    const double ALT_DT       = 0.032; // Časový krok simulace [s]
    double prev_pz = -1.0;             // Předchozí výška pro výpočet vertikální rychlosti
    double prev_px = -1e9;             // Předchozí X-souřadnice pro výpočet vx
    double prev_py = -1e9;             // Předchozí Y-souřadnice pro výpočet vy

    int nav_debug_steps = 0;           // Počítadlo pro diagnostický výpis úvodních kroků
    int degrade_cnt     = 0;           // Počítadlo pro řídké zapomínání starých detekcí
    int replan_timer    = 0;           // Časovač pro omezení frekvence přeplánování

    // Předpočítané směry pro 26-směrné okolí v Theta* prohledávání
    vector<Dir3D> dirs26;

    // ----------------------------------------------------------
    //  PD regulátor výšky
    //  Vrací cílovou výšku pro autopilota včetně K_ALT_OFFSET korekce
    // ----------------------------------------------------------
    double altTarget(double pz, double vel_z, double desired_alt) const {
        double desired_vel = max(-ALT_VMAX, min((desired_alt - pz) * ALT_KP, ALT_VMAX));
        double vel_err     = desired_vel - vel_z;
        double correction  = max(-ALT_MAX_STEP, min(vel_err * ALT_KV_EFF, ALT_MAX_STEP));
        return pz + correction - K_ALT_OFFSET;
    }

    // ----------------------------------------------------------
    //  Přepočet středu a velikosti mapy
    //  Volá se při startu a při každé změně cílového waypointu.
    //  Mapa se centruje na střed úseku UAV → WP a její velikost se přizpůsobuje
    //  délce úseku, aby pokryla celou potřebnou oblast s rezervou MAP_MARGIN.
    // ----------------------------------------------------------
    void computeMapParams(double px, double py, const Point3D& wp) {
        // Střed mapy = střed úseku UAV → waypoint
        map_origin_x = (px + wp.x) * 0.5;
        map_origin_y = (py + wp.y) * 0.5;

        // Velikost mapy = délka úseku + okraj, minimálně MIN_MAP_HALF
        double half_x = abs(wp.x - px) * 0.5 + MAP_MARGIN;
        double half_y = abs(wp.y - py) * 0.5 + MAP_MARGIN;
        double half   = max(half_x, half_y);

        int new_half = max(MIN_MAP_HALF, (int)ceil(half / GRID_RES));

        // Ochrana před příliš velkou mapou (omezení paměti)
        if (new_half > MAX_MAP_HALF) {
            cout << "[MAP] UPOZORNĚNÍ: WP je " << (half * 2.0) << " m daleko, "
                 << "mapa oříznutá na " << (MAX_MAP_HALF * GRID_RES * 2.0) << " m. "
                 << "Doporučuje se přidat průletový WP uprostřed." << endl;
            new_half = MAX_MAP_HALF;
        }

        int new_dim = new_half * 2;

        // Realokace mřížky a inicializace na nulové hodnoty
        map_dim_xy = new_dim;
        grid3d.assign((size_t)map_dim_xy * map_dim_xy * MAP_DIM_Z, 0);

        double x_min = map_origin_x - new_half * GRID_RES;
        double x_max = map_origin_x + new_half * GRID_RES;
        double y_min = map_origin_y - new_half * GRID_RES;
        double y_max = map_origin_y + new_half * GRID_RES;

        cout << "[MAP] Střed: [" << map_origin_x << ", " << map_origin_y << "]"
             << "  Rozsah X: [" << x_min << ", " << x_max << "]"
             << "  Y: [" << y_min << ", " << y_max << "]"
             << "  Dim: " << map_dim_xy << "x" << map_dim_xy
             << " (" << (grid3d.size() / 1000000.0) << " MB)" << endl;

        // Cesta v původní mapě se stává neplatnou
        path_idx = -1;
    }

    // ----------------------------------------------------------
    //  Transformace souřadnic mezi světovým a mřížkovým systémem
    // ----------------------------------------------------------
    Node3D worldToGrid(double x, double y, double z) const {
        int cx = map_dim_xy / 2;
        int r  = cx - (int)((y - map_origin_y) / GRID_RES);
        int c  = cx + (int)((x - map_origin_x) / GRID_RES);
        int l  = (int)((z - Z_ORIGIN) / GRID_RES);
        return {
            max(0, min(r, map_dim_xy - 1)),
            max(0, min(c, map_dim_xy - 1)),
            max(0, min(l, MAP_DIM_Z  - 1))
        };
    }

    Point3D gridToWorld(Node3D n) const {
        int cx = map_dim_xy / 2;
        return {
            map_origin_x + (double)(n.c - cx) * GRID_RES,
            map_origin_y + (double)(cx - n.r) * GRID_RES,
            Z_ORIGIN + n.l * GRID_RES
        };
    }

    int gridIdx(Node3D n) const {
        return n.l * (map_dim_xy * map_dim_xy) + n.r * map_dim_xy + n.c;
    }

    bool inBounds(Node3D n) const {
        return n.r >= 0 && n.r < map_dim_xy
            && n.c >= 0 && n.c < map_dim_xy
            && n.l >= 0 && n.l < MAP_DIM_Z;
    }

    // ----------------------------------------------------------
    //  3D kontrola přímé viditelnosti (Line of Sight)
    //  Spojnice mezi dvěma uzly je vzorkována po malých krocích a v každém
    //  kroku se kontroluje, zda odpovídající buňka mřížky není obsazena.
    // ----------------------------------------------------------
    bool lineOfSight3D(Node3D s, Node3D e) const {
        int dx    = abs(e.c - s.c);
        int dy    = abs(e.r - s.r);
        int dz    = abs(e.l - s.l);
        int steps = max({dx, dy, dz});
        if (steps == 0) return true;
        for (int i = 0; i <= steps; i++) {
            int x = s.c + (int)round((double)i * (e.c - s.c) / steps);
            int y = s.r + (int)round((double)i * (e.r - s.r) / steps);
            int z = s.l + (int)round((double)i * (e.l - s.l) / steps);
            Node3D cur = {y, x, z};
            if (!inBounds(cur) || grid3d[gridIdx(cur)] > 50) return false;
        }
        return true;
    }

    // ----------------------------------------------------------
    //  Theta* 3D Planner
    //  Rozšíření klasického Theta* z 2D do 3D — používá 26-směrné okolí
    //  a 3D kontrolu viditelnosti pro generování plynulých trajektorií
    //  pod libovolným prostorovým úhlem.
    // ----------------------------------------------------------
    bool runPlanner(Point3D start_p, Point3D goal_p) {
        Node3D start = worldToGrid(start_p.x, start_p.y, start_p.z);
        Node3D goal  = worldToGrid(goal_p.x,  goal_p.y,  goal_p.z);

        // Kontrola obsazenosti cílové buňky
        if (grid3d[gridIdx(goal)] > 50) {
            static int warn_cnt = 0;
            if (++warn_cnt % 50 == 1)
                cout << "[PLAN3D] Cíl blokován, occupancy=" << (int)grid3d[gridIdx(goal)] << endl;
            return false;
        }

        using Element = pair<double, Node3D>;
        priority_queue<Element, vector<Element>, greater<Element>> frontier;
        frontier.push({0.0, start});

        map<Node3D, Node3D>  came_from;
        map<Node3D, double>  g_score;
        came_from[start] = start;
        g_score[start]   = 0.0;

        int expanded = 0;

        while (!frontier.empty()) {
            if (++expanded > 50000) break;

            Node3D cur = frontier.top().second;
            frontier.pop();

            // Cíl dosažen — rekonstrukce cesty zpětným průchodem
            if (cur == goal) {
                path3d.clear();
                Node3D temp = goal;
                while (temp != start) {
                    path3d.push_back(gridToWorld(temp));
                    temp = came_from[temp];
                }
                reverse(path3d.begin(), path3d.end());
                path_idx = 0;
                return true;
            }

            Node3D parent = came_from.count(cur) ? came_from[cur] : cur;

            // Expanze 26 sousedů (3D ekvivalent Mooreova okolí)
            for (const Dir3D& d : dirs26) {
                Node3D nb = {cur.r + d.dr, cur.c + d.dc, cur.l + d.dl};
                if (!inBounds(nb) || grid3d[gridIdx(nb)] > 50) continue;

                // Cena přesunu závisí na typu souseda (přímý / hranou / rohem)
                int nonzero   = (d.dr != 0) + (d.dc != 0) + (d.dl != 0);
                double move_cost = (nonzero == 1) ? 1.0 : (nonzero == 2) ? 1.414 : 1.732;

                double  new_g;
                Node3D  actual_parent;

                // Klíčový rozdíl Theta* oproti A* — kontrola přímé viditelnosti k rodiči
                if (lineOfSight3D(parent, nb)) {
                    new_g         = g_score[parent] + dist3d(parent, nb);
                    actual_parent = parent;
                } else {
                    new_g         = g_score[cur] + move_cost;
                    actual_parent = cur;
                }

                if (!g_score.count(nb) || new_g < g_score[nb]) {
                    g_score[nb]   = new_g;
                    came_from[nb] = actual_parent;
                    frontier.push({new_g + dist3d(nb, goal), nb});
                }
            }
        }
        return false;
    }

public:
    // ----------------------------------------------------------
    //  Konstruktor — inicializace seznamu směrů pro 26-směrné okolí
    // ----------------------------------------------------------
    Navigator3D(const vector<Point3D>& wps) : waypoints(wps) {
        for (int dr = -1; dr <= 1; dr++)
            for (int dc = -1; dc <= 1; dc++)
                for (int dl = -1; dl <= 1; dl++)
                    if (dr || dc || dl)
                        dirs26.push_back({dr, dc, dl});

        cout << "[3D NAV] Mise: " << waypoints.size() << " waypointů" << endl;
        for (int i = 0; i < (int)waypoints.size(); i++)
            cout << "  WP" << i+1 << ": [" << waypoints[i].x << ", "
                 << waypoints[i].y << ", " << waypoints[i].z << "]" << endl;
        cout << "[3D NAV] Mapa se inicializuje při prvním kroku (po získání GPS polohy)." << endl;
    }

    bool isLanded() const { return state == LANDED; }

    // ----------------------------------------------------------
    //  FÁZE 1: 3D Mapování
    //  - postupná degradace hodnot (zapomínání starých detekcí)
    //  - transformace bodů ze senzorového do světového rámce
    //  - filtrace výškou a self-maskem UAV
    //  - asymetrická inflace v ose Z (rozšíření překážek směrem nahoru)
    //  - čištění okolí UAV a cílové oblasti
    // ----------------------------------------------------------
    void updateMap3D(const Lidar *lidar,
                     double px, double py, double pz,
                     double roll, double pitch, double yaw) {
        if (map_dim_xy == 0) return;

        // Řídká degradace (každých 20 kroků) pro stabilnější mapu —
        // hodnota buňky 100 vydrží přibližně 160 sekund místo 16 sekund.
        if (++degrade_cnt % 20 == 0) {
            for (auto &cell : grid3d)
                if (cell > 0) cell = (int8_t)max(0, (int)cell - 1);
        }

        double cr = cos(roll),  sr = sin(roll);
        double cp = cos(pitch), sp = sin(pitch);
        double cy = cos(yaw),   sy = sin(yaw);

        const WbLidarPoint *pts = lidar->getPointCloud();
        int n_pts = lidar->getNumberOfPoints();

        const Point3D &cur_wp = waypoints[min(wp_idx, (int)waypoints.size() - 1)];

        for (int i = 0; i < n_pts; i += 5) {
            if (!isfinite(pts[i].x)) continue;

            // Transformace bodu z tělového do světového rámce pomocí rotační matice
            double lx = pts[i].x, ly = pts[i].y, lz = pts[i].z;
            double wx = (cy*cp)*lx + (cy*sp*sr - sy*cr)*ly + (cy*sp*cr + sy*sr)*lz;
            double wy = (sy*cp)*lx + (sy*sp*sr + cy*cr)*ly + (sy*sp*cr - cy*sr)*lz;
            double wz = (-sp)*lx   + (cp*sr)*ly            + (cp*cr)*lz;

            // Výškový filtr — ignoruj body nad/pod letovou vrstvou
            double abs_z = pz + wz;
            if (abs_z < 0.2 || abs_z > 24.0) continue;

            // Self-mask UAV — ignoruj body příliš blízko středu (vlastní tělo)
            if (wx*wx + wy*wy < 1.5*1.5 && wz*wz < 1.0*1.0) continue;

            double world_x = px + wx;
            double world_y = py + wy;

            // Ochrana cílové oblasti před zápisem překážky
            double d_goal = sqrt(pow(world_x - cur_wp.x, 2)
                               + pow(world_y - cur_wp.y, 2)
                               + pow(abs_z   - cur_wp.z, 2));
            if (d_goal < 2.0) continue;

            // Inflace překážky — asymetrická v ose Z (dl jde do +2 místo +1)
            // pro spolehlivou detekci horizontálních ploch (např. střechy budov)
            Node3D cell = worldToGrid(world_x, world_y, abs_z);
            for (int dr = -1; dr <= 1; dr++)
                for (int dc = -1; dc <= 1; dc++)
                    for (int dl = -1; dl <= 2; dl++) {
                        Node3D nb = {cell.r+dr, cell.c+dc, cell.l+dl};
                        if (inBounds(nb))
                            grid3d[gridIdx(nb)] = (int8_t)min(100, (int)grid3d[gridIdx(nb)] + 10);
                    }
        }

        // Čištění okolí UAV (5×5×3 buněk) — vlastní tělo nesmí být v mapě
        Node3D dn = worldToGrid(px, py, pz);
        for (int dr = -2; dr <= 2; dr++)
            for (int dc = -2; dc <= 2; dc++)
                for (int dl = -1; dl <= 1; dl++) {
                    Node3D nb = {dn.r+dr, dn.c+dc, dn.l+dl};
                    if (inBounds(nb)) grid3d[gridIdx(nb)] = 0;
                }

        // Čištění cílové oblasti (poloměr 1,5 m) — cíl musí být vždy dosažitelný
        Node3D gn = worldToGrid(cur_wp.x, cur_wp.y, cur_wp.z);
        for (int dr = -3; dr <= 3; dr++)
            for (int dc = -3; dc <= 3; dc++)
                for (int dl = -3; dl <= 3; dl++) {
                    Node3D nb = {gn.r+dr, gn.c+dc, gn.l+dl};
                    if (inBounds(nb)) {
                        double d = sqrt(pow(dr*GRID_RES,2)+pow(dc*GRID_RES,2)+pow(dl*GRID_RES,2));
                        if (d <= 1.5) grid3d[gridIdx(nb)] = 0;
                    }
                }
    }

    // ----------------------------------------------------------
    //  FÁZE 2 + Řízení: Výpočet rychlostí pro autopilota
    //  Realizuje stavový automat mise a převádí naplánovanou trasu
    //  na rychlostní příkazy pro stabilizační vrstvu.
    // ----------------------------------------------------------
    void getVelocity3D(double px, double py, double pz, double yaw, double gyro_z,
                       double &vx, double &vy, double &vyaw, double &target_alt) {

        // Odhad rychlostí ze změny GPS polohy pro PD regulátory
        double vel_z       = (prev_pz > -1.0)  ? (pz - prev_pz) / ALT_DT : 0.0;
        double vel_x_world = (prev_px > -1e8) ? (px - prev_px) / ALT_DT : 0.0;
        double vel_y_world = (prev_py > -1e8) ? (py - prev_py) / ALT_DT : 0.0;
        prev_pz = pz;
        prev_px = px;
        prev_py = py;

        // ===== PŘISTÁVACÍ SEKVENCE (dvoufázová) =====
        // Fáze 1 (STABILIZACE) — PD regulátor XY při konstantní výšce.
        //   Přechod do fáze 2 nastane po splnění: dist_xy < 1,5 m a vel_xy < 0,4 m/s,
        //   nebo po timeoutu 300 kroků (~9,6 s).
        // Fáze 2 (KLESÁNÍ) — PD regulátor XY + plynulé snižování cílové výšky k zemi.
        if (state == LANDING) {
            double dx      = land_x - px;
            double dy      = land_y - py;
            double dist_xy = hypot(dx, dy);
            double vel_xy  = hypot(vel_x_world, vel_y_world);

            if (!land_descending) {
                // Fáze 1: STABILIZACE — silné PD XY při konstantní výšce
                // Tlumicí poměr ζ = KD/(2·√KP) ≈ 1,77 → přetlumeno, bez oscilací
                land_stab_cnt++;
                const double STAB_KP = 2.0;
                const double STAB_KD = 5.0;
                double vx_w = STAB_KP * dx - STAB_KD * vel_x_world;
                double vy_w = STAB_KP * dy - STAB_KD * vel_y_world;
                vx   = max(-4.0, min( cos(yaw)*vx_w + sin(yaw)*vy_w, 4.0));
                vy   = max(-4.0, min(-sin(yaw)*vx_w + cos(yaw)*vy_w, 4.0));
                vyaw = 0.0;
                target_alt = altTarget(pz, vel_z, pz);

                static int stab_log = 0;
                if (++stab_log % 20 == 0)
                    cout << "[LANDING/STAB] cnt=" << land_stab_cnt
                         << " dist_xy=" << dist_xy
                         << " vel_xy=" << vel_xy
                         << " pz=" << pz << endl;

                if ((dist_xy < 1.5 && vel_xy < 0.4) || land_stab_cnt >= 300) {
                    land_descending = true;
                    cout << "[LANDING] → klesání | dist_xy=" << dist_xy
                         << " vel_xy=" << vel_xy
                         << " (cnt=" << land_stab_cnt << ")" << endl;
                }
                return;
            }

            // Fáze 2: KLESÁNÍ s PD korekcí XY pozice
            const double LAND_KP = 1.5;
            const double LAND_KD = 3.0;
            double vx_w = LAND_KP * dx - LAND_KD * vel_x_world;
            double vy_w = LAND_KP * dy - LAND_KD * vel_y_world;
            vx   = max(-2.0, min( cos(yaw)*vx_w + sin(yaw)*vy_w, 2.0));
            vy   = max(-2.0, min(-sin(yaw)*vx_w + cos(yaw)*vy_w, 2.0));
            vyaw = 0.0;
            target_alt = altTarget(pz, vel_z, 0.0);

            static int desc_log = 0;
            if (++desc_log % 20 == 0)
                cout << "[LANDING/DESC] dist_xy=" << dist_xy
                     << " vel_xy=" << vel_xy
                     << " pz=" << pz << " m" << endl;

            if (pz < 0.25) {
                state = LANDED;
                cout << "[LANDED] Přistání dokončeno! Výška: " << pz << " m" << endl;
            }
            return;
        }

        if (state == LANDED) {
            vx = vy = vyaw = 0.0;
            target_alt = 0.0;
            return;
        }

        // ===== UKONČENÍ MISE =====
        if (wp_idx >= (int)waypoints.size()) {
            state         = LANDING;
            land_target_z = pz;
            land_x        = waypoints.back().x;
            land_y        = waypoints.back().y;
            target_alt    = pz;
            vx = vy = vyaw = 0.0;
            cout << "[MISSION COMPLETE] Všechny WP dosaženy → klesání z " << pz << " m"
                 << " na XY=[" << land_x << "," << land_y << "]" << endl;
            return;
        }

        const Point3D &wp = waypoints[wp_idx];

        // Inicializace nebo přepočet mapy při změně waypointu
        if (map_dim_xy == 0 || wp_idx != map_wp_idx_last) {
            computeMapParams(px, py, wp);
            map_wp_idx_last = wp_idx;
        }

        // ===== TAKEOFF =====
        // UAV stoupá svisle na výšku prvního waypointu, horizontální pohyb je zastaven.
        // Cílová výška se předává přímo (bez PD), aby autopilot získal maximální tah.
        if (state == TAKEOFF) {
            vx = vy = vyaw = 0.0;
            target_alt = wp.z - K_ALT_OFFSET;
            if (pz >= wp.z - 0.8) {
                state = NAVIGATING;
                cout << "[TAKEOFF→NAV] Výška " << pz << " m → zahajuji horizontální navigaci" << endl;
            }
            return;
        }

        // Cílová výška = Z-souřadnice aktuálního path bodu (plánovač rozhoduje o výšce,
        // PD regulátor ji sleduje a tlumí přestřelení)
        { double desired_alt = (path_idx >= 0 && path_idx < (int)path3d.size())
                                ? path3d[path_idx].z
                                : wp.z;
          target_alt = altTarget(pz, vel_z, desired_alt); }

        double dx_g     = wp.x - px, dy_g = wp.y - py, dz_g = wp.z - pz;
        double dist3d_g = sqrt(dx_g*dx_g + dy_g*dy_g + dz_g*dz_g);
        double dist2d_g = hypot(dx_g, dy_g);

        // Dosažení waypointu (tolerance 5 m ve 3D)
        if (dist3d_g < 5.0) {
            cout << "[WP " << wp_idx+1 << "/" << waypoints.size()
                 << "] Dosažen! 3D dist=" << dist3d_g << " m" << endl;
            wp_idx++;
            path_idx = -1;
            return;
        }

        // ===== ROZHODNUTÍ O PŘEPLÁNOVÁNÍ =====
        // Kontrola viditelnosti probíhá řídce (každých ~4,8 s), aby UAV dostalo
        // čas na zarovnání a pohyb před další kontrolou.
        bool needs_replan = (path_idx < 0 || path_idx >= (int)path3d.size());
        if (!needs_replan && dist2d_g > 3.0) {
            if (++replan_timer >= 150) {
                replan_timer = 0;
                Node3D dn  = worldToGrid(px, py, pz);
                Node3D wpn = worldToGrid(path3d[path_idx].x,
                                         path3d[path_idx].y,
                                         path3d[path_idx].z);
                if (!lineOfSight3D(dn, wpn)) {
                    needs_replan = true;
                    cout << "[REPLAN] LOS blokován k path[" << path_idx << "] → přeplánování" << endl;
                }
            }
        }

        if (needs_replan) {
            if (!runPlanner({px, py, pz}, wp)) {
                if (path_idx >= 0 && path_idx < (int)path3d.size()) {
                    static int fail_cnt = 0;
                    if (++fail_cnt % 50 == 1)
                        cout << "[PLAN3D] Selhalo, pokračuji po staré cestě" << endl;
                } else {
                    vx = vy = vyaw = 0.0;
                    static int stuck_cnt = 0;
                    if (++stuck_cnt % 50 == 1)
                        cout << "[PLAN3D] STUCK — žádná cesta k WP" << wp_idx+1 << endl;
                    return;
                }
            } else {
                replan_timer = 0;
                cout << "[PLAN3D] Nová cesta: " << path3d.size()
                     << " bodů → WP" << wp_idx+1
                     << " | UAV=[" << px << "," << py << "," << pz << "]" << endl;
                for (int i = 0; i < (int)path3d.size(); i++)
                    cout << "  [" << i << "] "
                         << path3d[i].x << ", " << path3d[i].y << ", " << path3d[i].z << endl;
            }
        }

        if (path_idx < 0 || path_idx >= (int)path3d.size()) {
            vx = vy = vyaw = 0.0;
            return;
        }

        // ===== DOPŘEDNÉ SKENOVÁNÍ CESTY (forward path scan) =====
        // Umožňuje obejít zbytečné mezibody a letět přímo k nejdál viditelnému
        // bodu trasy. Přeskok na poslední bod je omezen úhlovou tolerancí,
        // aby se zabránilo nárazu při náhlé změně směru.
        {
            Node3D dn = worldToGrid(px, py, pz);
            int fwd_end = ((int)path3d.size() > 1 && dist3d_g > 5.5)
                          ? (int)path3d.size() - 2
                          : (int)path3d.size() - 1;
            for (int i = fwd_end; i > path_idx; i--) {
                Node3D pn = worldToGrid(path3d[i].x, path3d[i].y, path3d[i].z);
                if (lineOfSight3D(dn, pn)) {
                    if (i > path_idx) {
                        // Kontrola úhlu pro přeskok na poslední bod
                        if (i == (int)path3d.size() - 1) {
                            double dx_chk = path3d[i].x - px;
                            double dy_chk = path3d[i].y - py;
                            double aim_chk = atan2(dy_chk, dx_chk);
                            double err_chk = aim_chk - yaw;
                            while (err_chk >  M_PI) err_chk -= 2.0 * M_PI;
                            while (err_chk < -M_PI) err_chk += 2.0 * M_PI;
                            if (fabs(err_chk) > M_PI * 0.6) break;
                        }
                        cout << "[FWD] path_idx " << path_idx << " → " << i
                             << " (přeskočeno " << (i - path_idx) << " bodů)" << endl;
                        path_idx = i;
                    }
                    break;
                }
            }
        }

        // ===== PURE PURSUIT — výpočet rychlostí ke sledovanému bodu cesty =====
        // Na posledním úseku se naviguje přímo ke cíli waypointu (ne k bodu cesty),
        // aby UAV doletělo přesně, a ne k poslední buňce mřížky.
        bool direct_wp_nav = (path_idx == (int)path3d.size() - 1 && dist3d_g < 20.0);
        const Point3D *nav_pt = direct_wp_nav ? &wp : &path3d[path_idx];
        if (direct_wp_nav) {
            static int final_log = 0;
            if (++final_log % 15 == 1)
                cout << "[FINAL] Přímá navigace k WP" << wp_idx+1
                     << " dist3D=" << dist3d_g << " m" << endl;
        }

        double dx_wp     = nav_pt->x - px;
        double dy_wp     = nav_pt->y - py;
        double dist2d_wp = hypot(dx_wp, dy_wp);

        // Světový azimut ke sledovanému bodu cesty
        double aim_wp = atan2(dy_wp, dx_wp);

        // Úhlová chyba mezi aktuálním kurzem a požadovaným azimutem
        double angle_err = aim_wp - yaw;
        while (angle_err >  M_PI) angle_err -= 2.0 * M_PI;
        while (angle_err < -M_PI) angle_err += 2.0 * M_PI;

        // Adaptivní rychlost — zpomalení při přiblížení k cíli (zabraňuje přestřelení)
        double speed_factor = (dist3d_g < 12.0) ? max(0.25, dist3d_g / 12.0) : 1.0;
        double speed = max(0.3, min(1.0, dist2d_wp / 3.0)) * speed_factor;

        // Inverzní transformace ze světového azimutu na tělové rychlosti.
        // Alignment faktor — koeficient v rozsahu 0–1, který škáluje horizontální
        // rychlost podle kvality zarovnání UAV k cíli. Při velké úhlové chybě
        // (> 90°) alignment klesá k nule a UAV se nejprve natáčí, čímž se
        // zabraňuje bočnímu driftu a nárazům do překážek během rotace.
        double alignment = max(0.0, cos(angle_err));
        vx   = max(-1.5, min(cos(angle_err) * speed * alignment, 1.5));
        vy   = max(-1.5, min(sin(angle_err) * speed * alignment, 1.5));

        // PD regulátor yaw — proporcionální složka tlumená derivativním členem
        // s gyroskopickou zpětnou vazbou (zabraňuje přestřelení a oscilacím)
        vyaw = max(-0.8, min(angle_err * 0.25 - gyro_z * 0.5, 0.8));

        // Přepnutí na další waypoint trasy při dostatečném přiblížení
        if (dist2d_wp < 3.0 && path_idx < (int)path3d.size() - 1)
            path_idx++;

        // Diagnostický výpis prvních 30 kroků navigace
        if (nav_debug_steps < 30) {
            nav_debug_steps++;
            cout << "[DBG" << nav_debug_steps << "]"
                 << " yaw=" << (yaw * 180.0 / M_PI) << "°"
                 << " aim=" << (aim_wp * 180.0 / M_PI) << "°"
                 << " err=" << (angle_err * 180.0 / M_PI) << "°"
                 << " gz=" << gyro_z
                 << " vx=" << vx << " vy=" << vy << " vyaw=" << vyaw
                 << " | pos=[" << px << "," << py << "," << pz << "]"
                 << " | pt=[" << nav_pt->x << "," << nav_pt->y << "," << nav_pt->z << "]"
                 << " idx=" << path_idx << "/" << path3d.size()
                 << " | alt_tgt=" << target_alt << endl;
        }

        // Periodický diagnostický výpis stavu navigace
        static int diag_cnt = 0;
        if (++diag_cnt % 30 == 0) {
            cout << "[NAV3D] WP" << wp_idx+1 << "/" << waypoints.size()
                 << " | pos=[" << px << "," << py << "," << pz << "]"
                 << " | dist3D=" << dist3d_g
                 << " | path_idx=" << path_idx << "/" << path3d.size()
                 << (direct_wp_nav ? " (DIRECT)" : "")
                 << " | pt=[" << nav_pt->x << "," << nav_pt->y << "," << nav_pt->z << "]"
                 << " | aim=" << (aim_wp * 180.0 / M_PI) << "°"
                 << " yaw=" << (yaw * 180.0 / M_PI) << "°"
                 << " err=" << (angle_err * 180.0 / M_PI) << "°"
                 << " | sf=" << speed_factor
                 << " gz=" << gyro_z
                 << " vx=" << vx << " vy=" << vy << " vyaw=" << vyaw
                 << " | alt_tgt=" << target_alt << endl;
        }
    }
};

// ============================================================
//  3. VRSTVA: HLAVNÍ SMYČKA
// ============================================================
int main() {
    Robot *robot = new Robot();
    int ts = (int)robot->getBasicTimeStep();

    // Inicializace senzorů
    GPS          *gps   = robot->getGPS("gps");
    InertialUnit *imu   = robot->getInertialUnit("inertial unit");
    Gyro         *gyro  = robot->getGyro("gyro");
    Lidar        *lidar = robot->getLidar("lidar");
    if (!lidar) lidar   = robot->getLidar("Lidar");
    if (!lidar) lidar   = robot->getLidar("Velodyne VLP-16");

    gps->enable(ts);
    imu->enable(ts);
    gyro->enable(ts);

    if (!lidar) {
        cout << "[ERROR] LiDAR nenalezen — zkontrolujte název senzoru ve Scene Tree." << endl;
        delete robot;
        return 1;
    }
    lidar->enable(ts);
    lidar->enablePointCloud();

    MavicAutopilot autopilot(robot);

    // První krok simulace pro inicializaci senzorů (nutná platná GPS poloha)
    if (robot->step(ts) == -1) { delete robot; return 0; }
    const double *pos0 = gps->getValues();
    double start_x = pos0[0], start_y = pos0[1];
    cout << "[INIT] Startovní GPS: [" << start_x << ", " << start_y << "]" << endl;

    // Sestavení absolutních GPS souřadnic waypointů podle nastavení
    vector<Point3D> actual_waypoints;
    if (USE_RELATIVE_WAYPOINTS) {
        for (const auto& wp : MISSION_WAYPOINTS)
            actual_waypoints.push_back({start_x + wp.x, start_y + wp.y, wp.z});
        cout << "[INIT] WP (relativní → GPS):" << endl;
    } else {
        actual_waypoints = MISSION_WAYPOINTS;
        cout << "[INIT] WP (absolutní GPS):" << endl;
    }
    for (int i = 0; i < (int)actual_waypoints.size(); i++)
        cout << "  WP" << i+1 << ": ["
             << actual_waypoints[i].x << ", "
             << actual_waypoints[i].y << ", "
             << actual_waypoints[i].z << "]" << endl;

    Navigator3D navigator(actual_waypoints);

    cout << "=== THETA3D START ===" << endl;

    int    step             = 0;
    double dynamic_target_z = actual_waypoints[0].z;

    // Hlavní řídicí smyčka simulace
    while (robot->step(ts) != -1) {
        step++;

        const double *pos  = gps->getValues();
        const double *rpy  = imu->getRollPitchYaw();
        const double *gyrv = gyro->getValues();

        double px = pos[0], py = pos[1], pz = pos[2];
        double roll = rpy[0], pitch = rpy[1], yaw = rpy[2];

        // Po přistání zastavit motory a setrvat v klidu
        if (navigator.isLanded()) {
            autopilot.stop();
            if (step % 200 == 0)
                cout << "[LANDED] Mise dokončena." << endl;
            continue;
        }

        // Mapování každý 5. krok pro snížení výpočetní zátěže
        if (step % 5 == 0)
            navigator.updateMap3D(lidar, px, py, pz, roll, pitch, yaw);

        // Výpočet rychlostí a předání autopilotovi
        double vx = 0.0, vy = 0.0, vyaw = 0.0;
        navigator.getVelocity3D(px, py, pz, yaw, gyrv[2], vx, vy, vyaw, dynamic_target_z);
        autopilot.update(pz, roll, pitch, yaw, vx, vy, dynamic_target_z, vyaw, gyrv);
    }

    delete robot;
    return 0;
}