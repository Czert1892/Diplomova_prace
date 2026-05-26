/*
 * DIPLOMOVÁ PRÁCE: Návrh systému pro autonomně bezkolizní řízení UAV
 * Příloha A
 * Autor: rtm. Richard Línek
 * 2D řešení
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
#include <cstdint>

using namespace webots;
using namespace std;

// Pomocné struktury pro reprezentaci bodu ve světě a uzlu v mřížce
struct Point { double x, y; };
struct Node {
    int r, c;
    bool operator==(const Node& other) const { return r == other.r && c == other.c; }
    bool operator!=(const Node& other) const { return !(*this == other); }
    bool operator<(const Node& other) const {
        return (r < other.r) || (r == other.r && c < other.c);
    }
};

// Euklidovská vzdálenost mezi dvěma uzly v mřížce
double get_dist(Node a, Node b) { return sqrt(pow(a.r - b.r, 2) + pow(a.c - b.c, 2)); }

// Bresenhamova kontrola přímé viditelnosti mezi dvěma uzly (Line of Sight)
// Vrací true, pokud spojnice neprochází žádnou obsazenou buňkou
bool line_of_sight(const vector<int8_t>& grid, int dim, Node s, Node e) {
    int x0 = s.c, y0 = s.r, x1 = e.c, y1 = e.r;
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int x = x0, y = y0;
    int n = 1 + dx + dy;
    int x_inc = (x1 > x0) ? 1 : -1, y_inc = (y1 > y0) ? 1 : -1;
    int error = dx - dy;
    dx *= 2; dy *= 2;

    for (; n > 0; --n) {
        if (grid[y * dim + x] > 50) return false;
        if (x == x1 && y == y1) return true;
        if (error > 0) { x += x_inc; error -= dy; }
        else { y += y_inc; error += dx; }
    }
    return true;
}

// ==============================================================================
// 1. VRSTVA: MAVIC AUTOPILOT (stabilizace letu)
// ==============================================================================
// Realizuje nízkoúrovňové řízení čtyř rotorů na základě požadovaných rychlostí
// v tělovém souřadnicovém systému (vx, vy, vyaw) a požadované výšky (target_z).
// Vstupy pro stabilizaci pocházejí ze senzorů GPS, IMU a gyroskopu.
class MavicAutopilot {
private:
    Motor *motors[4];
    const double K_VERTICAL_THRUST = 68.5, K_VERTICAL_OFFSET = 0.6, K_VERTICAL_P = 3.0;
    const double K_ROLL_P = 50.0, K_PITCH_P = 30.0, MAX_TILT = 0.2;
    double cmd_pitch = 0, cmd_roll = 0;

public:
    MavicAutopilot(Robot *r) {
        string names[4] = {"front left propeller", "front right propeller",
                           "rear left propeller", "rear right propeller"};
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
        double t_pitch =  (max(-1.0, min(vx / 5.0, 1.0))) * MAX_TILT;
        double t_roll  = -(max(-1.0, min(vy / 5.0, 1.0))) * MAX_TILT;

        // Filtr typu low-pass pro plynulou změnu požadovaného náklonu
        cmd_pitch = 0.9 * cmd_pitch + 0.1 * t_pitch;
        cmd_roll  = 0.9 * cmd_roll  + 0.1 * t_roll;

        // PID složky pro náklony a výšku
        double r_in    = K_ROLL_P  * (roll - cmd_roll)  + gyro[0];
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
};

// ==============================================================================
// 2. VRSTVA: NAVIGATOR (Theta* Any-Angle Path Planning)
// ==============================================================================
// Realizuje obě fáze navigační smyčky:
//   Fáze 1 — dynamická aktualizace mřížky obsazenosti z dat LiDARu
//   Fáze 2 — plánování trasy algoritmem Theta* k cílovému waypointu
class Navigator {
private:
    const double GRID_RES = 0.5;   // Rozlišení mřížky [m/buňka]
    const int MAP_DIM = 200;       // Počet buněk v každé ose (mapa pokrývá 100×100 m)
    vector<int8_t> grid;           // Mřížka obsazenosti, hodnota 0–100
    vector<Point> path;            // Aktuální naplánovaná trasa (sekvence bodů)
    int path_idx = -1;             // Index aktuálního waypointu na trase
    Point goal_point = {0, 0};     // Globální cíl mise (ochrana před zápisem překážky)
    int planning_failures = 0;

    // Transformace světových souřadnic [m] na index v mřížce
    Node worldToGrid(double x, double y) {
        int center = MAP_DIM / 2;
        int r = center - (int)(y / GRID_RES);
        int c = center + (int)(x / GRID_RES);
        return {max(0, min(r, MAP_DIM-1)), max(0, min(c, MAP_DIM-1))};
    }

    // Zpětná transformace indexu mřížky na světové souřadnice
    Point gridToWorld(Node n) {
        int center = MAP_DIM / 2;
        return {(double)(n.c - center) * GRID_RES, (double)(center - n.r) * GRID_RES};
    }

public:
    Navigator() { grid.resize(MAP_DIM * MAP_DIM, 0); }

    // Nastavení globálního cíle (využívá se pro ochranu cílové oblasti při mapování)
    void setGoal(double x, double y) {
        goal_point = {x, y};
    }

    // Diagnostická funkce pro zjištění hodnoty obsazenosti v dané pozici
    int getGridValue(double x, double y) {
        Node n = worldToGrid(x, y);
        return grid[n.r * MAP_DIM + n.c];
    }

    // FÁZE 1: Aktualizace mřížky obsazenosti z LiDARového mračna bodů
    // - postupná degradace hodnot (zapomínání starých detekcí)
    // - transformace bodů ze senzorového do světového rámce
    // - filtrace výškou a self-maskem UAV
    // - inflace detekovaných překážek o ochrannou zónu
    // - čištění okolí UAV a cílové oblasti
    void updateMap(const Lidar *lidar, double px, double py, double pz,
                   double roll, double pitch, double yaw) {
        // Postupná degradace hodnot buněk pro zapomínání starých detekcí
        for(auto &cell : grid) if(cell > 0) cell = (int8_t)max(0, (int)cell - 1);

        const WbLidarPoint *pts = lidar->getPointCloud();
        double cr = cos(roll), sr = sin(roll);
        double cp = cos(pitch), sp = sin(pitch);
        double cy = cos(yaw),   sy = sin(yaw);

        for (int i = 0; i < lidar->getNumberOfPoints(); i += 5) {
            if (!isfinite(pts[i].x)) continue;

            // Transformace bodu z tělového do světového rámce pomocí rotační matice
            double wx = (cy*cp)*pts[i].x + (cy*sp*sr - sy*cr)*pts[i].y + (cy*sp*cr + sy*sr)*pts[i].z;
            double wy = (sy*cp)*pts[i].x + (sy*sp*sr + cy*cr)*pts[i].y + (sy*sp*cr - cy*sr)*pts[i].z;
            double wz = (-sp)*pts[i].x   + (cp*sr)*pts[i].y            + (cp*cr)*pts[i].z;

            // Výškový filtr — ignoruj body nad/pod relevantní vrstvou
            if (pz + wz < 0.2 || pz + wz > 2.5) continue;

            // Self-mask UAV — ignoruj body příliš blízko středu (vlastní tělo)
            if (wx*wx + wy*wy < 1.4) continue;

            double world_x = px + wx;
            double world_y = py + wy;

            // Ochrana cílové buňky před zápisem překážky
            double point_to_goal = hypot(world_x - goal_point.x, world_y - goal_point.y);
            if (point_to_goal < GRID_RES) continue;

            // Inflace detekované překážky — zápis i do 8 okolních buněk
            Node cell = worldToGrid(world_x, world_y);
            for(int dr = -1; dr <= 1; dr++) {
                for(int dc = -1; dc <= 1; dc++) {
                    int nr = cell.r + dr, nc = cell.c + dc;
                    if(nr >= 0 && nr < MAP_DIM && nc >= 0 && nc < MAP_DIM) {
                        grid[nr*MAP_DIM + nc] = (int8_t)min(100, (int)grid[nr*MAP_DIM + nc] + 10);
                    }
                }
            }
        }

        // Čištění okolí UAV (5×5 buněk) — vlastní tělo nesmí být v mapě
        Node d = worldToGrid(px, py);
        for(int r = d.r-2; r <= d.r+2; r++)
            for(int c = d.c-2; c <= d.c+2; c++)
                if(r >= 0 && r < MAP_DIM && c >= 0 && c < MAP_DIM)
                    grid[r*MAP_DIM + c] = 0;

        // Čištění cílové oblasti (poloměr 1,5 m) — cíl musí být vždy dosažitelný
        Node goal_node = worldToGrid(goal_point.x, goal_point.y);
        for(int r = goal_node.r-3; r <= goal_node.r+3; r++) {
            for(int c = goal_node.c-3; c <= goal_node.c+3; c++) {
                if(r >= 0 && r < MAP_DIM && c >= 0 && c < MAP_DIM) {
                    double dist = hypot((r - goal_node.r) * GRID_RES,
                                        (c - goal_node.c) * GRID_RES);
                    if(dist <= 1.5) grid[r*MAP_DIM + c] = 0;
                }
            }
        }
    }

    // FÁZE 2: Plánování trasy algoritmem Theta*
    // Kombinuje klasické A* prohledávání s kontrolou viditelnosti (any-angle)
    bool planThetaStar(Point start_p, Point goal_p) {
        goal_point = goal_p;
        Node start = worldToGrid(start_p.x, start_p.y);
        Node goal  = worldToGrid(goal_p.x, goal_p.y);

        // Kontrola obsazenosti cílové buňky
        int goal_occ = grid[goal.r * MAP_DIM + goal.c];
        if (goal_occ > 50) {
            static int block_warn = 0;
            if(++block_warn % 100 == 1) {
                cout << "[THETA*] Cíl blokován! Grid[" << goal.r << "," << goal.c
                     << "] occupancy=" << goal_occ << endl;
            }
            return false;
        }

        typedef pair<double, Node> Element;
        priority_queue<Element, vector<Element>, greater<Element>> frontier;
        frontier.push({0, start});

        map<Node, Node> came_from;
        map<Node, double> g_score;
        came_from[start] = start;
        g_score[start]   = 0;

        int expanded = 0;
        int dr[] = {0, 0, 1, -1, 1,  1, -1, -1};
        int dc[] = {1,-1, 0,  0, 1, -1,  1, -1};

        while (!frontier.empty()) {
            if (++expanded > 2000) break;
            Node current = frontier.top().second; frontier.pop();

            // Cíl dosažen — rekonstrukce cesty zpětným průchodem
            if (current == goal) {
                path.clear();
                Node temp = goal;
                while (temp != start) {
                    path.push_back(gridToWorld(temp));
                    temp = came_from[temp];
                }
                reverse(path.begin(), path.end());
                path_idx = 0;
                return true;
            }

            // Expanze 8 sousedů v Mooreově okolí
            for (int i = 0; i < 8; i++) {
                Node neighbor = {current.r + dr[i], current.c + dc[i]};
                if (neighbor.r < 0 || neighbor.r >= MAP_DIM ||
                    neighbor.c < 0 || neighbor.c >= MAP_DIM) continue;
                if (grid[neighbor.r * MAP_DIM + neighbor.c] > 50) continue;

                Node parent = came_from[current];
                double new_g;
                Node actual_parent;

                // Klíčový rozdíl Theta* oproti A* — kontrola přímé viditelnosti k rodiči
                if (line_of_sight(grid, MAP_DIM, parent, neighbor)) {
                    new_g = g_score[parent] + get_dist(parent, neighbor);
                    actual_parent = parent;
                } else {
                    new_g = g_score[current] + (i < 4 ? 1.0 : 1.41);
                    actual_parent = current;
                }

                if (g_score.find(neighbor) == g_score.end() || new_g < g_score[neighbor]) {
                    g_score[neighbor]   = new_g;
                    came_from[neighbor] = actual_parent;
                    frontier.push({new_g + get_dist(neighbor, goal), neighbor});
                }
            }
        }
        return false;
    }

    // Výpočet požadovaných rychlostí pro autopilota podle aktuálního waypointu
    // Realizuje sledování trasy regulátorem typu Pure Pursuit
    void getVelocity(double px, double py, double yaw,
                     double tx, double ty,
                     double &vx, double &vy, double &vyaw) {
        // Kontrola dosažení globálního cíle
        double dist_to_goal = hypot(tx - px, ty - py);
        if (dist_to_goal < 0.5) {
            vx = 0; vy = 0; vyaw = 0;
            cout << "[GOAL REACHED] Cíl dosažen! Vzdálenost: " << dist_to_goal << " m" << endl;
            return;
        }

        // Rozhodnutí o nutnosti přeplánování
        bool needs_replan = (path_idx == -1 || path_idx >= (int)path.size());
        if (!needs_replan) {
            double dist_to_wp = hypot(path[path_idx].x - px, path[path_idx].y - py);

            // Přeplánuj pouze při ztrátě viditelnosti ke vzdálenějšímu waypointu
            if (dist_to_wp > 3.0) {
                if (!line_of_sight(grid, MAP_DIM,
                                   worldToGrid(px, py),
                                   worldToGrid(path[path_idx].x, path[path_idx].y))) {
                    needs_replan = true;
                }
            }
        }

        if (needs_replan) {
            Node goal_node = worldToGrid(tx, ty);
            int goal_occupancy = grid[goal_node.r * MAP_DIM + goal_node.c];

            if (!planThetaStar({px, py}, {tx, ty})) {
                // Plánování selhalo — pokračuj po staré trase, pokud existuje
                if (path_idx >= 0 && path_idx < (int)path.size()) {
                    static int fail_counter = 0;
                    if(++fail_counter % 50 == 1) {
                        cout << "[PLANNING FAILED] Nová cesta nenalezena, pokračuji po staré | Occupancy: "
                             << (int)goal_occupancy << " | WP: " << path_idx << "/" << path.size() << endl;
                    }
                } else {
                    // Žádná dostupná trasa — UAV zůstává na místě
                    vx = 0; vy = 0; vyaw = 0;
                    static int stuck_counter = 0;
                    if(++stuck_counter % 50 == 1) {
                        cout << "[STUCK] Žádná cesta k cíli | Occupancy: " << (int)goal_occupancy << endl;
                    }
                    return;
                }
            } else {
                planning_failures = 0;
                cout << "[PLANNING SUCCESS] Nová cesta nalezena | Délka: " << path.size()
                     << " bodů | Cíl occupancy: " << (int)goal_occupancy << endl;
            }
        }

        // Výpočet vektoru rychlosti k aktuálnímu waypointu
        Point target = path[path_idx];
        double dist_to_waypoint = hypot(target.x - px, target.y - py);
        double angle_err = atan2(target.y - py, target.x - px) - yaw;
        while (angle_err >  M_PI) angle_err -= 2*M_PI;
        while (angle_err < -M_PI) angle_err += 2*M_PI;

        // Diagnostický výpis stavu navigace (každých 20 kroků)
        static int diag_counter = 0;
        if (++diag_counter % 20 == 0) {
            cout << "[NAV] Pos:[" << px << "," << py << "] Yaw:" << (yaw*180/M_PI) << "°"
                 << " | Path:" << path_idx << "/" << path.size()
                 << " | WP dist:" << dist_to_waypoint << "m"
                 << " | Goal dist:" << dist_to_goal << "m"
                 << " | v:[" << vx << "," << vy << "]" << endl;
        }

        // Adaptivní rychlost — plynulé zpomalení při přibližování k waypointu
        double adaptive_speed = min(1.0, dist_to_waypoint / 3.0);
        adaptive_speed = max(0.3, adaptive_speed);

        // Při velké chybě kurzu se UAV nejprve natáčí, aby se zabránilo bočnímu driftu
        double turn_rate = (abs(angle_err) > M_PI/4) ? 0.8 : 1.2;

        vx   = cos(angle_err) * adaptive_speed * 1.2;
        vy   = sin(angle_err) * adaptive_speed * 1.2;
        vyaw = angle_err * turn_rate;

        // Saturace výstupních rychlostí
        vx   = max(-1.5, min(vx,   1.5));
        vy   = max(-1.5, min(vy,   1.5));
        vyaw = max(-1.5, min(vyaw, 1.5));

        // Přepnutí na další waypoint při dostatečném přiblížení
        if (dist_to_waypoint < 1.2 && path_idx < (int)path.size() - 1) {
            path_idx++;
            cout << "[NAV] Přepínám na další waypoint: " << path_idx << endl;
        }
    }
};

// ==============================================================================
// HLAVNÍ SMYČKA
// ==============================================================================
int main() {
    Robot *robot = new Robot();
    int ts = (int)robot->getBasicTimeStep();

    // Inicializace senzorů
    GPS *gps = robot->getGPS("gps"); gps->enable(ts);
    InertialUnit *imu = robot->getInertialUnit("inertial unit"); imu->enable(ts);
    Gyro *gyro = robot->getGyro("gyro"); gyro->enable(ts);
    Lidar *lidar = robot->getLidar("lidar");
    if(!lidar) lidar = robot->getLidar("Velodyne VLP-16");
    lidar->enable(ts);
    lidar->enablePointCloud();

    // Inicializace navigačních vrstev
    MavicAutopilot autopilot(robot);
    Navigator navigator;

    // Cílový waypoint mise
    double TX = -52.0, TY = -52.0;
    navigator.setGoal(TX, TY);

    // Hlavní řídicí smyčka — v každém kroku: čtení senzorů, mapování, plánování, řízení
    while (robot->step(ts) != -1) {
        const double *p   = gps->getValues();
        const double *rpy = imu->getRollPitchYaw();
        const double *g   = gyro->getValues();

        double vx = 0.0, vy = 0.0, vyaw = 0.0;

        navigator.updateMap(lidar, p[0], p[1], p[2], rpy[0], rpy[1], rpy[2]);
        navigator.getVelocity(p[0], p[1], rpy[2], TX, TY, vx, vy, vyaw);
        autopilot.update(p[2], rpy[0], rpy[1], rpy[2], vx, vy, 1.5, vyaw, g);
    }

    delete robot;
    return 0;
}