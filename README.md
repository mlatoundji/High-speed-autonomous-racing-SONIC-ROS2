# High-Speed Autonomous Racing — SONIC ROS 2

Projet D : robot de course autonome sur circuit (Ackermann), basé sur [AutoCarROS2](https://github.com/winstxnhdw/AutoCarROS2), **ROS 2 Humble** + Gazebo 11. Objectifs : racing line, contrôle haute vitesse, chronométrage, robustesse (latence + bruit odométrie).

## Résultats clés (F1, Albert Park ≈ 590 m)


| Étape             | Stack                            | Meilleur tour (s) | Notes                                    |
| ----------------- | -------------------------------- | ----------------- | ---------------------------------------- |
| I — ovale         | Stanley / MPC / **Pure Pursuit** | **92.5**          | PP retenu (racing line)                  |
| II — F1 open-loop | Pure Pursuit + waypoints CSV     | **86.7**          | racing line offline + finetune           |
| III — F1 LiDAR    | Pure Pursuit + SLAM              | **115.1**         | lap 1 explore, lap 2+ racing line online |


Gain principal : **géométrie racing line** (−57 % vs baseline centerline), pas une consigne de vitesse plus agressive. Plafond PP ≈ **8 m/s** sur ce circuit. Détail : `[docs/experimental_results.pdf](docs/experimental_results.pdf)`.

## Stacks de navigation

```text
/autocar/state2D ──► global_planner[_lidar] ──► /autocar/goals
                              │
                              ▼
                     local_planner (spline + vitesse, 10 Hz)
                              │
                              ▼
                     path_tracker PP / MPC (50 Hz) ──► /autocar/auto_cmd_vel
                              │
                              ▼
                     control_manager ──► /autocar/cmd_vel ──► Gazebo
```


| Launch                              | Usage                                      |
| ----------------------------------- | ------------------------------------------ |
| `race_launch.py`                    | Stanley (baseline)                         |
| `race_pure_pursuit_launch.py`       | PP + waypoints (`line:=centerline|racing`) |
| `race_pure_pursuit_lidar_launch.py` | PP + LiDAR + SLAM (F1, pas de CSV)         |
| `race_mpc_launch.py`                | MPC (bonus)                                |


Packages ajoutés : `autocar_nav_pure_pursuit`, `autocar_nav_pure_pursuit_lidar`, `autocar_racing_line`, `autocar_nav_mpc`, `autocar_gui`. Voir aussi `[src/AutoCarROS2/autocar_nav_pure_pursuit_lidar/README.md](src/AutoCarROS2/autocar_nav_pure_pursuit_lidar/README.md)`.

## Structure du dépôt

```text
autocar/
├── README.md
├── docs/                              # Rapports, synthèse expérimentale, figures
│   ├── experimental_results.pdf
│   └── images/
├── scripts/                           # Benchmark batch et analyse post-run
│   ├── benchmark.py
│   └── configs/                       # YAML campagnes (F1, latence, tuning…)
├── results/                           # Sorties de session (lap_times.csv, params.yaml)
├── container/                         # (optionnel) Docker Foxy legacy — noVNC + API
│   ├── compose.yaml
│   └── api/
└── src/AutoCarROS2/                   # Workspace ROS 2 (colcon build à la racine)
    ├── requirements.sh
    ├── launches/launch/               # race_launch.py, race_pure_pursuit_*.py, …
    ├── autocar_description/           # URDF / meshes véhicule
    ├── autocar_gazebo/                # Mondes Gazebo (f1, ovale, city…)
    ├── autocar_map/                   # Waypoints centerline
    ├── autocar_msgs/
    ├── autocar_nav/                   # Baseline Stanley, global/local planner
    ├── autocar_nav_pure_pursuit/      # Path tracker PP + waypoints CSV
    ├── autocar_nav_pure_pursuit_lidar/ # PP + LiDAR + SLAM (F1 sans CSV)
    ├── autocar_nav_mpc/               # Contrôleur MPC (bonus)
    ├── autocar_racing_line/           # Génération racing line (data/*.csv)
    └── autocar_gui/                   # Panneau de contrôle Qt
```

Artefacts de build (`build/`, `install/`, `log/`) créés par `colcon` à la racine ; `results/` alimenté par les lancements et `scripts/benchmark.py`.

## Installation

**Prérequis :** Ubuntu 22.04, ROS 2 Humble, Gazebo 11, `colcon`, `rosdep`. WSL2 : affichage graphique (WSLg ou X).

```bash
git clone <url-du-depot> autocar && cd autocar
cd src/AutoCarROS2 && sh requirements.sh && cd ../..
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### Docker (optionnel)

Le chemin recommandé est l’installation native ci-dessus (Humble). Le dossier `container/` propose une stack **ROS 2 Foxy** embarquée, utile seulement si vous ne pouvez pas installer Humble localement :

```bash
cd container && docker compose up --build
```

noVNC : `http://localhost:6080` · API : `:8001`.

## Lancer

```bash
# PP F1 racing line (meilleur chrono open-loop)
ros2 launch launches race_pure_pursuit_launch.py track:=f1_circuit line:=racing

# PP + LiDAR (défaut : f1_circuit_fenced ; lap 1 explore, lap 2+ racing line online)
ros2 launch launches race_pure_pursuit_lidar_launch.py line:=racing

# Stanley baseline
ros2 launch launches race_launch.py control_mode:=auto gui:=true rviz:=true
```

Benchmark batch (latence / bruit) :

```bash
python3 scripts/benchmark.py --config scripts/configs/f1_circuit.yaml
```

Chaque session écrit `results/<stack>_<timestamp>/lap_times.csv` + `params.yaml`. Agrégats : `results/benchmark_*/summary.csv`.

## Interface GUI

```bash
pip install -r src/AutoCarROS2/autocar_gui/requirements.txt
colcon build --packages-select autocar_gui
ros2 run autocar_gui control_panel.py   # avec simulation lancée
```

## Dépannage

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
```

Si nodes fantômes ou Gazebo bloqué :

```bash
pkill -9 -f "ros2 launch|tracker.py|localplanner.py|globalplanner|slam|lap_timer" 2>/dev/null
killall -9 gzserver gzclient 2>/dev/null
ros2 daemon stop && sleep 2 && ros2 daemon start
```

Rebuild propre : `rm -rf build install log && colcon build`.