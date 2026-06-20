# Référence des paramètres de configuration

Ce document recense **tous** les paramètres qui pilotent la pile AutoCarROS2
(localisation, planification, contrôle, arbitrage, instrumentation,
perturbations) ainsi que les arguments de lancement et de benchmark. Pour
chaque paramètre on trouve : son rôle, l'effet des différentes valeurs, des
recommandations de réglage, ses dépendances avec d'autres paramètres, et un
exemple concret.

Le document se termine par des **profils prêts à l'emploi** (conservateur,
équilibré, agressif) et un **workflow de tuning** pour partir des profils et
adapter la voiture à une nouvelle piste.

> **Lecture rapide :** si vous voulez juste lancer une course, sautez à
> [§ 8 Profils prêts à l'emploi](#8-profils-prêts-à-lemploi). Si vous changez
> de circuit, lisez d'abord [§ 7 Workflow de tuning](#7-workflow-de-tuning).

## Table des matières

- [1. Vue d'ensemble — où vivent les paramètres](#1-vue-densemble--où-vivent-les-paramètres)
- [2. Arguments de lancement (`ros2 launch`)](#2-arguments-de-lancement-ros2-launch)
- [3. Harness de benchmark (YAML `scripts/configs/*.yaml`)](#3-harness-de-benchmark-yaml-scriptsconfigsyaml)
- [4. Arbitre de contrôle — `control_manager`](#4-arbitre-de-contrôle--control_manager)
- [5. Instrumentation — `lap_timer` et collisions](#5-instrumentation--lap_timer-et-collisions)
- [6. Perturbations perception — latence et bruit](#6-perturbations-perception--latence-et-bruit)
- [7. Stack de navigation](#7-stack-de-navigation)
  - [7.1 `localisation`](#71-localisation)
  - [7.2 `global_planner`](#72-global_planner)
  - [7.3 `local_planner`](#73-local_planner)
  - [7.4 `path_tracker` — Stanley](#74-path_tracker--stanley)
  - [7.5 `path_tracker` — Pure Pursuit](#75-path_tracker--pure-pursuit)
  - [7.6 `path_tracker` — MPC](#76-path_tracker--mpc)
  - [7.7 `global_planner_lidar` — LiDAR / SLAM](#77-global_planner_lidar--lidar--slam)
- [8. Profils prêts à l'emploi](#8-profils-prêts-à-lemploi)
- [9. Workflow de tuning](#9-workflow-de-tuning)

---

## 1. Vue d'ensemble — où vivent les paramètres

Quatre couches se superposent ; chaque couche peut surcharger la précédente :

| Couche | Fichier / mécanisme | Portée |
| --- | --- | --- |
| **A.** Valeurs par défaut codées dans les nœuds | `nodes/*.py` (`declare_parameters`) | Filets de sécurité quand rien d'autre n'est défini. |
| **B.** YAML de navigation du pack | `src/AutoCarROS2/<navpkg>/config/navigation_params.yaml` | Paramètres ROS de `localisation`, `local_planner`, `global_planner` (ou `global_planner_lidar`), `path_tracker` pour une stack donnée. |
| **C.** Arguments de lancement | `ros2 launch ... arg:=valeur` | Métadonnées d'expérience (stack, ligne, latence, bruit, mode, monde…) et points d'injection. |
| **D.** Surcharge YAML inline du benchmark | `scripts/configs/*.yaml` → bloc `navigation:` | Remplace **entièrement** le YAML par-défaut d'une stack le temps d'une expérience. |

> ⚠️ **Important** : un bloc `navigation:` inline dans le YAML benchmark
> **remplace** le YAML par défaut, il ne le complète pas. Tout paramètre absent
> retombe sur la valeur codée en dur dans le nœud, **pas** sur le YAML pack
> (`navigation_params.yaml`). Voir l'avertissement dans
> [`f1_circuit.yaml`](../scripts/configs/f1_circuit.yaml) à propos du
> `steering_rate_limit`.

Flux d'un message d'état :

```text
Gazebo → /autocar/state2D_raw → latency_injector → /autocar/state2D_mid
                              → odom_noise_injector → /autocar/state2D
                              → planners + tracker → /autocar/auto_cmd_vel
                              → control_manager (arbitrage + sécurité)
                              → /autocar/cmd_vel → Gazebo
```

---

## 2. Arguments de lancement (`ros2 launch`)

Déclarés dans
[`race_launch_common.py`](../src/AutoCarROS2/launches/launch/race_launch_common.py).
Tous sont passables en ligne de commande : `ros2 launch launches race_launch.py arg:=val`.

### `use_sim_time`
- **Type / défaut :** `bool` / `true`.
- **Rôle :** synchronise tous les nœuds sur l'horloge Gazebo (`/clock`).
- **Effet :** `true` (recommandé en simulation) garantit que les latences et
  durées de tour sont mesurées en *temps simulé*, donc reproductibles même si
  la simulation tourne en slow-motion sur un PC chargé. `false` utilise
  l'horloge système — utile uniquement pour piloter un vrai robot.
- **Dépendances :** doit rester cohérent entre tous les nœuds (lap_timer,
  trackers, injectors).
- **Exemple :** `ros2 launch launches race_launch.py use_sim_time:=true`.

### `gui`
- **Type / défaut :** `bool` / `true`.
- **Rôle :** lance ou non `gzclient` (vue 3D de Gazebo).
- **Effet :** mettre à `false` économise ~1 GB de RAM et 1 cœur — préférer
  pour les benchmarks longs en headless / WSL.
- **Exemple :** `ros2 launch launches race_launch.py gui:=false rviz:=true`.

### `rviz`
- **Type / défaut :** `bool` / `true`.
- **Rôle :** lance RViz2 avec la configuration `view.rviz`.
- **Effet :** RViz reste plus léger que `gzclient` et affiche les
  marqueurs de status, la trajectoire et l'erreur latérale — gardez-le en
  cas de fragilité graphique côté Gazebo.

### `world`
- **Type / défaut :** chemin / `race_circuit.world`.
- **Rôle :** monde Gazebo à charger.
- **Valeurs typiques :** `race_circuit.world` (anneau de référence),
  `autocar.world` (route droite), `autocar_city.world`,
  `populated_road.world`, ou un `<piste>.world` custom.
- **Exemple :** `world:=$(ros2 pkg prefix autocar_gazebo)/share/autocar_gazebo/worlds/race_circuit.world`.

### `line`
- **Type / défaut :** `centerline` ou `racing` / `centerline`.
- **Rôle :** choisit le fichier de waypoints publié par `global_planner`.
- **Effet :** `centerline` = trajectoire centrale (stable, lente) ;
  `racing` = trajectoire optimisée (rapide, exige des gains plus calmes).
- **Dépendances :** sur PP, raccourcir `lookahead_min` et baisser
  `lookahead_gain` est souvent nécessaire pour bien suivre `racing` ; voir
  [`r5_pp_racing_lookahead.yaml`](../scripts/configs/r5_pp_racing_lookahead.yaml).

### `control_mode`
- **Type / défaut :** `manual`, `semi`, `auto` / `auto`.
- **Rôle :** mode initial de l'arbitre de contrôle (`control_manager`).
- **Valeurs :**
  - `manual` : seul `/autocar/manual_cmd_vel` est suivi ; la voiture reste à
    l'arrêt sans téléopération.
  - `semi` : autonomie active, mais une commande manuelle récente prend la
    main pendant `semi_override_timeout_s`.
  - `auto` : autonomie complète, ignore toute commande manuelle.
- **Dépendances :** voir [§ 4](#4-arbitre-de-contrôle--control_manager).
- **Exemple :** `control_mode:=semi` pour pouvoir reprendre la main au clavier
  ou via le panneau `autocar_gui`.

### `camera_mode`
- **Type / défaut :** `free`, `top`, `follow` / `follow`.
- **Rôle :** vue par défaut dans RViz (`follow` suit la voiture).

### `profile`
- **Type / défaut :** chaîne libre / `default`.
- **Rôle :** **étiquette** enregistrée dans `lap_times.csv` pour différencier
  des runs sans changer les paramètres. Ne sélectionne **pas** automatiquement
  un YAML.
- **Recommandation :** utilisez `conservateur`, `equilibre`, `agressif`,
  `f1_baseline`, etc.

### `latency_ms`
- **Type / défaut :** `int` (ms) / `0`.
- **Rôle :** retarde `/autocar/state2D_raw` → `/autocar/state2D_mid`.
- **Effet :** voir [§ 6.1](#61-latency_injector). 0 = pass-through.
- **Exemple :** `latency_ms:=200` pour simuler un GPS lent.

### `odom_noise_std`
- **Type / défaut :** `float` (m, rad) / `0.0`.
- **Rôle :** bruit gaussien additif sur `pose.x`, `pose.y`, `pose.theta`.
- **Effet :** voir [§ 6.2](#62-odom_noise_injector).

---

## 3. Harness de benchmark (YAML `scripts/configs/*.yaml`)

Chaque entrée du YAML décrit **un run** (une combinaison stack × paramètres).
Le harness ([`scripts/benchmark.py`](../scripts/benchmark.py)) répète chaque
run, agrège `lap_times.csv` et `collisions.csv`.

| Clé | Défaut | Description |
| --- | --- | --- |
| `stack` | `stanley` | `stanley`, `pure_pursuit`, `mpc`, `pure_pursuit_lidar`. Choisit le pack de nodes nav lancé. `pure_pursuit_lidar` lance en plus `slam_toolbox` et `global_planner_lidar`. |
| `profile` | `default` | Étiquette dans le CSV uniquement. |
| `line` | `centerline` | `centerline` ou `racing`. Pour `pure_pursuit_lidar`, `racing` s'applique au tour 2+ (ligne extraite de la carte SLAM du tour 1). |
| `track` | `circuit` | Monde Gazebo : `circuit`, `oval`, `f1_circuit`, `f1_circuit_fenced` (variante F1 avec clôtures — défaut du launch LiDAR). |
| `latency_ms` | `0` | Latence injectée sur `state2D`. |
| `odom_noise_std` | `0.0` | σ de bruit sur la pose. |
| `lap_count` | `3` | Nombre de tours **attendus** avant de tuer le launch. |
| `warmup_laps` | `1` | Nombre de tours exclus de `summary.csv` (mise en régime). |
| `camera` | `true` | Active la caméra arrière Gazebo (CPU-gourmande). Mettre `false` en headless. |
| `navigation` | absent | Bloc YAML inline qui **remplace** `navigation_params.yaml` pour ce run. Voir [§ 7](#7-stack-de-navigation). |

> ⚠️ Si `navigation:` est présent, il doit contenir **l'arbre complet**
> (`localisation`, `local_planner`, `global_planner` *ou* `global_planner_lidar`,
> `path_tracker`). Toute clé manquante retombe sur la valeur codée en dur dans
> le nœud, pas sur le YAML pack.

**Stack LiDAR** — voir
[`scripts/configs/f1_pure_pursuit_lidar.yaml`](../scripts/configs/f1_pure_pursuit_lidar.yaml) :
`lap_count ≥ 2` (tour 1 = exploration SLAM, tour 2+ = racing line lissée),
`warmup_laps: 1` (le tour 1 est exclu de `summary.csv`), `track: f1_circuit_fenced`,
`line: racing`, `camera: false` recommandé en headless.

```bash
python3 scripts/benchmark.py --config scripts/configs/f1_pure_pursuit_lidar.yaml
```

Exemple minimal (stack CSV) :

```yaml
- stack: pure_pursuit
  profile: equilibre
  line: racing
  latency_ms: 0
  odom_noise_std: 0.0
  lap_count: 3
  warmup_laps: 1
```

Lancement : `python3 scripts/benchmark.py --config scripts/configs/mon_run.yaml`.

---

## 4. Arbitre de contrôle — `control_manager`

Fichier : [`control_manager.py`](../src/AutoCarROS2/autocar_nav/nodes/control_manager.py).
Tous les paramètres sont des `ros__parameters` du nœud `control_manager`.

### `update_frequency` (Hz)
- **Défaut :** `50.0`.
- **Rôle :** fréquence du timer principal qui re-publie `/autocar/cmd_vel`.
- **Effet :** valeurs élevées (≥50) → meilleure latence de commande ; au-delà
  de 100 vous risquez de saturer le CPU sans gain notable. <20 Hz augmente
  l'oscillation perçue.
- **Recommandation :** laisser à 50 ; descendre à 30 si CPU saturé.

### `initial_mode`
- **Défaut :** `auto`.
- **Rôle :** mode au démarrage du nœud (voir l'argument
  [`control_mode`](#control_mode)).
- **Effet :** identique à `control_mode` mais évalué côté nœud.

### `manual_timeout_s`
- **Défaut :** `0.35` s.
- **Rôle :** délai pendant lequel une commande sur `/autocar/manual_cmd_vel`
  reste active après le dernier message reçu (en mode `manual`).
- **Effet :** trop court → la voiture se met à l'arrêt entre deux paquets ;
  trop long → la voiture continue d'avancer après lâcher de la commande.
- **Recommandation :** ~0.3–0.5 s pour 10 Hz de téléopération.

### `semi_override_timeout_s`
- **Défaut :** `1.0` s.
- **Rôle :** délai pendant lequel le mode `semi` privilégie la commande
  manuelle après le dernier message.
- **Recommandation :** plus grand que `manual_timeout_s` car l'override doit
  rester perceptible à l'utilisateur.

### `max_accel_mps2`
- **Défaut :** `10.0` m/s².
- **Rôle :** limite la variation de vitesse longitudinale par cycle.
- **Effet :** valeurs faibles (4) donnent un démarrage très progressif et
  réduisent le risque de patinage, mais bridant les phases d'accélération en
  ligne droite. Valeurs élevées (12+) raccourcissent les temps morts mais
  peuvent provoquer du sous-virage à la sortie de courbe.
- **Recommandation :** 8–12 pour la course ; 4–6 pour rouler en ville ou en
  test conservateur.
- **Note :** ce limiteur est **désactivé en mode auto** depuis la correction
  du problème d'oscillation — voir `_rate_limit` dans le code et la note ci-dessous.

### `max_steer_rate_radps`
- **Défaut :** `8.0` rad/s.
- **Rôle :** vitesse angulaire maximale de braquage en mode `manual`/`semi`.
- **Effet :** trop bas (≤2) → la commande arrive en retard → oscillations en
  boucle fermée ; trop haut (≥15) → secousses brusques côté Gazebo.
- **Recommandation :** 6–10 pour des commandes humaines ; en mode `auto`,
  cette valeur n'a plus d'effet (`_rate_limit` est court-circuité).
- **Historique :** baissé à 1.8 lors du merge `setup_ros_container` →
  oscillations massives — voir [§ 9.4](#94-anti-oscillation).

### `max_steer` (rad)
- **Défaut :** `0.85` rad ≈ 49°.
- **Rôle :** cap absolu du braquage envoyé à Gazebo (toujours actif, même en
  `auto`).
- **Effet :** doit être ≥ `steering_limits` du tracker, sinon on coupe les
  consignes du tracker.
- **Recommandation :** ne pas descendre sous 0.6 sur le circuit de course
  (≈34°), sinon les épingles deviennent infaisables.

### `stuck_speed_threshold` (m/s)
- **Défaut :** `0.15`.
- **Rôle :** vitesse mesurée en-dessous de laquelle on considère que la
  voiture est immobile.

### `stuck_command_threshold` (m/s)
- **Défaut :** `0.8`.
- **Rôle :** commande de vitesse au-dessus de laquelle, *si la voiture ne
  bouge pas*, on déclenche un état "stuck".

### `stuck_time_s` (s)
- **Défaut :** `1.2`.
- **Rôle :** durée minimale d'un état "commande non suivie" avant que
  `_detect_collision` ne déclenche la latching collision.
- **Effet :** augmenter (2–3 s) si la voiture déclenche faussement à
  l'accélération en sortie de virage.

### `collision_hold_s` (s)
- **Défaut :** `0.8`.
- **Rôle :** durée pendant laquelle on force `zero_twist` après détection
  d'une collision, pour laisser la voiture s'arrêter avant de réessayer.
- **Effet :** trop court → on reprend la commande alors que la voiture est
  encore engagée dans l'obstacle ; trop long → temps mort visible.

### `prolonged_collision_threshold_s` (s)
- **Défaut :** `120.0`.
- **Rôle :** si l'état de collision (`collision_latched`) persiste plus
  longtemps que ce seuil, l'expérience est considérée terminée :
  - log `ERROR`,
  - publication latched `/autocar/experiment_terminated = true`,
  - `_rate_limit` force `zero_twist` indéfiniment,
  - `lap_timer` écrit un row partiel (`experiment_terminated = 1`).
- **Effet :** régler à 60 s pour un benchmark agressif où l'on coupe court ;
  à 180 s si la voiture peut se sortir seule d'une mauvaise position.
- **Reset :** publier `True` sur `/autocar/resume_auto` annule la latche.
- **Voir :** [§ 5](#5-instrumentation--lap_timer-et-collisions).

### Empreinte de la voiture (`footprint_length`, `footprint_width`)
- **Défauts :** `4.8` m × `2.1` m.
- **Rôle :** rectangle utilisé pour échantillonner la carte `/map` et
  détecter si le pied de roue empiète sur une cellule occupée (≥ 80).
- **Effet :** trop grand → faux positifs (la voiture se met en collision
  alors qu'elle frôle un mur) ; trop petit → on détecte la collision après
  le choc.
- **Cohérence :** doit refléter l'URDF du véhicule.

### Garde-corps géométriques (`track_inner_radius`, `track_outer_radius`, `track_boundary_margin`)
- **Défauts :** `96.0`, `112.0`, `0.8` m.
- **Rôle :** sortie de piste *logique* (en plus du `/map`) : si
  `√(x² + y²)` sort de `[inner-margin, outer+margin]`, on signale collision.
- **Effet :** spécifique au circuit anneau de référence ; à recalibrer si
  vous changez de monde.

---

## 5. Instrumentation — `lap_timer` et collisions

Fichier :
[`lap_timer.py`](../src/AutoCarROS2/autocar_nav/nodes/lap_timer.py).
Schema CSV :
[`lap_times_paths.py`](../src/AutoCarROS2/autocar_nav/autocar_nav/lap_times_paths.py).

### Paramètres
- `stack` (str) : étiquette de stack (`stanley` / `pure_pursuit` / `mpc` /
  `pure_pursuit_lidar`). Détermine le dossier `results/<stack>_<run_id>/`.
- `run_id` (str, opt) : identifiant explicite ; sinon `YYYY-MM-DDTHH-MM-SS`.
- `run_dir` (str, opt) : dossier de sortie complet ; surcharge `stack`/`run_id`.
- `lap_times_csv` (str, opt) : chemin explicite vers `lap_times.csv`.
- `collisions_csv` (str, opt) : chemin explicite vers `collisions.csv`.
- `profile` (str) : étiquette CSV.
- `latency_ms`, `odom_noise_std` : étiquettes CSV (les vraies valeurs sont
  appliquées par les injectors).
- `offtrack_threshold_m` (float, défaut 4.0) : seuil d'erreur latérale
  au-dessus duquel un événement "offtrack" est compté.
- `prolonged_collision_threshold_s` (float, défaut 120.0) : doit être ≥ celui
  du `control_manager` pour rester cohérent.

### Schéma `lap_times.csv` (20 colonnes)

| Colonne | Type | Sens |
| --- | --- | --- |
| `session_id` | str | Identifiant unique du run. |
| `lap_number` | int | 1-indexé, incrémenté au passage de la ligne. |
| `timestamp_iso` | str | Horodatage UTC ISO à l'écriture. |
| `duration_s` | float | Durée du tour. |
| `avg_speed_mps`, `max_speed_mps` | float | Vitesse moyenne / pic mesurée. |
| `distance_m` | float | Distance accumulée pendant le tour. |
| `controller` | str | `stanley` / `pure_pursuit` / `mpc`. |
| `profile`, `latency_ms`, `odom_noise_std` | — | Étiquettes d'expérience. |
| `lateral_error_rms`, `lateral_error_max` | float | Erreur latérale (m). |
| `steering_rate_max` | float | Pic de vitesse angulaire de braquage observée. |
| `offtrack_events` | int | Nombre d'événements offtrack pendant le tour. |
| `collision_events` | int | Nombre de collisions terminées (ou en cours) attribuées à ce tour. |
| `collision_total_s` | float | Temps cumulé en collision **dans ce tour**. |
| `collision_max_s` | float | Plus longue collision active (incluant les segments). |
| `collision_mean_s` | float | Moyenne. |
| `experiment_terminated` | 0/1 | Flag du dernier row si fin par seuil prolongé. |

### Schéma `collisions.csv` (8 colonnes)

`session_id, lap_number, event_index, start_iso, start_elapsed_s, duration_s, ended_by_termination, reason`.

Chaque collision génère une ligne au front descendant (sauf si interrompue
par la terminaison d'expérience — `ended_by_termination=1`).

### Topics publiés
- `/autocar/lap_time` — durée du dernier tour.
- `/autocar/current_lap_time` — chronomètre en cours.
- `/autocar/lap_count` — compteur.

### Topics consommés
- `/autocar/state2D` — détection de croisement de ligne.
- `/autocar/lateral_error` — RMS/max/offtrack.
- `/autocar/cmd_vel` — `steering_rate_max`.
- `/autocar/collision` (Bool) — fronts montants/descendants.
- `/autocar/control_status` (String/JSON) — capture du `collision_reason`.
- `/autocar/experiment_terminated` (Bool, latched QoS) — flush partiel.

---

## 6. Perturbations perception — latence et bruit

### 6.1 `latency_injector`
- **Paramètre :** `latency_ms` (int, défaut 0).
- **Effet :** retarde `state2D_raw` → `state2D_mid` d'exactement N ms. Mode
  pass-through si 0.
- **Recommandation :** balayer `[0, 100, 200, 300, 500, 1000]` (cf.
  [`r4_latency_sweep.yaml`](../scripts/configs/r4_latency_sweep.yaml)).
  Performance attendue : courbe en U avec optimum vers 200–300 ms (le
  controller anticipe à la bonne échelle), dégradation rapide >500 ms.
- **Interaction :** Pure Pursuit avec `lookahead_gain` élevé tolère mieux la
  latence que Stanley.

### 6.2 `odom_noise_injector`
- **Paramètre :** `odom_noise_std` (float, défaut 0.0).
- **Effet :** bruit gaussien sur `pose.x`, `pose.y`, `pose.theta` (mêmes
  unités m / rad).
- **Recommandation :** rester ≤ 0.10 m sur un anneau de 100 m de rayon ;
  au-delà la localisation devient incohérente avec la map et les sorties de
  piste explosent.
- **Interaction :** `steer_smoothing` ≥ 1.0 et `lateral_soft` ≥ 4.0 lissent
  significativement la sensibilité au bruit.

---

## 7. Stack de navigation

Les sections suivantes décrivent les paramètres ROS publiés dans
`navigation_params.yaml` (un par stack). Les valeurs **par défaut** indiquées
sont celles du pack Pure Pursuit (CSV), qui sert de référence ; les écarts
pour Stanley, MPC et `pure_pursuit_lidar` (LiDAR / SLAM) sont signalés.

### 7.1 `localisation`

#### `update_frequency` (Hz)
- **Défaut :** `50.0`.
- **Rôle :** fréquence de publication de `state2D` post-traité.
- **Effet :** doit ≥ `path_tracker.update_frequency` ; au-delà gaspille du CPU.

#### `use_slam` (`pure_pursuit_lidar` uniquement)
- **Défaut :** `true`.
- **Rôle :** si `true`, la pose publiée sur `/autocar/state2D` est corrigée
  par la TF SLAM (`map` → `base_link` via `map` → `odom`). Si `false`, seule
  l'odométrie roues (`/autocar/odom`) est utilisée.
- **Effet :** indispensable pour la pile LiDAR (lap 1 construit la carte,
  lap 2+ suit la racing line extraite de `/map`). Désactiver uniquement pour
  déboguer l'odom seule.
- **Dépendances :** `slam_toolbox` doit être actif et publier `map` → `odom`.

### 7.2 `global_planner`

#### `update_frequency` (Hz)
- **Défaut :** `2.0`.
- **Rôle :** fréquence de republication des waypoints "près du véhicule" sur
  `/autocar/goals`.
- **Effet :** 2 Hz suffit pour des waypoints espacés d'1 m à 8 m/s ; monter à
  5 Hz si vous voyez des sauts de waypoint en virage.

#### `waypoints_ahead`, `waypoints_behind`
- **Défauts :** `5`, `2` (Pure Pursuit) ; `3`, `2` (Stanley).
- **Rôle :** combien de waypoints à publier devant/derrière la position
  courante.
- **Effet :** plus haut → meilleure prévision pour `local_planner` mais plus
  de calcul ; trop bas → tracker myope dans les virages serrés.
- **Recommandation circuit serré (F1) :** `waypoints_ahead ≥ 5`.

#### `passed_threshold` (m)
- **Défaut :** `0.25`.
- **Rôle :** distance à laquelle on considère un waypoint "atteint".

#### `waypoint_search_ahead`
- **Défaut :** `30` (PP/MPC). Absent en Stanley.
- **Rôle :** fenêtre de recherche du waypoint "le plus proche en avant"
  utilisée à chaque tic pour éviter de revenir en arrière sur la boucle.

#### `waypoints_file`
- **Défaut :** `waypoints.csv` (centerline).
- **Rôle :** nom de fichier CSV (relatif au `share/` du pack) ; surchargé
  par l'argument `line` :
  - `centerline` → `waypoints.csv`
  - `racing` → `waypoints_racing.csv`

#### `centreofgravity_to_frontaxle`
- **Défaut :** `1.483` m.
- **Rôle :** géométrie du véhicule, doit refléter l'URDF.

### 7.3 `local_planner`

Publie `/autocar/path` (trajectoire à suivre, possiblement décalée pour
évitement) et `/autocar/target_velocity`.

#### `update_frequency` (Hz)
- **Défaut :** `10.0`.

#### `car_width` (m)
- **Défaut :** `2.0`.
- **Rôle :** largeur prise en compte pour l'évitement.

#### `frame_id`
- **Défaut :** `base_link`.

#### `exploration_velocity` (m/s, `pure_pursuit_lidar` uniquement)
- **Défaut :** `3.0`.
- **Rôle :** vitesse cible pendant le tour 1 (mode exploration :
  `nav_mode < 1`). Le tour 2+ bascule sur `cruise_velocity`.
- **Effet :** un tour 1 trop rapide dégrade la carte SLAM et la centerline
  enregistrée ; un tour 1 trop lent allonge le benchmark. `3.0 m/s` est le
  compromis retenu dans [`f1_pure_pursuit_lidar.yaml`](../scripts/configs/f1_pure_pursuit_lidar.yaml).

#### `cruise_velocity`, `avoid_velocity` (m/s)
- **Défauts :** `8.0` (CSV) ; `6.0` (LiDAR baseline F1).
- **Rôle :** vitesse cible en ligne droite (cruise) et en évitement (avoid).
- **Effet :** déterminent directement le temps au tour. À 8 m/s sur le
  circuit de référence on est à ≈ 80 s/tour théoriques.
- **Recommandation :** garder `cruise == avoid` pour éviter les coups de
  frein lors d'un changement de couloir (cf. commentaire de `localplanner.py`).
- **Plafond pratique :** `max_lateral_accel × R_min ≈ 5.5 × 96 ≈ 7.5 m/s` sur
  l'anneau ; au-dessus, le speed shaping freine de toute façon.

#### `max_lateral_accel` (m/s²)
- **Défaut :** `5.5`.
- **Rôle :** limite \(v ≤ \sqrt{a_{lat,max} / κ}\) appliquée par le
  planificateur de vitesse en fonction de la courbure.
- **Effet :** monter à 6.5 = passer plus vite en virage mais frôler le
  patinage / sortie de piste sur racing line ; descendre à 4 = pilotage très
  conservateur (utile sur sol glissant simulé).

#### `min_curvature` (1/m)
- **Défaut :** `0.012`.
- **Rôle :** plancher de courbure |κ| utilisé dans le speed shaping. En-deçà,
  on considère qu'on est en ligne droite.
- **Effet :** trop bas → freinage tardif sur courbes douces ; trop haut →
  freinage permanent même en ligne droite.

#### `curvature_lookahead` (n. waypoints)
- **Défaut :** `140`.
- **Rôle :** combien de waypoints en avant on regarde pour calculer la
  courbure max → vitesse cible.
- **Effet :** plus long = anticipation plus douce mais réaction tardive ;
  plus court = freinage plus tardif et plus brusque.

#### `curvature_smooth_window` (n. impair)
- **Défaut :** `5`.
- **Rôle :** taille de la moyenne glissante sur la courbure (anti-bruit).
- **Effet :** monter à 9 lisse les oscillations de vitesse sur des waypoints
  bruyants ; trop haut (≥15) masque les vraies courbes.

#### `accel_rate`, `decel_rate` (m/s²)
- **Défauts :** `5.0`, `7.0`.
- **Rôle :** slew-rate longitudinal envoyé à `path_tracker` (différent du
  `max_accel_mps2` du `control_manager` qui n'agit qu'en `manual`/`semi`).
- **Effet :** raccourcir le freinage (`decel_rate` ≥ 8) gagne du temps en
  fin de ligne droite ; rallonger l'accélération (`accel_rate` ≤ 4) est utile
  pour limiter le patinage en sortie de virage.

#### `path_resolution`, `obstacle_check_horizon`, `obstacle_blocked_fraction` (MPC uniquement)
- **Défauts MPC :** `0.1` m, `25.0` m, `0.35`.
- **Rôle :** résolution du path échantillonné pour MPC, horizon d'inspection
  obstacle, fraction de cellules occupées qui déclenche un évitement.

#### `obstacle_avoidance` (PP)
- **Défaut :** `False`.
- **Rôle :** active l'évitement latéral sur `/map`. Désactivé sur l'anneau
  car les murs sont marqués occupés (voir [`docs/nan_crash_root_cause.md`](nan_crash_root_cause.md)).

### 7.4 `path_tracker` — Stanley

Pack `autocar_nav`. Sortie : `/autocar/auto_cmd_vel`.

#### `update_frequency` (Hz)
- **Défaut :** `50.0`. Doit rester ≥ 50 pour un suivi propre.

#### `control_gain` (k)
- **Défaut :** `1.0`.
- **Rôle :** gain crosstrack (Stanley).
- **Effet :** **dominant pour la stabilité**. Trop haut (≥2) →
  oscillations ; trop bas (≤0.4) → suivi très mou, sortie de piste en
  épingle.
- **Recommandation :** 0.7 (agressif sur racing) – 1.2 (centerline tranquille).

#### `softening_gain` (k_soft)
- **Défaut :** `1.0`.
- **Rôle :** stabilisateur Stanley à basse vitesse (`δ = atan(k * e / (v + k_soft))`).
- **Effet :** monter à 2 supprime les twitchs sous 1 m/s mais ralentit la
  réponse à pleine vitesse.

#### `yawrate_gain`
- **Défaut :** `1.0`.
- **Rôle :** poids du terme yaw-rate dans la consigne.

#### `steering_limits` (rad)
- **Défaut :** `0.95`.
- **Rôle :** saturation côté tracker (avant `control_manager`).
- **Recommandation :** ≥ `max_steer` du control_manager pour ne pas écraser
  deux fois.

#### `centreofgravity_to_frontaxle`
- **Défaut :** `1.483` m.

### 7.5 `path_tracker` — Pure Pursuit

Pack `autocar_nav_pure_pursuit`.

#### `update_frequency` (Hz)
- **Défaut :** `50.0`.

#### `wheelbase` (m)
- **Défaut :** `2.966`. Doit refléter l'URDF.

#### `centreofgravity_to_frontaxle` / `centreofgravity_to_rearaxle`
- **Défauts :** `1.483`.

#### Pure Pursuit — `lookahead_gain` (k_pp), `lookahead_min`, `lookahead_max` (m)
- **Défauts :** `0.4`, `1.5`, `6.0`.
- **Rôle :** distance de suivi \(L_d = \text{clip}(k_{pp} \cdot v, L_{min}, L_{max})\).
- **Effet :**
  - `k` ↑ → suit un point plus loin → moins d'oscillation mais coupe plus
    large les épingles.
  - `k` ↓ → tracking plus serré, plus nerveux.
  - `lookahead_min` est dominant à basse vitesse (≤ 5 m/s). À 1.5 m la voiture
    devient nerveuse ; 2.5 m la calme dans les apex.
  - `lookahead_max` borne le suivi à haute vitesse (utile si le YAML est
    appliqué sur un circuit où `v > 15 m/s`).
- **Exemple F1 :** `k=0.3, min=1.0, max=6.0` (virages serrés, voir
  [`f1_circuit.yaml`](../scripts/configs/f1_circuit.yaml)).

#### `closest_search_ahead`
- **Défaut :** `120`.
- **Rôle :** taille de la fenêtre forward dans laquelle on cherche le point
  le plus proche du chemin.
- **Effet :** descendre à 100 réduit les "sauts" sur géométries serrées,
  monter à 150 lisse sur racing line lente.

#### `steering_limits` (rad)
- **Défaut :** `0.95`.

#### `steering_rate_limit` (rad/s)
- **Défaut :** `4.0`.
- **Rôle :** anti-oscillation **côté tracker** (différent du
  `max_steer_rate_radps` du control_manager). Écrête le scie de braquage qui
  apparaît quand racing line et PP "tirent" dans des directions différentes.
- **Effet :** `0.0` = illimité (la voiture peut secouer en NaN sur F1) ;
  `4.0` = bon compromis ; `2.0` = très lisse mais sous-vire en épingle.
- **Voir :** [`docs/nan_crash_root_cause.md`](nan_crash_root_cause.md).

#### `steer_smoothing`
- **Défaut :** `1.0`.
- **Rôle :** filtre passe-bas exponentiel sur le braquage (1.0 = pas de
  filtre, 0.5 = forte atténuation).
- **Effet :** descendre à 0.82 calme une voiture qui shake quand racing line
  et lookahead PP divergent.

#### `velocity_gain`
- **Défaut :** `1.0`.
- **Rôle :** facteur multiplicatif sur la vitesse cible publiée par
  `local_planner`. Sert à scaler la pile sans toucher au YAML local_planner.

#### `startup_ramp_s` (s)
- **Défaut :** `2.0`.
- **Rôle :** rampe progressive de la vitesse cible à partir de 0 à l'init.

#### `lateral_soft`, `heading_soft`
- **Défauts :** `4.0`, `0.6`.
- **Rôle :** poids "soft" dans la pénalité de suivi (plus haut = on tient
  plus au chemin / à l'orientation).
- **Effet :** `lateral_soft` ↑ → reste sur la racing line au prix du temps ;
  `heading_soft` ↑ → cap plus précis (utile pour générer un temps de tour
  propre).
- **Voir :** [`r5_pp_racing_speed_soft.yaml`](../scripts/configs/r5_pp_racing_speed_soft.yaml).

#### Lookahead adaptatif à la courbure (`pure_pursuit_lidar` uniquement)
- **`lookahead_curv_extra`** (m, défaut `0.0`) : allonge le lookahead sur les
  lignes droites (courbure faible) pour réduire le zigzag ; ~0 en virage serré.
  `0.0` = désactivé (comportement CSV). Profil `finetuned` : `4.0`.
- **`lookahead_curv_soft`** (1/m, défaut `0.08`) : seuil de courbure au-delà
  duquel l'extra lookahead décroît. Plus haut → moins sensible au bruit κ
  de la spline. Profil `finetuned` : `0.10`.
- **`lookahead_curv_window`** (n. waypoints, défaut `30`) : fenêtre forward
  pour estimer la courbure peak utilisée par le scale.
- **Formule :** \(L_d \leftarrow L_d + \text{extra} \times \text{scale}(\kappa)\)
  avec `scale` ∈ [0.45, 1] (voir `lookahead_curvature_scale()`).

### 7.6 `path_tracker` — MPC

Pack `autocar_nav_mpc`. La pile MPC remplace Pure Pursuit par une
optimisation horizon glissant.

#### `mpc_horizon`
- **Défaut :** `18`.
- **Rôle :** nombre de pas de l'horizon de prédiction.
- **Effet :** ↑ = anticipation plus fine mais O(N²) côté CPU.

#### `q_ey`, `q_epsi`
- **Défauts :** `200.0`, `36.0`.
- **Rôle :** poids quadratiques sur l'erreur latérale (`ey`) et de cap
  (`epsi`).
- **Effet :** ratio `q_ey / q_epsi` ≈ 5 = on suit le path plus que le cap ;
  inversement, ratio < 2 produit un comportement très "rallye".

#### `r_delta`, `r_ddelta`
- **Défauts :** `0.04`, `0.5`.
- **Rôle :** poids sur le braquage et sa dérivée (lissage).
- **Effet :** `r_ddelta` ↑ → braquage plus doux mais réaction tardive.

#### `cruise_velocity`
- **Défaut :** `8.0`.
- **Rôle :** vitesse cible interne au MPC (redondante avec celle du
  `local_planner`).

#### `steering_rate_limit`
- **Défaut MPC :** `0.0` (illimité au tracker, le MPC gère via `r_ddelta`).
- **Recommandation :** garder à 0 sur MPC ; le terme `r_ddelta` régule déjà.

### 7.7 `global_planner_lidar` — LiDAR / SLAM

Pack `autocar_nav_pure_pursuit_lidar`. Remplace `global_planner` pour la
stack `pure_pursuit_lidar`. Nœud :
[`global_planner_lidar.py`](../src/AutoCarROS2/autocar_nav_pure_pursuit_lidar/nodes/global_planner_lidar.py).

**Cycle de vie :**
- **Tour 1 (exploration)** : goals générés depuis le LiDAR (`/scan`) et la
  centerline locale ; trajectoire enregistrée dans la carte SLAM.
- **Fin du tour 1** : construction asynchrone de la racing line (min-curvature
  + lissage énergétique) à partir de `/map` et du tracé exploré.
- **Tour 2+** : publication des waypoints racing sur `/autocar/goals`
  (`nav_mode ≥ 1`).

#### Waypoints et exploration

| Paramètre | Défaut | Rôle |
| --- | --- | --- |
| `waypoints_ahead`, `waypoints_behind` | `5`, `2` | Fenêtre de goals publiés autour de la position courante (identique au CSV). |
| `passed_threshold` | `0.25` m | Distance pour marquer un waypoint atteint. |
| `waypoint_search_ahead` | `30` | Fenêtre de recherche du waypoint le plus proche en avant. |
| `exploration_goal_count` | `10` | Nombre de goals générés en mode exploration. |
| `exploration_goal_step` | `3.0` m | Pas le long de la centerline locale entre goals successifs. |
| `cg_to_lidar` | `2.4` m | Décalage CG → capteur LiDAR pour la projection des scans. |

#### Extraction centerline (tour 1)

| Paramètre | Défaut | Rôle |
| --- | --- | --- |
| `centerline_step` | `3.5` m | Espacement des points centerline extraits du scan. |
| `centerline_close_dist` | `4.0` m | Distance de fermeture pour boucler la polyligne locale. |
| `centerline_min_points` | `20` | Nombre minimal de points avant publication. |
| `centerline_post_smooth_passes` | `3` | Passes de lissage post-extraction. |
| `centerline_refine_passes` | `3` | Passes de raffinement géométrique. |

#### Racing line (tour 2+, depuis `/map`)

| Paramètre | Défaut | Rôle |
| --- | --- | --- |
| `racing_use_map_corridor` | `true` | Contraint la ligne aux couloirs libres de la carte SLAM. |
| `racing_boundary_margin` | `1.0` m | Marge aux bords occupés de la carte. |
| `racing_track_half_width` | `8.0` m | Demi-largeur de piste pour le couloir (F1 clôturé). |
| `racing_mincurv_max_offset` | `5.0` m | Décalage latéral max de l'optimisation min-curvature. |
| `racing_mincurv_iters` | `8000` | Itérations de l'optimiseur min-curvature. |
| `racing_smooth_alpha` | `0.4` | Poids du lissage Laplacien (0 = conservateur, 1 = agressif). |
| `racing_smooth_iters` | `10` | Itérations du lissage principal. |
| `racing_smooth_max_dev` | `1.5` m | Écart max autorisé par rapport à la centerline explorée. |
| `racing_smooth_pre_iters` | `4` | Pré-lissage avant lissage énergétique. |
| `racing_smooth_pre_alpha` | `0.4` | Alpha du pré-lissage. |
| `racing_smooth_coarse_points` | `80` | Résolution grossière intermédiaire. |
| `racing_smooth_energy_ratio` | `0.05` | Ratio énergie de courbure vs fidélité au tracé exploré. |

**Recommandations F1 (`f1_pure_pursuit_lidar.yaml`) :**
- **`baseline`** : valeurs ci-dessus (`cruise_velocity: 6.0`, `curvature_lookahead: 140`).
- **`finetuned`** : `cruise 7.5`, `curvature_lookahead 110`, `lookahead_gain 0.48`,
  `lookahead_curv_extra 4.0`, `centerline_step 3.0`, `racing_mincurv_max_offset 4.5` —
  stable au tour 2+ sans zigzag à haute vitesse.
- **`finetuned_perturbed_*`** : grille 3×3 sur `latency_ms` ∈ {0, 200, 500} et
  `odom_noise_std` ∈ {0, 0.05, 0.1} (8 profils ; le coin 0/0 = `finetuned`).

---

## 8. Profils prêts à l'emploi

Chaque profil est un point de départ ; ajustez la cruise speed à votre
circuit. Tous supposent stack `pure_pursuit` + `line: racing` sauf mention.

### 8.1 Conservateur (`profile: conservateur`)
**But :** finir tous les tours, zéro collision, voiture lisible.

```yaml
- stack: pure_pursuit
  profile: conservateur
  line: centerline
  latency_ms: 0
  odom_noise_std: 0.0
  lap_count: 3
  warmup_laps: 1
  navigation:
    localisation: { ros__parameters: { update_frequency: 50.0 } }
    local_planner:
      ros__parameters:
        update_frequency: 10.0
        car_width: 2.0
        centreofgravity_to_frontaxle: 1.483
        frame_id: base_link
        cruise_velocity: 6.0       # vitesse réduite
        avoid_velocity: 6.0
        max_lateral_accel: 4.0     # marge de grip
        min_curvature: 0.012
        curvature_lookahead: 160   # freine tôt
        curvature_smooth_window: 9 # lisse les bruits
        accel_rate: 3.5
        decel_rate: 6.0
    global_planner:
      ros__parameters:
        update_frequency: 2.0
        waypoints_ahead: 5
        waypoints_behind: 2
        passed_threshold: 0.25
        centreofgravity_to_frontaxle: 1.483
        waypoint_search_ahead: 30
        waypoints_file: waypoints.csv
    path_tracker:
      ros__parameters:
        update_frequency: 50.0
        centreofgravity_to_frontaxle: 1.483
        centreofgravity_to_rearaxle: 1.483
        wheelbase: 2.966
        lookahead_gain: 0.45       # vise plus loin
        lookahead_min: 2.0         # pas de twitch à basse vitesse
        lookahead_max: 6.0
        closest_search_ahead: 130
        steering_limits: 0.85      # plafond conservateur
        steering_rate_limit: 3.0   # plus calme
        steer_smoothing: 0.85
        velocity_gain: 1.0
        startup_ramp_s: 3.0
        lateral_soft: 5.0          # accroche au path
        heading_soft: 0.5
```

Justifications :
- `cruise=6.0` + `max_lateral_accel=4.0` → ≈ 80 % du grip théorique, marge
  pour le bruit / la latence.
- `curvature_lookahead=160` et `decel_rate=6` → freinage anticipé sur épingle.
- `lookahead_min=2.0` + `steer_smoothing=0.85` → suppression des oscillations
  basses vitesse.
- `prolonged_collision_threshold_s` = défaut 120 s : si la voiture est bloquée
  plus de 2 min, l'expérience est marquée terminée.

### 8.2 Équilibré (`profile: equilibre`)
**But :** meilleur compromis temps au tour / robustesse. **C'est le profil
recommandé pour un benchmark.**

```yaml
- stack: pure_pursuit
  profile: equilibre
  line: racing
  latency_ms: 0
  odom_noise_std: 0.0
  lap_count: 3
  warmup_laps: 1
  navigation:
    localisation: { ros__parameters: { update_frequency: 50.0 } }
    local_planner:
      ros__parameters:
        update_frequency: 10.0
        car_width: 2.0
        centreofgravity_to_frontaxle: 1.483
        frame_id: base_link
        cruise_velocity: 8.0
        avoid_velocity: 8.0
        max_lateral_accel: 5.5
        min_curvature: 0.012
        curvature_lookahead: 140
        curvature_smooth_window: 5
        accel_rate: 5.0
        decel_rate: 7.0
    global_planner:
      ros__parameters:
        update_frequency: 2.0
        waypoints_ahead: 5
        waypoints_behind: 2
        passed_threshold: 0.25
        centreofgravity_to_frontaxle: 1.483
        waypoint_search_ahead: 30
        waypoints_file: waypoints.csv
    path_tracker:
      ros__parameters:
        update_frequency: 50.0
        centreofgravity_to_frontaxle: 1.483
        centreofgravity_to_rearaxle: 1.483
        wheelbase: 2.966
        lookahead_gain: 0.4
        lookahead_min: 1.5
        lookahead_max: 6.0
        closest_search_ahead: 120
        steering_limits: 0.95
        steering_rate_limit: 4.0
        steer_smoothing: 1.0
        velocity_gain: 1.0
        startup_ramp_s: 2.0
        lateral_soft: 4.0
        heading_soft: 0.6
```

C'est la **baseline** utilisée dans tous les benchmarks R5 ; partez de là
pour tuner.

### 8.3 Agressif (`profile: agressif`)
**But :** minimiser le temps au tour, quitte à friser la sortie de piste.

```yaml
- stack: pure_pursuit
  profile: agressif
  line: racing
  latency_ms: 0
  odom_noise_std: 0.0
  lap_count: 5
  warmup_laps: 2
  navigation:
    localisation: { ros__parameters: { update_frequency: 50.0 } }
    local_planner:
      ros__parameters:
        update_frequency: 10.0
        car_width: 2.0
        centreofgravity_to_frontaxle: 1.483
        frame_id: base_link
        cruise_velocity: 9.5       # vise le plafond grip
        avoid_velocity: 9.5
        max_lateral_accel: 6.0     # exploite toute l'adhérence
        min_curvature: 0.010
        curvature_lookahead: 120   # freine plus tard
        curvature_smooth_window: 5
        accel_rate: 6.0
        decel_rate: 8.0            # freinage plus court
    global_planner:
      ros__parameters:
        update_frequency: 2.5
        waypoints_ahead: 6
        waypoints_behind: 2
        passed_threshold: 0.25
        centreofgravity_to_frontaxle: 1.483
        waypoint_search_ahead: 35
        waypoints_file: waypoints.csv
    path_tracker:
      ros__parameters:
        update_frequency: 50.0
        centreofgravity_to_frontaxle: 1.483
        centreofgravity_to_rearaxle: 1.483
        wheelbase: 2.966
        lookahead_gain: 0.33       # nerveux
        lookahead_min: 1.2
        lookahead_max: 5.5
        closest_search_ahead: 100
        steering_limits: 0.95
        steering_rate_limit: 5.0
        steer_smoothing: 1.0
        velocity_gain: 1.0
        startup_ramp_s: 1.5
        lateral_soft: 3.0          # plus tolérant à l'écart
        heading_soft: 0.85         # mais cap précis
```

Justifications :
- `cruise=9.5` + `max_lateral_accel=6` → on s'approche du plafond
  \(v_{max} = \sqrt{6 \cdot 96} ≈ 24 m/s\) mais reste sain.
- `curvature_lookahead=120` et `decel_rate=8` → freinage tardif et court.
- `lookahead_gain=0.33` + `lookahead_min=1.2` → suit la racing line au plus près.
- ⚠️ Risque accru de NaN si `steering_rate_limit` était mis à 0 — gardez ≥ 3.
- **Surveiller :** `lateral_error_max`, `collision_events`, `offtrack_events`
  dans `lap_times.csv`.

### 8.4 Circuit serré (`profile: f1_baseline`)
Voir [`scripts/configs/f1_circuit.yaml`](../scripts/configs/f1_circuit.yaml).
Spécificités : `lookahead_gain=0.3`, `lookahead_min=1.0`, `track:
f1_circuit` (charge `waypoints_f1_racing.csv` et le bon `.world`).

### 8.5 Robustesse latence (`profile: latency_robust`)
Variante de `equilibre` testée sous `latency_ms: 300`.

```yaml
- stack: pure_pursuit
  profile: latency_robust
  line: racing
  latency_ms: 300
  odom_noise_std: 0.0
  lap_count: 3
  warmup_laps: 1
  navigation:
    # ... bloc équilibré ci-dessus, avec ces deltas :
    path_tracker:
      ros__parameters:
        lookahead_gain: 0.5        # vise plus loin pour compenser
        lookahead_min: 1.8
        steer_smoothing: 0.9
        # le reste comme equilibre
```

### 8.6 Robustesse bruit odométrie (`profile: noisy_odom`)
Variante de `conservateur` avec `odom_noise_std: 0.05`. Augmenter
`curvature_smooth_window` à 11 et `lateral_soft` à 6 pour absorber le bruit.

### 8.7 Circuit F1 LiDAR / SLAM (`stack: pure_pursuit_lidar`)

Profils définis dans
[`scripts/configs/f1_pure_pursuit_lidar.yaml`](../scripts/configs/f1_pure_pursuit_lidar.yaml).
Référence pack :
[`autocar_nav_pure_pursuit_lidar/config/navigation_params.yaml`](../src/AutoCarROS2/autocar_nav_pure_pursuit_lidar/config/navigation_params.yaml).

**Prérequis :** `lap_count ≥ 2`, `warmup_laps: 1`, `track: f1_circuit_fenced`,
`line: racing`, `camera: false` (benchmark headless).

| Profil | Perturbations | Navigation (résumé) |
| --- | --- | --- |
| `baseline` | `latency_ms=0`, `odom_noise_std=0` | Défaut pack : cruise 6 m/s, PP standard, pas de `lookahead_curv_extra`. |
| `finetuned` | idem | Cruise 7.5 m/s, `lookahead_gain 0.48`, `lookahead_curv_extra 4.0`, courbure / racing-line tuning (voir § 7.7). |
| `finetuned_perturbed_l{L}_n{N}` | L ∈ {0, 200, 500}, N ∈ {0, 005, 01} | Même nav que `finetuned` ; seuls `latency_ms` et `odom_noise_std` varient. |

Lancement :

```bash
python3 scripts/benchmark.py --config scripts/configs/f1_pure_pursuit_lidar.yaml
```

Le harness écrit le bloc `navigation:` inline vers
`results/benchmark_<ts>/nav_overrides/run_*.yaml` et le passe en
`nav_config:=` à `race_pure_pursuit_lidar_launch.py`.

**Interprétation des tours :**
- Tour 1 : exploration SLAM (exclu de `summary.csv` via `warmup_laps: 1`).
- Tour 2 : premier tour chronométré sur la racing line extraite de la carte.

---

## 9. Workflow de tuning

### 9.1 Démarrer
1. Vérifier le build : `colcon build && source install/setup.bash`.
2. Lancer la baseline équilibrée et vérifier qu'on boucle :
   `ros2 launch launches race_launch.py control_mode:=auto`.
3. Inspecter `results/<run>/lap_times.csv` : valeurs sensées
   (`duration_s` ~80–200 s, `lateral_error_max` < 4 m, `collision_events == 0`).

### 9.2 Itérer sur un paramètre à la fois
Comparer `profile=A` vs `profile=B` via `benchmark.py` (un seul YAML, deux
entrées). Toujours figer `latency_ms` et `odom_noise_std` pendant qu'on
ajuste un paramètre de tracker.

### 9.3 Comprendre les métriques
- `lateral_error_max > 3 m` → la voiture coupe les virages : baisser
  `lookahead_gain` ou monter `lateral_soft`.
- `steering_rate_max ≈ steering_rate_limit` (égalité parfaite) → le limiteur
  sature → augmenter le cap ou réduire `control_gain`/`lookahead_gain`.
- `collision_events > 0` mais `collision_total_s < 5 s` → frôlements sur
  bordures : élargir `track_boundary_margin` ou baisser `cruise_velocity`.
- `experiment_terminated == 1` → bloqué > 2 min : revoir l'évitement,
  augmenter `prolonged_collision_threshold_s` ou changer de ligne.

### 9.4 Anti-oscillation
Si la voiture oscille en mode `auto` :
1. Vérifier dans `lap_times.csv` : `steering_rate_max` égal à la limite
   `steering_rate_limit` du tracker ou `max_steer_rate_radps` du
   `control_manager` ?
2. Le `control_manager` ne *rate-limite plus* en mode `auto` depuis le
   correctif récent — il ne reste donc que `steering_rate_limit` côté
   tracker (PP) et les gains internes.
3. Côté Stanley : baisser `control_gain` de 1.0 → 0.7.
4. Côté Pure Pursuit : monter `lookahead_min` (1.5 → 2.0) ou baisser
   `steer_smoothing` (1.0 → 0.85).
5. Si l'oscillation persiste avec `latency_ms = 0`, c'est un problème de
   gain ; sinon, c'est la latence — voir [`r4_latency_sweep.yaml`](../scripts/configs/r4_latency_sweep.yaml).

### 9.5 Anti-NaN
Voir [`docs/nan_crash_root_cause.md`](nan_crash_root_cause.md). En résumé :
toujours garder `steering_rate_limit ≥ 2.0` rad/s sur les pistes serrées et
ne pas désactiver l'`obstacle_avoidance` sans vérifier que `/map` ne marque
pas les murs comme occupés.

---

## Voir aussi

- [`README.md`](../README.md) — vue d'ensemble du projet.
- [`docs/REPORT_BASELINE.md`](REPORT_BASELINE.md) — baseline Stanley référence.
- [`docs/REPORT_PURE_PURSUIT.md`](REPORT_PURE_PURSUIT.md) — résultats Pure Pursuit.
- [`docs/REPORT_MPC.md`](REPORT_MPC.md) — résultats MPC.
- [`scripts/configs/README.md`](../scripts/configs/README.md) — détail des
  batches de benchmark R1–R5.
- [`scripts/configs/f1_pure_pursuit_lidar.yaml`](../scripts/configs/f1_pure_pursuit_lidar.yaml)
  — baseline / finetuned / grille perturbations F1 LiDAR.
- [`results/`](../results/) — runs historiques (un dossier `params.yaml` +
  `lap_times.csv` + `collisions.csv` par run).
