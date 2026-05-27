# High-Speed Autonomous Racing - SONIC ROS 2

Projet D : développement d'un robot de course autonome capable de terminer des tours de piste le plus rapidement possible sans collision avec les limites du circuit. Le projet s'appuie sur [AutoCarROS2](https://github.com/winstxnhdw/AutoCarROS2), un environnement ROS 2 + Gazebo pour véhicule non holonome avec modèle Ackermann. Le code tourne sur **ROS 2 Humble** (l'upstream AutoCarROS2 visait Foxy ; le portage Humble est transparent côté APIs utilisées).

L'objectif est de dépasser une navigation conservatrice classique en ajoutant une logique de course : génération d'une racing line, contrôle latéral/longitudinal agressif, mesure des temps au tour et analyse de la robustesse face à la latence et au bruit d'odométrie.

## Objectifs

- Simuler un véhicule Ackermann sur circuit avec ROS 2 Humble et Gazebo 11.
- Implémenter un contrôleur haute vitesse, sous forme de plugin Nav2 ou de noeud ROS 2 autonome.
- Comparer plusieurs réglages de conduite : conservateur, équilibré et agressif.
- Calculer et suivre une racing line plutôt qu'une simple ligne centrale.
- Mesurer l'impact de la latence perception-commande et du drift d'odométrie sur les temps au tour.

## Architecture Cible

```text
Gazebo / circuit
    |
    | /autocar/state2D, /odom, capteurs simulés
    v
Localisation + estimation d'état
    |
    | état filtré, vitesse, yaw rate
    v
Générateur de trajectoire
    |
    | centreline -> racing line -> profil de vitesse
    v
Contrôleur haute vitesse
    |
    | Pure Pursuit, Stanley amélioré ou MPC
    v
Arbitre de contrôle
    |
    | auto / semi-auto / manuel
    v
/autocar/cmd_vel
    |
    v
Plugin Ackermann Gazebo
```

Le dossier `src/AutoCarROS2` (code source intégré depuis [AutoCarROS2](https://github.com/winstxnhdw/AutoCarROS2)) fournit déjà les briques de base :

| Package | Rôle |
| --- | --- |
| `launches` | Points d'entrée ROS 2 (`default_launch.py`, `click_launch.py`) |
| `autocar_gazebo` | Modèle du véhicule, mondes Gazebo et plugin Ackermann |
| `autocar_description` | URDF/Xacro et configuration RViz |
| `autocar_nav` | Localisation, planification et contrôleur Stanley existant |
| `autocar_msgs` | Messages personnalisés (`State2D`, `Path2D`, `Twist2D`) |
| `autocar_map` | Occupancy grid / Bayesian Occupancy Filter |

## Plan D'implémentation

1. Baseline fonctionnelle
   - Lancer la simulation AutoCarROS2.
   - Valider l'arbitre de commande (`/autocar/manual_cmd_vel`, `/autocar/auto_cmd_vel`, `/autocar/cmd_vel`).
   - Mesurer un premier temps au tour avec le contrôleur existant.

2. Instrumentation de course
   - Ajouter un noeud de chronométrage des tours.
   - Publier les métriques : temps au tour, vitesse moyenne, erreur latérale, collisions ou sorties de piste.
   - Sauvegarder les résultats en CSV pour comparaison.

3. Racing line
   - Extraire ou définir les limites du circuit.
   - Construire une centerline puis optimiser une trajectoire plus rapide.
   - Associer un profil de vitesse en fonction de la courbure et des limites d'adhérence.

4. Contrôle haute vitesse
   - Implémenter une première version Pure Pursuit avec lookahead dynamique.
   - Ajouter des limites de vitesse, d'accélération et d'angle de braquage.
   - Évaluer une version MPC si le Pure Pursuit atteint ses limites à haute vitesse.

5. Gestion de la latence
   - Timestamp des messages d'état et de commande.
   - Compensation simple par prédiction d'état à horizon court.
   - Comparaison sans compensation, avec compensation, puis avec latence artificielle.

6. Étude du drift d'odométrie
   - Ajouter un noeud injectant du bruit sur l'odométrie ou `State2D`.
   - Tester plusieurs niveaux de bruit.
   - Tracer la dégradation des temps au tour et de l'erreur latérale.

## Roadmap

| Étape | Résultat attendu | Statut |
| --- | --- | --- |
| 1 | Workspace ROS 2 compilé et simulation lancée | À faire |
| 2 | Baseline avec contrôleur existant et temps au tour de référence | À faire |
| 3 | Noeud de métriques + export CSV | À faire |
| 4 | Racing line et profil de vitesse | À faire |
| 5 | Contrôleur Pure Pursuit haute vitesse | À faire |
| 6 | Tuning conservateur/agressif et comparaison des temps | À faire |
| 7 | Compensation de latence | À faire |
| 8 | Injection de bruit d'odométrie + graphiques | Bonus |
| 9 | Variante MPC ou plugin Nav2 | Bonus |

## Installation

### Prérequis

- Ubuntu 22.04 (WSL2 ou natif) pour ROS 2 Humble.
- ROS 2 Humble Hawksbill (installé dans `/opt/ros/humble`).
- Gazebo 11 (Gazebo Classic).
- `git`, `python3-pip`, `colcon`, `rosdep`.

Sur WSL2, il faut aussi un affichage graphique fonctionnel pour Gazebo/RViz, par exemple WSLg sous Windows 11 ou un serveur X configuré.

### Cloner le projet

```bash
git clone <url-du-depot> autocar
cd autocar
```

Le code AutoCarROS2 est versionné directement dans `src/AutoCarROS2` ; aucune initialisation de sous-module n'est nécessaire.

### Démarrage via conteneur ROS 2 Foxy (optionnel, legacy)

> Note : le développement principal se fait en local sur ROS 2 Humble. Le conteneur Foxy ci-dessous a été utile pour valider le portage initial d'AutoCarROS2 mais n'est plus le chemin recommandé pour les expériences de racing.

Le dépôt fournit un conteneur ROS 2 Foxy Fitzroy + Gazebo 11 qui expose :

- le bureau XFCE via noVNC : `http://localhost:6080`
- l'API HTTP de contrôle : `http://localhost:8001/api/health`

Depuis la racine du dépôt :

```bash
docker compose up --build
```

Puis ouvrir le bureau dans le navigateur :

```text
http://localhost:6080/vnc.html
```

Lancer la simulation autonome par API :

```bash
curl -X POST http://localhost:8001/api/sim/start \
  -H "Content-Type: application/json" \
  -d '{ "mode": "default" }'
```

Modes disponibles :

| Mode | Usage |
| --- | --- |
| `default` | Lance `ros2 launch launches default_launch.py` avec la pile de navigation existante |
| `click` | Lance `click_launch.py` et accepte des goals sur `/goal_pose` |
| `gazebo` | Lance Gazebo seul, pratique pour valider le plugin Ackermann |
| `race` | Lance `race_launch.py` avec les modes `manual`, `semi` ou `auto` |
| `race_mpc` | Lance `race_mpc_launch.py` avec le même arbitre de contrôle |
| `race_pure_pursuit` | Lance `race_pure_pursuit_launch.py` avec le même arbitre de contrôle |

Exemples API :

```bash
curl http://localhost:8001/api/status

curl -X POST http://localhost:8001/api/command/manual \
  -H "Content-Type: application/json" \
  -d '{ "linear_x": 2.0, "angular_z": 0.2, "duration_sec": 3 }'

curl -X POST http://localhost:8001/api/control/mode \
  -H "Content-Type: application/json" \
  -d '{ "mode": "semi" }'

curl -X POST http://localhost:8001/api/control/stop
curl -X POST http://localhost:8001/api/control/resume
curl http://localhost:8001/api/control/status

curl -X POST http://localhost:8001/api/navigation/goal \
  -H "Content-Type: application/json" \
  -d '{ "x": 10.0, "y": 2.0, "yaw": 0.0 }'

curl -X POST http://localhost:8001/api/sim/stop
```

Echange de fichiers :

```bash
curl -F "file=@mission.json" http://localhost:8001/api/files/upload
curl http://localhost:8001/api/files
curl http://localhost:8001/api/files/mission.json
```

Les fichiers échangés sont montés dans `shared/`. Les logs du bureau, noVNC, API et simulation sont montés dans `runtime/logs/`.

### Installation locale AutoCarROS2

Depuis la racine du workspace :

```bash
cd src/AutoCarROS2
sh ros-foxy-desktop-full-install.sh
```

> Le script porte "foxy" dans son nom pour des raisons historiques (upstream AutoCarROS2). Sur une machine Humble, suis plutôt la procédure officielle ROS 2 Humble (`apt install ros-humble-desktop-full`) puis le script `requirements.sh` ci-dessous.

Si ROS 2 (Foxy ou Humble) est déjà installé, installer uniquement les dépendances du projet :

```bash
cd src/AutoCarROS2
sh requirements.sh
```

Puis revenir à la racine du workspace :

```bash
cd ../..
```

### Compiler

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Pour charger automatiquement l'environnement du workspace :

```bash
echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Lancer La Simulation

Lancement complet avec waypoints par défaut :

```bash
ros2 launch launches default_launch.py
```

Lancement course interactif avec interface stable :

```bash
ros2 launch launches race_launch.py control_mode:=auto gui:=true rviz:=true
```

Modes de contrôle disponibles :

```bash
# Le véhicule reste immobile tant qu'aucune commande manuelle n'est publiée.
ros2 launch launches race_launch.py control_mode:=manual gui:=true rviz:=true

# L'autonomie reste active, avec override utilisateur temporaire.
ros2 launch launches race_launch.py control_mode:=semi gui:=true rviz:=true

# Environnement fragile/container : Gazebo serveur + RViz seulement.
ros2 launch launches race_launch.py control_mode:=auto gui:=false rviz:=true
```

Commande manuelle via l'arbitre de contrôle :

```bash
ros2 topic pub -r 10 /autocar/manual_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 0.2}}"
```

Changer de mode ou arrêter/reprendre :

```bash
ros2 topic pub --once /autocar/control_mode std_msgs/msg/String "{data: 'manual'}"
ros2 topic pub --once /autocar/stop std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /autocar/resume_auto std_msgs/msg/Bool "{data: true}"
```

RViz affiche la trajectoire cible, le point de suivi, le mode courant, la vitesse,
l'erreur latérale et l'état de collision/reprise. La vue par défaut suit
`base_link`, avec des vues sauvegardées `Follow Behind`, `Top Down` et
`Free Overview`.

La caméra arrière Gazebo publie son flux sur :

```bash
ros2 topic echo --once /autocar/third_person_camera/image_raw --field header
```

Dans RViz, le display `Third Person Camera` est branché sur
`/autocar/third_person_camera/image_raw`. Dans Gazebo, le cône bleu indique
seulement le champ de vue de la caméra, pas l'image vidéo rendue.

Le circuit de course contient aussi des garde-corps physiques sur les bords
intérieur et extérieur. Le noeud `control_manager` signale également une sortie
de piste logique via `/autocar/collision` et `/autocar/control_status`.

## Interface de contrôle desktop

Le package `autocar_gui` fournit une fenêtre PySide6 minimaliste pour piloter
la simulation et afficher le flux caméra arrière sans passer par RViz.

### Prérequis

```bash
pip install -r src/AutoCarROS2/autocar_gui/requirements.txt
colcon build --packages-select autocar_gui
source install/setup.bash
```

### Lancement

Terminal 1 — simulation :

```bash
ros2 launch launches race_launch.py control_mode:=semi gui:=true rviz:=true
```

Terminal 2 — panel desktop :

```bash
ros2 run autocar_gui control_panel.py
```

Variantes :

```bash
# Forcer le backend ROS (flux caméra + topics locaux)
ros2 run autocar_gui control_panel.py --backend ros

# Repli HTTP via l'API container (contrôle OK, pas de caméra)
ros2 run autocar_gui control_panel.py --backend http --api-url http://localhost:8001

# Via launch file
ros2 launch launches control_panel_launch.py backend:=auto api_url:=http://localhost:8001
```

### Architecture topics

| Direction | Topic | Type | Rôle |
| --- | --- | --- | --- |
| Entrée UI | `/autocar/third_person_camera/image_raw` | `sensor_msgs/Image` | Flux vidéo (ROS uniquement) |
| Entrée UI | `/autocar/control_status` | `std_msgs/String` (JSON) | Mode, vitesse, collision, commande |
| Entrée UI | `/autocar/state2D` | `autocar_msgs/State2D` | État véhicule (secours vitesse) |
| Entrée UI | `/autocar/lateral_error` | `std_msgs/Float64` | Erreur latérale |
| Sortie UI | `/autocar/manual_cmd_vel` | `geometry_msgs/Twist` | Commande manuelle (10 Hz) |
| Sortie UI | `/autocar/control_mode` | `std_msgs/String` | Boutons manual / semi / auto |
| Sortie UI | `/autocar/stop` | `std_msgs/Bool` | Arrêt latched |
| Sortie UI | `/autocar/resume_auto` | `std_msgs/Bool` | Reprise autonomie |

En mode HTTP, le panel interroge l'API container (`GET /api/control/status`,
`POST /api/control/*`, `POST /api/command/manual`) documentée plus haut.
Le flux caméra reste disponible uniquement via ROS (même domaine DDS que la
simulation).

## Expériences Attendues

Les comparaisons de performance seront organisées autour de trois profils de paramètres :

| Profil | But | Exemples de paramètres |
| --- | --- | --- |
| Conservateur | Finir le tour de façon stable | vitesse limitée, lookahead élevé, gains faibles |
| Équilibré | Bon compromis vitesse/stabilité | vitesse variable selon courbure, gains modérés |
| Agressif | Minimiser le temps au tour | vitesse élevée, lookahead court, forte anticipation |

Métriques à produire :

- Temps au tour.
- Vitesse moyenne et maximale.
- Erreur latérale moyenne/maximale.
- Nombre de collisions ou sorties de piste.
- Impact du bruit d'odométrie sur le temps au tour.

## Dépannage

Si `ros2 launch` ne trouve pas les packages, recharger l'environnement :

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Si la compilation ne prend pas en compte certains changements :

```bash
rm -rf build install log
colcon build
source install/setup.bash
```

Si Gazebo charge le monde mais que l'interface graphique plante, tester le serveur Gazebo seul ou lancer directement le monde :

```bash
gazebo install/autocar_gazebo/share/autocar_gazebo/worlds/autocar.world
```

## Baseline

Premier temps au tour de référence avec le tracker Stanley existant : **3 min 11 s** (190.9 s sur ~650 m, vitesse moyenne 3.4 m/s, pic 5.85 m/s). Détails et reproductibilité dans [`docs/REPORT_BASELINE.md`](docs/REPORT_BASELINE.md).

Pour reproduire :

```bash
ros2 launch launches race_launch.py
```

Chaque tour complété est enregistré sous **`results/<stack>_<run_id>/lap_times.csv`** avec les paramètres dans **`params.yaml`** (voir [`results/README.md`](results/README.md)). Comparaison manuelle via les `REPORT_*.md` ou les CSV.


# 1) Tuer ABSOLUMENT tout
pkill -9 -f "ros2 launch" 2>/dev/null
pkill -9 -f tracker.py 2>/dev/null
pkill -9 -f localplanner.py 2>/dev/null
pkill -9 -f globalplanner.py 2>/dev/null
pkill -9 -f localisation.py 2>/dev/null
pkill -9 -f lap_timer.py 2>/dev/null
pkill -9 -f latency_injector.py 2>/dev/null
pkill -9 -f control_manager.py 2>/dev/null
killall -9 gzserver gzclient gazebo 2>/dev/null
killall -9 robot_state_publisher rviz2 2>/dev/null
ros2 daemon stop
sleep 3


pkill -9 -f bof 2>/dev/null
killall -9 bof 2>/dev/null
pkill -9 -f odom_noise 2>/dev/null
ros2 daemon stop
sleep 2
ros2 daemon start
ros2 node list