# High-Speed Autonomous Racing - SONIC ROS 2

Projet D : développement d'un robot de course autonome capable de terminer des tours de piste le plus rapidement possible sans collision avec les limites du circuit. Le projet s'appuie sur [AutoCarROS2](https://github.com/winstxnhdw/AutoCarROS2), un environnement ROS 2 Foxy + Gazebo pour véhicule non holonome avec modèle Ackermann.

L'objectif est de dépasser une navigation conservatrice classique en ajoutant une logique de course : génération d'une racing line, contrôle latéral/longitudinal agressif, mesure des temps au tour et analyse de la robustesse face à la latence et au bruit d'odométrie.

## Objectifs

- Simuler un véhicule Ackermann sur circuit avec ROS 2 Foxy et Gazebo 11.
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
   - Valider le topic de commande `/autocar/cmd_vel`.
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

- Ubuntu 20.04 recommandé pour ROS 2 Foxy.
- ROS 2 Foxy Fitzroy.
- Gazebo 11.
- `git`, `python3-pip`, `colcon`, `rosdep`.

Sur WSL2, il faut aussi un affichage graphique fonctionnel pour Gazebo/RViz, par exemple WSLg sous Windows 11 ou un serveur X configuré.

### Cloner le projet

```bash
git clone <url-du-depot> autocar
cd autocar
```

Le code AutoCarROS2 est versionné directement dans `src/AutoCarROS2` ; aucune initialisation de sous-module n'est nécessaire.

### Démarrage via conteneur ROS 2 Foxy

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
| `gazebo` | Lance Gazebo seul, pratique pour les commandes manuelles sur `/autocar/cmd_vel` |

Exemples API :

```bash
curl http://localhost:8001/api/status

curl -X POST http://localhost:8001/api/command/manual \
  -H "Content-Type: application/json" \
  -d '{ "linear_x": 2.0, "angular_z": 0.2, "duration_sec": 3 }'

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

Si ROS 2 Foxy est déjà installé, installer uniquement les dépendances du projet :

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
source /opt/ros/foxy/setup.bash
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

Lancement interactif :

```bash
ros2 launch launches click_launch.py
```

Commande manuelle utile pour valider le plugin Ackermann :

```bash
ros2 topic pub -r 10 /autocar/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 0.2}}"
```

Arrêt du véhicule :

```bash
ros2 topic pub --once /autocar/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

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
source /opt/ros/foxy/setup.bash
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

Chaque tour complété est enregistré sous **`docs/lap_times/lap_times_<stack>.csv`** (voir [`docs/README.md`](docs/README.md)). Comparaison manuelle via les `REPORT_*.md` ou les CSV.
