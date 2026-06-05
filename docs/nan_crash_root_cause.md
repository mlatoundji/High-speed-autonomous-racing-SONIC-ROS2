# Crash NaN / reset à l'origine — analyse de cause racine

> Statut : **cause racine confirmée par le code**. Correctif : paramètre
> `obstacle_avoidance`, **défaut `false`** (évitement désactivé = suivi de racing
> line). Mettre `true` pour réactiver l'évitement.

## Symptôme

En simulation (Gazebo), la voiture roule un moment puis **se téléporte à
l'origine** (0,0,0) et reste figée. Le benchmark (`scripts/benchmark.py`) pend
alors indéfiniment sur le run mort.

Observé surtout sur machine **CPU‑limitée** (RTF < 1, ici un i7‑1165G7,
4 cœurs). Le collègue, sur une machine plus rapide (RTF ≈ 1), le déclenche
rarement — d'où le « ça marche chez lui, pas chez moi ».

## Ce qui se passe vraiment

1. **`odom` part en NaN.** Le plugin Gazebo `libgazebo_ros_ackermann_drive`
   publie une vitesse (`/autocar/odom.twist`) = `NaN`. Comme la pose du modèle
   devient NaN, **Gazebo remet le modèle à l'origine**. L'odométrie n'est que le
   *messager* : le NaN naît dans la **physique** (drive Ackermann), pas dans
   notre code de contrôle (`cmd_vel` reste fini — vérifié).

2. **Le déclencheur est une déviation latérale brutale.** Juste avant le 1er
   NaN, le log montre systématiquement :

   ```
   ... waypoint #9..#12 : "All lateral offsets blocked -- slowing to crawl"  (répété)
   wp #12 passé  ->  "Path blocked, deviating by -4.5 m"   <-- écart latéral BRUTAL
   wp #13        ->  odom twist non-finite                  <-- crash
   ```

   La déviation soudaine de **−4,5 m** produit un ordre de braquage sec, qui sur
   le **PID de direction raide** (`4000 0 1` dans `race_circuit.world`) et un
   **CPU sans marge** fait diverger la vitesse du joint → NaN.

## Cause racine (le « pourquoi » amont)

Le **local planner** (`autocar_nav_pure_pursuit/nodes/localplanner.py`) fait de
l'**évitement d'obstacles** : il essaie des décalages latéraux
`[0, ±1.5, ±3, ±4.5, ±6]` et rejette tout décalage dont la trajectoire (7
waypoints en avant) traverse une case **occupée** de la carte `/map`.

Or `/map` est produite par le nœud `bof` (`autocar_map/src/bof.cpp`)
**directement à partir du LiDAR** :

```cpp
// pour chaque rayon LiDAR, au-delà du point touché (r > R) :
gmap.setGridOcc(px, py);   // -> OCCUPÉ
```

→ **les murs de la piste sont marqués comme obstacles occupés.**

Conséquence en virage : la trajectoire en avant, **quel que soit l'offset**,
finit par croiser un mur → `path_is_blocked = True` pour **tous** les offsets →
**« all blocked » → crawl**. En sortie de virage, un grand offset (−4,5 m)
redevient libre d'un coup → **déviation brutale → braquage sec → NaN**.

**En une phrase :** le local planner *esquive les murs statiques de la piste*
(que le LiDAR/`bof` cartographie comme obstacles). Pour un suivi de racing line
**sans obstacle dynamique**, c'est inutile **et** c'est la cause amont du crash.
Le PID raide + le CPU lent ne font que transformer la conséquence (déviation
brutale) en NaN.

## Pourquoi ça crashe chez moi et pas chez mon collègue (même code)

Le collègue tourne le **même code** (main, `obstacle_avoidance` activé) : le
déclencheur — « all blocked » puis déviation brutale de −4,5 m — **se produit
aussi chez lui**. La différence n'est pas *si* la déviation arrive, mais *si elle
fait basculer la physique en NaN*. C'est une **instabilité limite**, sensible au
**timing**.

Point clé : **Gazebo et les nœuds ROS de contrôle sont des process séparés et
asynchrones.** Gazebo n'attend pas le stack — il avance à son rythme ; le local
planner (10 Hz) et le tracker (50 Hz) consomment l'odom et renvoient `cmd_vel`
*quand ils peuvent*.

- **Machine rapide (collègue, RTF ≈ 1)** : le contrôle reste collé à la simu. La
  déviation −4,5 m est absorbée **proprement sur plusieurs cycles** → le PID
  raide n'a jamais une grosse erreur d'un coup → pas de slam → **stable**.
- **Machine lente (CPU saturé, RTF < 1)** : le contrôle **prend du retard**. La
  voiture roule « en aveugle » plus longtemps → la correction arrive **tardive
  et brutale** → le PID raide slamme la direction → vitesse de joint diverge →
  **NaN**.

En image : chez lui un **coup de volant progressif**, chez toi un **à‑coup sec**
parce que le contrôle a pris du retard. Même braquage cible, appliqué
brutalement, fait exploser le solveur.

**En une phrase :** même code, même déclencheur — mais instabilité au bord du
gouffre. Sa machine la maintient du bon côté (contrôle synchrone → corrections
douces) ; une machine sous‑dimensionnée la fait basculer (contrôle en retard →
corrections brutales). Le fix (`obstacle_avoidance: false`) **supprime la
déviation brutale elle‑même**, donc il n'y a plus rien d'abrupt à encaisser,
même quand le contrôle prend du retard → pas de NaN.

## Ce que ce n'est PAS

Écartés, preuves à l'appui (diffs, logs, mesures) :

- ❌ Un paramètre de tuning du YAML (diff config = identique à la config qui
  roulait).
- ❌ Le RTF en soi (le crash s'est aussi produit à RTF 1.0).
- ❌ La caméra / le GUI (aident la charge CPU, mais le crash persiste).
- ❌ Notre code de contrôle (`cmd_vel` toujours fini ; garde‑fous NaN OK).
- ❌ La version de Gazebo (11.10.2 standard).

## Correctif appliqué

1. **Paramètre `obstacle_avoidance`** dans `localplanner.py`, **défaut `False`**.
   Quand `False`, `path_is_blocked` renvoie `False` → la voiture suit toujours la
   racing line (offset 0) → plus de crawl, plus de déviation brutale → **le
   déclencheur du NaN disparaît**. Le défaut étant `false`, **toutes les configs
   en bénéficient automatiquement** (pas besoin de le mettre par config). Mettre
   `obstacle_avoidance: true` dans le bloc `local_planner` pour réactiver
   l'évitement sur une config donnée.

### Le LiDAR devient‑il inutile ?

Seulement **dans ce benchmark** (piste vide → rien à éviter). La capacité
d'évitement reste présente (`obstacle_avoidance: true`). Pour qu'elle soit
*vraiment* utile, il faudrait que la `/map` **distingue les murs (statiques, à
longer) des vrais obstacles (dynamiques, à éviter)** — p.ex. précharger une
carte statique de la piste et ne marquer comme obstacles que les retours LiDAR
nouveaux. C'est un chantier séparé.

## Garde‑fous complémentaires (déjà en place)

- **Anti‑NaN** dans `localisation.py` (rejette un odom NaN au lieu de le
  propager) et `tracker.py` (ne publie jamais un `cmd_vel` NaN → arrêt de
  sécurité). Ils empêchent la *propagation* du NaN, mais ne peuvent pas empêcher
  Gazebo de resetter son propre modèle — d'où l'importance de supprimer le
  *déclencheur* en amont (ci‑dessus).
- **Toggles `camera` / `gui`** dans le launch + YAML pour alléger le CPU
  (headless = RTF ~0.72 vs ~0.35 avec GUI).
