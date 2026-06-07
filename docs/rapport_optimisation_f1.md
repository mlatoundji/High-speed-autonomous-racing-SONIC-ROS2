# Rapport — Optimisation du circuit F1 (pure pursuit)

Synthèse d'une session de tuning du circuit F1 (Albert Park) avec le stack
pure pursuit. Objectif : réduire le temps au tour. Méthode : **piloter par la
donnée** (métriques + validation hors-ligne avant tout déploiement), pas à l'œil.

Lancement de référence :
```
python3 scripts/benchmark.py --config scripts/configs/f1_circuit.yaml
python3 scripts/diag_metrics.py --duration 180   # métriques (2e terminal)
```

---

## Résultat

| Étape | Temps au tour (stabilisé) | Commit |
|---|---|---|
| Départ de session | 94.9 s (0 offtrack) | `2720531` |
| **Fin de session** | **89.7 s (0 offtrack)** | **`4a04ac4`** |

**Gain net : −5.2 s (−5.5 %)**, et oscillation de braquage **−58 %**
(`steer_rate_max` 8.16 → 3.44), suivi amélioré (`lat_max` 2.59 → 1.32 m).

> Note sur la mesure : le **tour 1 ne compte pas** comme référence — la voiture
> part du spawn (hors ligne) et converge pendant ce tour (94.9 s n'est atteint
> qu'au **tour 2 stabilisé** ; tour 1 ≈ 102 s). 1 tour de chauffe ne suffit pas
> à stabiliser ; comparer toujours des **tours stabilisés**.

---

## Ce qui a MARCHÉ ✅

### Lissage de la racing line (le gain retenu)
**Diagnostic :** la racing line min-courbure avait un **kink artefactuel** au
chicane juste avant la ligne de départ — un waypoint mal placé (idx 133 :
rayon 4.4 m, espacement 2.24 m contre ~4.2 m ailleurs). Ce kink (1) faisait
**osciller** la voiture et (2) **plafonnait la vitesse** du virage.

**Fix :** `smooth_racing_line.py` — lissage Laplacien **contraint** (déplacement
max 1.5 m/point) + ré-échantillonnage uniforme, **validé hors-ligne** (la
courbure doit baisser ET la ligne doit rester dans le couloir) avant tout sim.

**Résultat :** rayon mini 4.44 → 6.54 m, courbure² −46 %, espacement uniforme,
ligne toujours dans la piste (offset 4.45 < 8 m). En sim : **94.9 → 89.7 s**,
oscillation −58 %, 0 offtrack.

**Leçon :** toujours vérifier les **kinks / l'espacement** de la racing line
(`R_mini` + flips de courbure). Un seul point mal placé coûte du temps ET fait
osciller. Outils : voir la régularité d'espacement et la courbure par index.

---

## Ce qui a ÉCHOUÉ ❌ (et pourquoi — utile pour ne pas refaire)

### 1. Augmenter la vitesse de croisière (`cruise_velocity` 8 → 9, 8 → 10)
**Crash.** Over-speed en entrée de virage → la voiture ne tient pas la ligne →
sortie → explosion physique (NaN, `lateral_error` ~90-100 m). 

### 2. Profil de vitesse hors-ligne (forward-backward, « freinage garanti »)
**Crash** (v_max 9 ET 10). Le profil point-masse est trop optimiste : il
planifie la vitesse mais **n'ajoute ni grip ni capacité de suivi**. La vraie
voiture (sous-virage + retard du pure pursuit) ne tient pas la ligne aux
vitesses planifiées. *(Un bug d'intégration a été corrigé au passage — lookup
nearest-(x,y) qui saute de section ; fix = index progressif le long de la ligne
— mais même corrigé, le plafond reste.)*

### 3. Régulation type RPP (`regulated_min_radius`, ralentir si on braque fort)
**Régression** (+~10-12 s). Rendait les virages plus lisses mais **trop lents**
(se déclenchait en cornering normal).

### 4. `steer_smoothing` (filtre passe-bas sur le braquage)
**Pire.** Le retard de phase du filtre **amplifie** l'oscillation au lieu de la
réduire.

### 5. « Corriger » le grip (roue avant-gauche `fl_1` mu 1.1 → 1.7)
**Régression** (+~10 s, voiture nerveuse). La voiture a une asymétrie d'origine
(`fl_1` à 1.1, les 3 autres roues à 1.7, dans `autocar_description/urdf/autocar.xacro`).
La passer à 1.7 partout = **balance neutre** → la voiture tourne trop volontiers
→ le pure pursuit **oscille davantage**. **Le pure pursuit préfère une voiture
légèrement sous-vireuse (stable).** Ne pas y toucher.

---

## Le plafond de ~8 m/s (constat important)

Confirmé 3 fois (cruise 9, cruise 10, profil de vitesse) : **le suivi réel du
pure pursuit plafonne à ~8 m/s** sur ce circuit twisty. Au-dessus, la voiture
dévie en virage et ne récupère pas. Ce n'est **pas** un problème de réglage de
vitesse — c'est la capacité de suivi du contrôleur + le grip véhicule.

→ Pour franchir ce plafond, il faudrait **changer de contrôleur** (Vector
Pursuit ou Nav2 Regulated Pure Pursuit — meilleur suivi à haute vitesse) ou
modifier le grip véhicule. Pas un réglage de pure pursuit.

---

## Défauts annexes identifiés (non bloquants)

- **Raccord de boucle** : la racing line a un point dupliqué à la fermeture
  (`[0] == [dernier]`) et le global planner ne **wrappe pas** la fenêtre de
  goals (modes 'start'/'end' qui clampent) → un léger à-coup **1×/tour** au
  passage de la ligne. Impact mineur (mesuré faible).
- **Pic visuel sur le mesh** au raccord : segment de longueur nulle dans le
  `<road>` Gazebo (le `road.append(road[0])` redondant du générateur, alors que
  le ré-échantillonnage ferme déjà la boucle). **Purement cosmétique** : le
  `<road>` n'a pas de collision, la voiture roule sur le `ground_plane` plat.

---

## Reste à faire (optionnel)

- **Léger zigzag sur la grande ligne droite** (lookahead court à haute vitesse).
  Mineur depuis le lissage. Fix ciblé possible : **lookahead adaptatif à la
  courbure** (long en droite = lisse, court en virage = pas de coupe). La
  fonction `lookahead_curvature_scale` existe déjà mais n'est pas branchée.
- **Franchir le plafond ~8 m/s** : intégrer un meilleur contrôleur (Vector
  Pursuit / Nav2 RPP).

---

## Outils de mesure (réutilisables)

- `scripts/diag_metrics.py` — logge vitesse, target_vel, braquage, erreur
  latérale dans `/tmp/diag_metrics.csv`.
- `scripts/analyze_diag.py` — attribue la perte de vitesse aux mécanismes.
- `scripts/check_zigzag.py` — compte les changements de signe du braquage.
- `src/.../autocar_racing_line/scripts/smooth_racing_line.py` — lissage validé
  hors-ligne de la racing line (le générateur du gain de cette session).
