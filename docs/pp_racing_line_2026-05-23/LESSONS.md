# Lessons learned (live log)

One entry per incident or non-obvious decision, **written when it happens, not at the end**. The PDF report cherry-picks the most instructive ones for its "Lessons learned" section.

Format per entry:

```
## YYYY-MM-DD - one-line title
**Symptom**: what we saw.
**Root cause**: what was actually wrong.
**Fix**: what we did.
**Lesson**: what to take away for next time.
```

---

## 2026-05-23 - Phantom file modifications after branch switch

**Symptom**: After `git checkout need_for_speed`, 8 Python files under `src/AutoCarROS2/autocar_nav/` showed up as Modified in `git status`. Running `git restore` cleaned them, but they reappeared instantly.

**Root cause**: `core.filemode = true` combined with WSL/Windows mixed filesystem access. The committed files had the executable bit set (`100755`, since they are ROS 2 nodes), but when the repository is touched from the `\\wsl.localhost\` Windows view the executable bit is dropped at the filesystem layer. Git therefore saw a real `old mode 100755 / new mode 100644` diff that `restore` could not fix because the filesystem kept stripping the bit again.

**Fix**: `git config core.filemode false` for this repo. Diff was 0-byte content, only metadata, so disabling filemode tracking is safe.

**Lesson**: On any project edited across WSL and Windows, set `core.filemode false` from day one. The cost of forgetting is hours of confusion that look like a Git bug.

---

## 2026-05-23 - Decision: integration before features

**Context**: The brief explicitly says we are evaluated on system integration and engineering validation, not on writing functional code.

**Decision**: We invest in `bench.py` (experimental harness) and the extended metrics CSV **before** writing Pure Pursuit. This way the first Pure Pursuit run is directly measurable against the Stanley baseline with the same instrumentation.

**Trade-off**: 0.5 day delayed gratification on the "fun" controller work, in exchange for every subsequent experiment being one command and one CSV row.

**Lesson**: When the grading rubric rewards rigor, build the measurement rig first. The temptation to "just code the controller" is exactly the failure mode the brief is trying to catch.

---

## 2026-05-23 - CSV schema extended without breaking baseline

**Context**: Step 1 extends `lap_timer.py` from a 7-column CSV (timing only) to a 15-column CSV (timing + controller config + cross-track stats + steering rate + off-track events). The existing baseline row (`session_id=2026-05-20T22-07-12`) must stay intact and readable.

**Decision**: in-place migration on node startup. If the existing `~/.ros/autocar_lap_times.csv` has the legacy 7-col header, we rewrite the file once with the new header and pad historical rows with empty cells for the 8 new fields. After that, the file is uniform and pandas can read it directly without ragged-row warnings. `baseline_lap_times.csv` is a frozen snapshot and is not touched.

**Alternative considered**: renaming the legacy file aside and starting a fresh CSV. Rejected because we want a single source of truth, and ragged CSVs make analysis brittle. The migration is 15 lines of code and runs once.

**Lesson**: when extending a persisted format, write the migration as defensive node-startup code rather than asking the user to run a script. The migration becomes invisible and the project keeps the "single source of truth" property.

---

## 2026-05-23 - Topic contract for metrics: `/autocar/lateral_error`

**Decision**: every controller (Stanley today, Pure Pursuit and MPC tomorrow) MUST publish `std_msgs/Float64` on `/autocar/lateral_error` (signed metres, positive on one side of the path). `lap_timer` aggregates it into per-lap RMS and max, and uses a configurable `offtrack_threshold_m` (default 4.0 m) to count rising-edge off-track events.

**Why**: keeps the metrics CSV comparable across controllers. If a future controller computes lateral error differently (Pure Pursuit may use lookahead-based projection rather than nearest-point projection), the contract still holds as long as it publishes the signed distance to the path it is tracking.

**Lesson**: instrumentation belongs on a topic, not inside the controller's logs. Any new controller is then drop-in compatible with the recorder.

---

## 2026-05-23 - CRLF line endings broke every Python node

**Symptom**: after `colcon build`, every Python ROS node died at startup with `exit code 127` and a kernel-level message `/usr/bin/env: 'python3\r': No such file or directory`. Only `lap_timer.py` (which had been rewritten from scratch by the assistant via WSL filesystem) actually ran; the others (localisation, globalplanner, localplanner, tracker) all crashed instantly, so the car spawned but could not be driven.

**Root cause**: `core.autocrlf = true` in the local git config. The repo had been cloned originally with that setting active, so Git silently converted LF to CRLF in the working tree. Linux's kernel takes the shebang `#!/usr/bin/env python3` literally — with a trailing carriage return it tries to execute an interpreter called `python3\r`, which doesn't exist. The shell error is fatal, the node never starts.

**Fix (two parts)**:
1. **One-shot cleanup**: `find src/AutoCarROS2 -type f \( -name '*.py' -o -name '*.sh' -o -name '*.yaml' -o -name '*.xml' -o -name '*.xacro' -o -name '*.world' \) -exec sed -i 's/\r$//' {} +` and a full `rm -rf install build log && colcon build`.
2. **Durable prevention**: add `.gitattributes` enforcing `* text=auto eol=lf` (plus explicit `eol=lf` for every text type) and `git config core.autocrlf false`. Now any future `git checkout` keeps Linux-friendly line endings even if the repo is re-cloned from a Windows machine.

**Bad attempt en route**: `colcon build --symlink-install` was tried as a side improvement. It interacted poorly with `install(PROGRAMS ...)` in CMakeLists.txt: install/ ended up missing all the .py executables, breaking the launch *differently*. Reverted to plain `colcon build` (copy mode), which works.

**Lesson**: on any Linux-target ROS 2 project that can be cloned from Windows or via `\\wsl.localhost\`, ship a `.gitattributes` and set `core.autocrlf = false` from day one. The cost of forgetting is hours of "but the file looks fine in the editor!" debugging where the actual character is invisible.

---

## 2026-05-23 - Stanley oscillation visually confirmed (matches CSV numbers)

**Observation**: when watching the running simulation, the car was visibly oscillating, *especially in the second half of the lap once the cruise velocity (6.0 m/s) was reached*. The wheels were slapping left-right around the path rather than tracking smoothly.

**Data matches**: the lap row from this run shows `steering_rate_max = 8.76 rad/s` (~500 deg/s, very high), `lateral_error_max = 5.96 m`, and `offtrack_events = 6` (six rising-edge crossings of the 4 m threshold). The numbers say what the eye saw.

**Explanation**: at higher speed the cross-track term in Stanley `arctan(k * e / (k_soft + v))` becomes saturated and reactive. Without anticipation, every disturbance produces a correction that arrives late, overshoots, and gets reverted on the next tick. The oscillation is sustained.

**Why this matters for the report**: this is a textbook motivation for Pure Pursuit. The lookahead in Pure Pursuit grows with speed, which damps the loop naturally. Once Pure Pursuit is implemented (step 3), we expect `steering_rate_max` to drop by ~3-5x and `offtrack_events` to fall to zero on the same circuit at the same target speed. The before/after comparison is one of the headline results in Section 5.2.

**Lesson**: keep an eye on the visual behavior during runs, not just the lap time. The instrumentation captures the why; the eye captures the what.

---

## 2026-05-23 - Pure Pursuit revealed a hidden architecture coupling

**Symptom**: Pure Pursuit ran its first full lap in 195.3 s (5.4 % faster than Stanley's cold-start 206.5 s) with twice better `lateral_error_rms` (0.43 vs 0.93 m) and far fewer off-track events (2 vs 7). Visual behaviour was much smoother than Stanley, as predicted. But during lap 2 the car suddenly left the road and ended up centred in the infield -- in spite of a steering algorithm that, on paper, just follows a path.

**Root cause**: the inherited `local_planner.py` ships with an obstacle-avoidance layer that, when the Bayesian Occupancy Filter (`bof`) flags something near the path, tries lateral offsets `[0, +/-1.5, +/-3, +/-4.5, +/-6]` until one is "free", and republishes the path shifted by that offset. On `race_circuit.world` there are no on-road obstacles, but the 48 hay bales bordering the track are close enough that the BOF occasionally flags them. Stanley masked the shift with its natural oscillation; Pure Pursuit, being far smoother, follows the shift faithfully. A 6 m lateral shift on a 16 m wide track is enough to put the car off the road.

**Fix**: in `localplanner.py`, restrict `LATERAL_OFFSETS = [0.0]` for racing experiments. The car always follows the centerline. Re-enable the full list only for obstacle-aware tests. Localisation and path planning are unaffected -- only the avoidance layer is disabled.

**Lesson (very important for the report)**: this is a textbook *integration coupling*. The behaviour of the controller cannot be evaluated in isolation -- it interacts with the planner's choices, which were tuned for a different controller and a different scenario. The brief's emphasis on "engineering validation" is exactly about catching this kind of cross-component effect, where each layer is locally correct but the composition produces an unexpected failure. Worth a paragraph in REPORT.md Section 6.

---

## 2026-05-23 - Self-inflicted: theoretical fix to localplanner made things worse

**Symptom**: after Pure Pursuit completed a clean cold-start lap (195.3 s) but went off the road on lap 2 (ended up centred in the infield), the assistant edited `localplanner.py` twice without running a test in between: first set `LATERAL_OFFSETS = [0.0]`, then bypassed the BOF blockage check entirely and locked `target_vel` to `CRUISE_VEL`. After rebuild and relaunch, the car oscillated wildly across the road and eventually hit a pillar.

**Root cause**: untested guesses cascaded. The two changes were theoretically reasonable (each addressed a real upstream effect) but their combined dynamic behaviour was never observed before being committed. As a bonus failure, one of the comments was written with `//` (C++) instead of `#` (Python), which only stopped being a syntax bomb because the line never executed.

**Fix**: revert both edits, return `localplanner.py` to its tree-baseline state, rebuild, relaunch. The known-good state is recovered. The remaining bug (PP off-track on lap 2) is real but must be diagnosed without changing the planner.

**Lesson**: when a change is "obvious", that is exactly when to run it once before assuming it works. Especially in tightly-coupled control loops where dynamic behaviour rarely matches static reasoning. The brief grades on engineering validation, not on theoretical correctness -- a fix that breaks something else is worse than the original bug.

---

## TODO - next incidents go here

(Examples of things we know will bite us, to keep an eye out for:)
- Gazebo gzserver not killed cleanly between runs - the launch already does `killall`, but `bench.py` will need to do it too.
- `use_sim_time` propagation: any new node that forgets this parameter will hang silently waiting for `/clock`.
- Pure Pursuit at low speed: dynamic lookahead `Ld = k_v * v + Ld0` needs the `+ Ld0` floor or the car oscillates at standstill.
- Min-curvature QP: feasibility issues when track width is small relative to vehicle wheelbase.
