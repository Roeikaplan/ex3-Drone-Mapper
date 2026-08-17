# Project Context — TAU Advanced Topics in Programming 2026B

> Internal reference document summarizing Assignment 1 and Assignment 2 requirements.
> Source: `Advanced Topics TAU 2026B - Assignment 1 - v2.pdf` and `Advanced Topics TAU 2026B - Assignment 2.pdf`.

---

## ⚠️ CRITICAL INSTRUCTION FOR CLAUDE

**This document is internal context. Treat it as the source of truth for the assignment's requirements, but do not start new implementation work off it alone — act only on the user's explicit instructions for the current task.**

**Current active task:** integration tests under `/tests/integration/` and correct **input/output file handling** — specifically making the composition loader accept the mandated file-reference format (see `inputs/`) and making `simulation_output.yaml` match the PDF's `score_report:` layout. Adhere strictly to the provided interfaces/data types; do not deviate from them.

---

## 🎯 CURRENT STATE & WORKING DIRECTIVE

> **Read this section first in every new session and adopt its mindset immediately.**

1. **FOCUS** — We are currently working **strictly on Assignment 2**. Assignment 1 is included in this document only as background context; it is not the active task.

2. **IMPLEMENTATION STRATEGY** — We are implementing the logic **from scratch** to fit the new **C++20 skeleton**, its explicit dependencies, and its interfaces (e.g. `SimulationRunImpl`, `MockLidar`). We are **NOT** blindly copying or migrating old Assignment 1 code. Old code may inform understanding, but every implementation must be written fresh against the Assignment 2 architecture.

3. **MINDSET** — Every time you (Claude) read this file in a new session, immediately adopt this mindset:
   - Adhere **strictly** to the Assignment 2 skeleton — do not deviate from the provided interfaces or data types.
   - Utilize the **new data types** (e.g. `MapConfig`, the updated `MissionConfigData`/`SimulationConfigData`/`SimulationCompositionData`, `LidarScanResult`).
   - Write **fresh, compliant code** that fits the provided architecture perfectly.

---

## 1. Assignment 1 — Requirements and Guidelines (Summary)

Assignment 1 establishes the conceptual foundation. The requirements were intentionally open: you chose your own API, class design, and file formats. Assignment 2 later formalizes these (see Section 2).

### The Drone Mapper
A drone that maps 3D buildings by flying inside and scanning its surroundings. You implement only the software simulation parts.

**Drone behavior:**
- Starts at a location `{X, Y, Height}` (the drone's center) with an initial XY-Angle (default 0). Angle convention: `0 = east (+X)`, `90 = south (+Y)`, `180 = west (-X)`, `270 = north (-Y)`.
- Moves through 3D space to map surroundings.
- Has an internal **Position Sensor** giving its exact center position `{X, Y, Height, XY-Angle}`.
- Has an internal **Lidar Sensor** reporting elements (walls, ceiling, floor, obstacles) within its field of view (FOV).
- Movement driver commands: `Rotate <Left/Right> by <angle>`, `Advance <distance>`, `Elevate <distance>`. Angle and distance may be positive or negative.
- Manages the mapping mission **autonomously**, stopping when mapping is complete and back at the initial position (in Ex1 returning to start is NOT required — may report "Finished" upon completion).

**Output of mapping** — a sparse 3D matrix queryable by `{X, Y, Height}`:
- `0` = empty
- `1` = occupied
- `-1` = not mapped (couldn't be mapped)
- `-2` = not mapped (outside required mapping boundaries)

### Configuration

**Drone capabilities config:**
- Minimal dimension it can pass through (width/length/height). In Ex1 assume the drone is a **perfect sphere**.
- Max movement per request: Rotate (angle), Advance (distance), Elevate (distance).

**Lidar capabilities:**
- Always emits one central beam along the FOV center ("Circle 0"). Additional beams point toward outer circles.
- **Z-min**: minimum operational distance (e.g. 20 cm). Below this, beams detect objects but cannot measure distance accurately (distance reported as `0`).
- **Z-max**: maximum operational distance (e.g. 120 cm). Beyond this, nothing is identified.
- **D**: spacing between consecutive beam circles measured at Z-min (e.g. 2.5 cm → Circle 1 radius 2.5 cm, Circle 2 radius 5 cm, …). Each outer circle has **4× the beams** of the previous, evenly spread around its circumference, aligned with the lidar elevation angle.
- **FOVC**: number of beam circles (e.g. 1 = single central beam; 5 = circles 0–4, **341 beams total**).
- Lidar has **blind spots** between beams. You must not ignore areas larger than the required mapping resolution.

**Mission config:**
- Mapping boundaries: bounded rectangle (min/max X, min/max Y) plus min/max height.
- Initial drone position `{X, Y, Height}` + initial XY-Angle (default 0 = east).
- Required result resolution (decimal places for X/Y, separately for Height). In Ex1 a single supported resolution may be assumed; non-matching requests may error.
- Recharging positions (set of `{X, Y, Height}`, possibly empty) — **NOT used in Ex1** (assume infinite battery).

### Lidar Scan Results
- Per hitting beam: 3D-angle (azimuth) and distance to detected object.
- Distance below Z-min → reported as `0`.
- Result size ≤ number of beams; may be `0` if nothing detected. (In Ex1 you may implement a lidar that returns info on all beams.)
- Each beam detects only **one** object (the front one); hidden objects behind are not detected by that beam.

### Input / Output / Flow
- **Inputs:** drone capabilities text file, mission text file, building map (text or binary) — the map is invisible to the drone and used ONLY by the Lidar mock.
- **Output:** generate an output map file in the same format as the building map; compare the two and produce a **score between 0 and 100** (100 may be unreachable if parts are inaccessible). Score formula was your choice in Ex1.
- **Flow:** load inputs → initialize drone with Lidar/Position/Movement mock interfaces + Building Map interface → run main loop asking the drone for commands (`Rotate`, `Advance`, `Elevate`, `Scan`, `Get Location`, `Finished`) → drone reads/writes the Building Map (values come ONLY from what the drone mapped, NEVER from the simulation input map) → on finish, write the map to file, compare, print score.
- **Scan** defaults: XY-angle defaults to drone direction; height angle defaults to zero.

### Algorithm
- Deterministic (no randomness), based on BFS/DFS or similar. Ex1 optimization not required.
- Must **never collide** with elements (collision ends the simulation with a failure notice).

### Error Handling
- Program must **never crash**.
- Recover from input file errors with reasonable defaults; log recovered errors to `input_errors.txt` (create only if errors exist).
- Unrecoverable error → print message to screen before finishing.
- Never use `exit()` or similar — always end by returning from `main`.

### Running (Ex1)
- `drone_mapper [<input_output_files_path>]` (missing arg → current working directory).
- Reads `drone_config.txt`, `mission_config.txt`, `map_input.txt`; writes `map_output.txt` (overwrite if exists).

### Strong Types
- Files: angles in degrees (0–360), distances in cm.
- Code: all values/APIs use the **mp-units** library — `si::unit_symbols::m` (meters), `cm` (centimeters), `deg` (degrees).

### Mandatory HLD document (part of grade)
- UML class diagram, UML sequence diagram of the main flow, design considerations/alternatives, and testing approach.

---

## 2. Assignment 2 — New Requirements, APIs, and Strict Constraints (Summary)

Assignment 2 is essentially a **refactoring exercise**: the API and file formats are now **formalized and mandatory**. You must refactor Ex1 code to match exactly, and add **Component Testing** and **Integration Testing** with **GTest and GMock**.

> **Hard constraint:** You may NOT deviate from the provided Interfaces and Data Types. Start from the published Ex2 skeleton and use the **exact** names given.

### Mandatory Components (implement with these exact names)
- **main** — minimal; creates `SimulationManager` and passes arguments.
- **SimulationManager** — creates and manages all `SimulationRun`s; performs scoring.
- **ISimulationRun** / **SimulationRunImpl** — single simulation working logic.
- **ISimulationRunFactory** / **SimulationRunFactory** — factory creating concrete runs.
- **IMissionControl** / **MissionControlImpl** — manages the mission (used in both simulation and real world).
- **IDroneControl** / **DroneControlImpl** — drone movement/lidar decisions; the mapping algorithm is injected via the `IDroneControl` constructor.
- **IMappingAlgorithm** / **MappingAlgorithmImpl** — used solely by drone control; same class for simulation and real world.
- **ILidar** — with `LidarDriver` (not in scope) and **MockLidar** (used by simulation, injected into drone control).
- **IGPS** — with `GPSDriver` (not in scope) and **MockGPS**.
- **IDroneMovement** — with `MovementDriver` (not in scope) and **MockMovement**.
- **IMap3D** / **Map3DImpl** — voxel map.
- **ScanResultToVoxels** — utility **provided by the course team**; takes Lidar scan output and applies it to the map.
- **MapsComparison** — standalone utility comparing input vs. generated output map.
- Helper data types: `LidarScanResult`, `MissionConfigData`, `DroneConfigData`, `SimulationConfigData`, etc.

All components are mandatory except those marked "not in scope".

### Documented API Change History (from the skeleton updates)
- **9.6.26** — `IMap3D` refactored to carry `MapConfig` (boundaries, offset, resolution). `SimulationRun` and `MissionManager` aligned; types decoupled. `MapsComparison` reworked. cpp-yaml added to the skeleton.
- **12.6.26** — `IMappingAlgorithm` API changed (see APIs section / forum).
- **15.6.26** — Major: `ScanResultToVoxels` now applies voxels directly to the output map. `MappingAlgorithm` constructor now takes `LidarConfigData` and `MissionConfigData`. `MockGPS` got a resolution field. `IMap3D` gained an `isInBounds` helper.
- **20.6.26** — `ILidar` got a `config()` method; `MissionConfigData` gained mission boundaries; `SimulationCompositionData` changed to support nested simulation → mission configs.

### File Formats (now strict)
- **Map input/output:** `.npy` files in **binary format** (as used by VoxelMap, Matplotlib, PyVista, Minecraft). Output map may be at a different resolution than input.
- **All config files: YAML.** Specific keys:
  - **drone_config:** `dimensions_cm` (sphere diameter), `max_rotate_deg`, `max_advance_cm`, `max_elevate_cm`.
  - **mission_config:** `max_steps`, `boundaries` (`x_boundary`/`y_boundary`/`height_boundary` each with `min_cm`/`max_cm`), `gps_resolution_cm`, `output_mapping_resolution_factor` (integer; missing → defaults to 1; `< 1` → ignored with error log).
  - **lidar_config:** `z_min_cm`, `z_max_cm`, `d_cm`, `fov_circles`.
  - **simulation_config:** `map_filename`, `map_resolution_cm`, `initial_drone_position` (`x_cm`/`y_cm`/`height_cm`), `initial_angle_deg`, `map_axes_offset` (`x_offset`/`y_offset`/`height_offset`).
  - **comparison_config:** `original` and `target`, each with `map_res_cm`, `map_offset`, `map_boundaries`. (Boundaries must be at most the map size.)
  - **simulation_compositions:** `simulations` (each a `simulation_config` + list of `mission_configs`), `drone_configs`, `lidar_configs`. The runs are the **cartesian product** `[mission_configs] × [drone_configs] × [lidar_configs]` **within each simulation** (missions are bound to their simulation).

#### Composition file layout — file-reference format is mandatory (IMPORTANT)

The PDF (pp. 4–5) mandates that a composition file **references separate config files by path**, it does not inline them:

```yaml
simulation_compositions:
  simulations:
    - simulation_config: "simulation/house_simulation.yaml"   # a path STRING
      mission_configs:
        - "mission/house_mission_lower.yaml"                  # path STRINGS
        - "mission/house_mission_full.yaml"
  drone_configs:
    - "drone/drone_small.yaml"                                # path STRINGS
  lidar_configs:
    - "lidar/lidar_long.yaml"                                 # path STRINGS
```

Each referenced file wraps its body under its own top-level key: `simulation_config:`, `mission_config:`, `drone_config:`, `lidar_config:` (and `comparison_config:` for the comparison utility). The provided sample dataset in `inputs/` (`inputs/sim_compose.yaml` + `inputs/{simulation,mission,drone,lidar,map}/…`) follows this mandated format.

- **Relative-path resolution:** in the `inputs/` dataset a referenced `simulation/house_simulation.yaml` itself points at `map_filename: "map/scenario_house.npy"`, which only resolves relative to the **composition file's base directory** (`inputs/`), not relative to the referenced yaml's own directory. Convention: **resolve all relative paths (referenced configs and `map_filename`) against the composition file's parent directory.**
- **Two formats exist in the repo:** the repo-root `simulation.yaml` uses an older *inline* layout (config maps written directly); `inputs/sim_compose.yaml` uses the mandated *file-reference* layout. The loader must support the file-reference layout; keeping inline support (auto-detect: scalar string ⇒ load referenced file, map ⇒ parse inline) preserves the existing inline example and its component test.
- **`output_mapping_resolution_factor`:** the PDF comments it as a *"relative mapping resolution factor vs GPS"* (integer; missing → 1; `< 1` → ignored with error log). The exact base it scales (GPS resolution vs. hidden-map resolution) is a design decision — see the decisions list.

### Output Files (Ex2)
- **Error log** — your chosen format; all errors **MUST be logged immediately when they occur, never deferred**.
- **Map output** — `.npy` binary, possibly different resolution.
- **simulation_output.yaml** — hierarchical score report nested under a top-level **`score_report:`** key with: `composition_file`, `generated_at_utc`, `metric`, `score_range` (min 0 / max 100), `error_score: -1`, a `summary` (`total_runs`, `scored_runs`, `error_runs`, `average_score`, `min_score`, `max_score`), and `simulations → missions → runs`. **`resolution_cm`** (the *actual* output resolution) and **`resolution_request_status`** (`IGNORED TOO SMALL` / `IGNORED` / `ACCEPTED`) sit at the **mission** level; each run carries `status` (`completed`/`max_steps`/`error`), `steps`, `score`, and `error_ref.code` on error.
  - **Identity is path-based (matches the PDF sample), with a value-based fallback.** For the mandated file-reference layout the report labels each level by its source config file path — `simulation_config:` / `mission_config:` / per-run `drone_config:` + `lidar_config:` — exactly as the sample shows. The paths are carried by a `CompositionPaths` side-channel (`include/drone_mapper/CompositionPaths.h`) filled by `loadComposition` and passed to the writer, so **no skeleton data type was modified**. For the inline composition layout (no config files) the writer falls back to value labels (`map_filename` / `output_map`). Implemented in `src/SimulationOutputWriter.cpp`.
- **output_results/** folder — all maps and error logs organized (naming/nesting your choice).
- Document the `simulation_output.yaml` and `output_results` formats in `readme.txt`.

### MapsComparison Utility (strict)
- API must match the reference stub. Standalone class — no interface needed.
- Additional executable: `./maps_comparison <origin_map> <target_map> [comparison_config=<path>]`.
  - `origin_map`/`target_map` are `.npy` filenames (with or without path).
  - `comparison_config` optional; if absent assume both maps share the same offset, boundaries, resolution.
  - Comparing **different resolutions** is an optional **bonus** feature only.
- Prints **only the score** (float 0–100, no extra text) to stdout. On error: print `-1` to stdout and a descriptive message to stderr.
- Checked properties: identical maps → 100; very similar → close to 100 but not 100; very distinct → close to 0; in-between → reasonable.

### Testing (mandatory, GTest + GMock)
- **Component tests** under `/tests/components/` for: `SimulationManager`, `SimulationRun` (also tests MockGPS + MockMovement), `MissionControl`, `DroneControl`, `MappingAlgorithm`, `MockLidar`, `MapsComparison`.
- **Integration tests** under `/tests/integration/` — at least two whole-flow tests (happy path focus): (1) all components with the real algorithm; (2) all components with a mock algorithm.
- **How tests are graded:** staff inject bugs into a component. Unaffected component tests must NOT fail; the affected component test must detect ≥ 60% of bugs; integration tests must detect ≥ 20%.

### Running (Ex2)
- **Program:** `./drone_mapper_simulation [<simulation.yaml>] [<output_path>]`.
  - Missing arg → use `simulation.yaml` in CWD. Filename only → look in CWD. Relative path → under CWD. Absolute path (`/…`) → use as given.
  - Output (in `output_path` or CWD, overwriting): `simulation_output.yaml` + `output_results/` folder.
- **Tests:** `./drone_mapper_simulation_test`, with `--gtest_filter=` support for `Integration.*`, `SimulationManager.*`, `SimulationRun.*`, `MissionControl.*`, `DroneControl.*`, `MappingAlgorithm.*`, `MockLidar.*`, `MapsComparison.*`.

### Error Handling (Ex2 additions)
- Same rules as Ex1, plus: if a scenario errors but the simulator can continue, give that scenario score `-1` and proceed to the next. If an entire group cannot run (e.g. bad map file), auto-fill `-1` for all cases in that group. Always log immediately.

### Key Ex2 Clarifications (FAQ)
- Algorithm need not be optimal but should be reasonably efficient (small maps mappable within ~10s; >2 min draws penalties; ~3× slower than ideal is fine for Ex2).
- Must use the **exact** provided Interfaces and Data Types; start from the Ex2 skeleton.
- Ex1 assumptions still hold unless redefined: no battery/recharging (also dropped in Ex3), drone is a perfect sphere, lidar may point to any requested angle.
- Single supported resolution still allowed — but now the simulation may decide it doesn't support the requested resolution, report it as **ignored**, and fall back to a default resolution fitting the mission.

### Bonuses (Ex2)
- Smart algorithms do NOT earn bonuses (that's later assignments). Eligible: visual simulation (external utility, must not be mandatory to run), supporting different resolution requests.
- Don't re-request an Ex1 bonus. Requesting a bonus requires a `bonus.txt` in the main directory describing each addition, how to check it, and (when relevant) a specific test filter.

---

## 3. Strong Types Reminder (applies to both)

All in-code values and APIs use **mp-units**. Files use degrees for angles and centimeters for distances. The skeleton defines distinct quantity specs for X/Y/Z lengths and horizontal/altitude angles to prevent accidental mixing.
