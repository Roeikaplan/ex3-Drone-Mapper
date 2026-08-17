# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

```bash
# Configure (only needed once, or after CMakeLists.txt changes)
cmake --preset default

# Build all targets
cmake --build --preset default

# Run the simulation
./build/drone_mapper_simulation [simulation.yaml] [output_path]

# Run maps comparison
./build/maps_comparison <origin_map> <target_map> [comparison_config=<path>]

# Run the yaml example
./build/example_yml
```

The build uses **Ninja** via vcpkg. All compiler warnings are errors (`-Wall -Wextra -Werror -pedantic`). Dependencies (`mp-units`, `yaml-cpp`, `tinynpy`, `gtest`) are managed by vcpkg — the toolchain file is set in `CMakePresets.json`.

**Note:** `yaml-cpp::yaml-cpp` is intentionally commented out of `drone_mapper`'s `target_link_libraries` in `CMakeLists.txt`. Uncomment it when you actually add YAML parsing.

## Architecture

This is a C++20 drone mapping simulator skeleton for an academic assignment. The core library is `libdrone_mapper.a`; two executables link against it: `drone_mapper_simulation` and `maps_comparison`.

### Interface Hierarchy

All major components are hidden behind pure interfaces in `include/drone_mapper/I*.h`. The only concrete implementations provided (and meant to be kept) are:

- **`MockLidar`** — the real LiDAR ray-marching sensor. Do not replace.
- **`Map3DImpl`** — wraps a `NpyArray` (tinynpy), implements both `IMap3D` (read-only) and `IMutableMap3D` (writable + saveable).

Everything else (`MissionControlImpl`, `DroneControlImpl`, `MappingAlgorithmImpl`, `SimulationRunFactoryImpl`, `SimulationManager`, `MapsComparison`, `ScanResultToVoxels`, mock GPS/movement) are stubs to be replaced.

### Object Ownership Model

`SimulationRunFactoryImpl::create()` is the single wiring point. It:
1. Loads the hidden `.npy` map from disk into a `shared_ptr<NpyArray>` → `Map3DImpl`.
2. Creates an empty output `Map3DImpl`.
3. Constructs all components (GPS, Lidar, Algorithm, DroneControl, MissionControl) injecting **references** to the run-owned maps.
4. Transfers `unique_ptr` ownership of everything into `SimulationRunImpl`.

`SimulationRunImpl` owns all objects via `unique_ptr`; inner components hold **non-owning references** to siblings. This means component lifetime is tied to `SimulationRunImpl`.

### Data Flow

`SimulationManager::run()` takes `SimulationCompositionData` (Cartesian product of simulations × missions × drones × lidars), creates one `SimulationRunImpl` per combination via the factory, calls `run()` on each, and aggregates results into `SimulationManagerReport`.

Per-run flow: `MissionControlImpl` drives a step loop → `DroneControlImpl::step()` → `IMappingAlgorithm::nextStep()` returns a `MappingStepCommand` → movement is validated and executed → `ILidar::scan()` → `ScanResultToVoxels::applyToMap()` writes voxels into the output map → `MissionControlImpl` saves the map → `MapsComparison::compare()` scores against the hidden map.

Two ordering rules enforced in `DroneControlImpl::step()`:
- **First step passes `nullptr`** for `latest_scan` because no LiDAR result exists yet.
- **When `MappingStepCommand` has both `movement` and `scan_orientation`**, movement must be validated and executed first; the scan then runs from the updated drone state.

### Error Handling & Logging

`ErrorLogger` has two sinks: **`errors.log`** (`output_results/`) gets *every* error immediately via `log()`; **`input_errors.txt`** (`output_path` root) gets *only recovered input-file errors* via `logInputError()` and is created lazily (so it appears only when the input had problems). `CompositionLoader` uses `logInputError` for `CONFIG_*` recoveries; `SimulationManager` logs run-reported errors — mission errors like `DRONE_STEP_ERROR` and `RESOLUTION_IGNORED` — the moment `run()` returns (`logRunOutcome`), plus `RUN_FAILED` for thrown exceptions. `main` never crashes/`exit()`s: the load and the whole run are wrapped, and it always returns from `main`.

### Units System (`Units.h`)

All physical quantities use `mp-units` (v2.3+). Key aliases:
- `PhysicalLength`, `XLength`, `YLength`, `ZLength` — in centimeters (`cm`)
- `HorizontalAngle`, `AltitudeAngle` — in degrees (`deg`)
- `Position3D { XLength x, YLength y, ZLength z }`
- `Orientation { HorizontalAngle horizontal, AltitudeAngle altitude }`

X, Y, and Z lengths are **distinct quantity specs** — the compiler rejects accidental mixing.

### Key Types (`include/drone_mapper/types/`)

| Type | Purpose |
|------|---------|
| `MapConfig` | Bundles `MappingBounds`, `Position3D offset`, `PhysicalLength resolution` — the canonical map geometry |
| `SimulationConfigData` | Hidden map path, resolution, offset, initial drone pose |
| `MissionConfigData` | `max_steps`, `gps_resolution`, `output_mapping_resolution_factor`, `mission_bounds` |
| `SimulationCompositionData` | Nested groups: `(SimulationConfig, [MissionConfig])` × drones × lidars |
| `MappingStepCommand` | Optional movement + optional scan orientation + `AlgorithmStatus` |
| `LidarConfigData` | `z_min`, `z_max`, `d` (ray spacing), `fov_circles` |
| `VoxelOccupancy` | `Unmapped(-1)`, `Empty(0)`, `Occupied(1)`, `PotentiallyOccupied(-3)`, `OutOfBounds(-2)` |

### Map Files

`.npy` files are NumPy 3D voxel arrays (one signed/unsigned byte per cell) used as hidden maps; `tinynpy` handles serialization. Two datasets:
- `data_maps/` — small synthetic maps (5×5×5, plus `benchmark_map.npy` 29×30×31). `benchmark_map.cw` is a companion file.
- `inputs/map/` — the **realistic scenario** maps referenced by `inputs/sim_compose.yaml`: `scenario_small.npy` (20³), `scenario_house.npy` (29×30×31), `scenario_big.npy` (30³). `.cw` files + `npy_to_cw.py` are a visualization/companion format the loader ignores.

Prefer small `data_maps` maps for fast tests; the `inputs/` scenarios are larger (slower to fully map with the real algorithm).

### Composition Input Format (two layouts)

Per the assignment, a composition file **references separate config files by path** (the mandated format). `inputs/sim_compose.yaml` follows it:

```yaml
simulation_compositions:
  simulations:
    - simulation_config: "simulation/house_simulation.yaml"   # path string
      mission_configs: ["mission/house_mission_lower.yaml"]   # path strings
  drone_configs: ["drone/drone_small.yaml"]                   # path strings
  lidar_configs: ["lidar/lidar_long.yaml"]                    # path strings
```

Each referenced file wraps its body under `simulation_config:` / `mission_config:` / `drone_config:` / `lidar_config:`. **Relative paths (both the referenced configs and each `map_filename`) resolve against the composition file's parent directory** — e.g. in `inputs/`, `house_simulation.yaml`'s `map_filename: "map/scenario_house.npy"` is `inputs/map/scenario_house.npy`.

`src/CompositionLoader.cpp` currently parses only the older **inline** layout (used by the repo-root `simulation.yaml`, where config maps are written directly). It must be extended to accept the file-reference layout; auto-detecting scalar-string vs. map nodes keeps inline support (and the existing `CompositionLoaderTest`) working. `cpp_yaml_example/` is a standalone `yaml-cpp` read/write reference.

## Documentation Conventions

Match the existing house style when adding or changing code. Exemplars: `MockGPS.h` /
`MockMovement.h` for headers; `MockMovement.cpp`, `ScanResultToVoxels.cpp`, `Map3DImpl.cpp` for
sources.

- **Headers** — Doxygen block on every class and each public method/notable member: `@brief`,
  `@param`, `@return`, and a `@note` calling out architectural boundaries (who owns a
  responsibility and why). Example: "DroneControl owns all movement validation because
  `MockMovement` never validates."
- **Sources** — concise *why-focused* comments explaining intent, not the obvious mechanics.
  Always document: the unit-stripping idiom (`force_numerical_value_in(cm)` → scale the plain
  scalar → re-attach the `x_extent[cm]` quantity spec), the angle convention (`0°=+X east`,
  `90°=+Y south`), the movement-before-scan ordering, and any non-obvious branch or invariant
  (e.g. first-step bootstrap, BFS frontier/termination, rotation splitting by `max_rotate`).
- **File-local helpers** — free functions and anonymous-namespace helpers (even short,
  throwaway ones like a `configForShape` in an executable's `main`) get the same full Doxygen
  block as header methods: `@brief`, `@param`, `@return`, and a `@note` for any assumption or
  boundary. Exemplar: `maps_comparison_main.cpp`.
- Keep comment density and phrasing consistent with the surrounding file; don't restate what the
  code plainly says.

## What to Implement

The core pipeline plus the mandated I/O formats, utilities, and tests are implemented. Done:
- **Composition loader** (`src/CompositionLoader.cpp`) — accepts both the mandated **file-reference** layout (`inputs/sim_compose.yaml`; config entries are path strings to files wrapped under `*_config:`, relative paths resolved against the composition file's parent dir) and the older inline layout. Optionally records each config's source path into a `CompositionPaths` out-param (`include/drone_mapper/CompositionPaths.h`) for the report.
- **`simulation_output.yaml`** (`src/SimulationOutputWriter.cpp`) — nested under `score_report:`, mission-level `resolution_cm` + `resolution_request_status`. **Path-based identity** for the file-reference layout (`simulation_config`/`mission_config`/`drone_config`/`lidar_config`, matching the PDF sample), driven by the `CompositionPaths` side-channel (**no skeleton struct was modified**); falls back to **value-based** labels (`map_filename` / `output_map`) for the inline layout or an empty `CompositionPaths`.
- **`maps_comparison comparison_config`** — `src/ComparisonConfig.cpp` parses `original`/`target` (`map_res_cm`/`map_offset`/`map_boundaries`); `maps_comparison_main.cpp` uses per-map geometry when a config is given (no config → identical-geometry assumption). This also enables comparing maps of differing resolution (the standalone-utility bonus), since `MapsComparison::compare` samples each map by world position.
- **Integration tests** under `/tests/integration/` (suite **`Integration`**): real-algorithm + mock-algorithm whole-flow + all `inputs/` scenarios, wired into `drone_mapper_simulation_test`.

The step-loop / voxel / serialization stubs listed in `docs/HLD.md` are already done — treat that list as historical.

## Bonuses

- **Supporting different resolution requests** (implemented) — the output map can be written at a resolution different from the input map (mission `gps_resolution_cm` × `output_mapping_resolution_factor`), and `MapsComparison` scores it against the different-resolution ground truth. See `bonus.txt`; tests under `tests/bonus/` with filter `--gtest_filter=BonusResolution.*`. Any new bonus requires a `bonus.txt` entry (description, how to check, test filter).

## Context Files

Read the following files for additional context:
- `docs/HLD.md` — high-level design, architecture, and requirements
- `project_context.md` — assignment requirements, goals, and constraints.
- `README.md` — project overview, build instructions, and usage