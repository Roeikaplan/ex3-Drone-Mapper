# Assignment 2 Refactor Skeleton

This repository is a compilable skeleton for Assignment 2 in the 2026 Advanced
Topics in Programming course. It intentionally provides interfaces, data types,
dependency-injected component stubs, and a preserved mock LiDAR implementation.
It **does not** implement the full simulator or mapping solution. You should not use ANY implementations provided in this repository (aside MockLidar).

## Contributors

| Name        | ID        |
|-------------|-----------|
| Roei Kaplan | 323998450 |

## Project Structure

```text
include/drone_mapper/      Public interfaces, data types, and skeleton classes
src/                     Implementations and executable entry points
tests/components/        Per-component GTest/GMock suites
tests/integration/       Whole-flow integration tests (Integration.* suite)
tests/bonus/             Bonus-feature suites (BonusResolution.* suite)
data_maps/               Small example NumPy voxel maps (fast tests)
inputs/                  Realistic sample dataset in the mandated composition format
                         (sim_compose.yaml + simulation/ mission/ drone/ lidar/ map/)
.devcontainer/           Development container setup
CMakeLists.txt           CMake build configuration
vcpkg.json               Dependency list
```


## Building

```bash
cmake --preset default
cmake --build --preset default
```

Compiled as C++20 with `-Wall -Wextra -Werror -pedantic` (gcc 11.4+). The main build targets are:

```text
drone_mapper_simulation
drone_mapper_simulation_test
maps_comparison
```

External libraries (course-approved, declared in `vcpkg.json` and resolved by the vcpkg toolchain
referenced in `CMakePresets.json`):

- **yaml-cpp** — YAML reading/writing. Parses the mandated config/composition files
  (`src/CompositionLoader.cpp`) and writes `simulation_output.yaml`
  (`src/SimulationOutputWriter.cpp`).
- **tinynpy** — `.npy` binary voxel-map I/O, wrapped by `Map3DImpl` for both the hidden input maps
  and the generated output maps.
- **mp-units** — compile-time units/quantities (v2.3+); every physical quantity is typed so the
  compiler rejects mixing X/Y/Z lengths, angles, resolutions, etc. (`include/drone_mapper/Units.h`).
- **gtest** — Google Test, used by the test suites under `tests/` only; it is NOT linked into the
  shipped executables.

## Running

Simulator:

```bash
./build/drone_mapper_simulation [<simulation.yaml>] [<output_path>]
```

`<simulation.yaml>` is a **composition** file. The assignment mandates a **file-reference**
layout where each `simulation_config` / `mission_configs` / `drone_configs` / `lidar_configs`
entry is a **path to a separate YAML file** (each wrapped under its own `*_config:` key). The
sample dataset in `inputs/` is in this format; relative paths inside it resolve against the
composition file's directory, so run it as:

```bash
cd inputs && ../build/drone_mapper_simulation sim_compose.yaml ../out
```

The repo-root `simulation.yaml` uses an older *inline* layout (config maps written directly) and
is kept as a minimal single-scenario example. Runs are the cartesian product
`[mission_configs] × [drone_configs] × [lidar_configs]` per simulation. Output (`simulation_output.yaml`
+ `output_results/`) is written under `<output_path>`; the input and output formats are detailed below.

Argument resolution:

- `<simulation.yaml>` — missing → `simulation.yaml` in the CWD; a filename or relative path is
  resolved against the CWD; an absolute path is used as-is.
- `<output_path>` — missing → the CWD. All output is written under this directory.

Maps comparison utility:

```bash
./build/maps_comparison <origin_map> <target_map> [comparison_config=<path>]
```

Prints only the comparison score (0..100) to stdout, or `-1` to stdout with a message on stderr on
any error. Without a `comparison_config`, both maps are assumed to share the same offset, boundaries,
and resolution (so their shapes must match). With a `comparison_config` YAML (`original`/`target`,
each with `map_res_cm` / `map_offset` / `map_boundaries`) each map uses its own geometry — which also
supports comparing maps of **different resolutions** (a bonus; `MapsComparison::compare` samples each
map by world position).

## Input format

The composition file lists the scenarios to run (its file-reference layout and cartesian-product
semantics are described under **Running**). Relative paths — both the referenced config files **and**
the `map_filename` inside a `simulation_config` — resolve against the **composition file's directory**;
e.g. running `inputs/sim_compose.yaml`, the map_filename `"map/scenario_house.npy"` means
`inputs/map/scenario_house.npy`.

Referenced config keys (units: cm and degrees):

- **simulation_config** — `map_filename`, `map_resolution_cm`,
  `initial_drone_position{x_cm, y_cm, height_cm}`, `initial_angle_deg`
  (`0`=east, `90`=south, `180`=west, `270`=north),
  `map_axes_offset{x_offset, y_offset, height_offset}`.
- **mission_config** — `max_steps`, `boundaries{x_boundary / y_boundary / height_boundary{min_cm, max_cm}}`,
  `gps_resolution_cm`, `output_mapping_resolution_factor`
  (integer; missing → 1; < 1 → ignored with an error log).
- **drone_config** — `dimensions_cm` (sphere **diameter**; radius = half), `max_rotate_deg`,
  `max_advance_cm`, `max_elevate_cm`.
- **lidar_config** — `z_min_cm`, `z_max_cm`, `d_cm`, `fov_circles`.

**Map files:** input and output maps are `.npy` binary NumPy 3D arrays (one byte per voxel). The
output map may be written at a different resolution than the input map.

## Output format (under `<output_path>`)

### `simulation_output.yaml`

A hierarchical score report nested under a top-level `score_report:` key. The report identifies
each level by its **values** (not by config file paths): a simulation by its hidden-map filename, a
mission by its parameters, and a run by its unique output-map filename.

- `composition_file` — the source composition path
- `generated_at_utc` — ISO-8601 UTC timestamp of the run
- `metric` — scoring metric name (occupied-voxel IoU, 0..100)
- `score_range` — `{min: 0, max: 100}`
- `error_score` — `-1` (the score assigned to a failed run)
- `summary` — `total_runs`, `scored_runs`, `error_runs`, `average_score`, `min_score`,
  `max_score` (statistics are over scored runs only; error runs are excluded)
- `simulations` — one entry per simulation:
  - `map_filename` — the hidden map file
  - `map_resolution_cm` — hidden-map voxel size
  - `map_offset` — `{x_cm, y_cm, height_cm}`
  - `missions` — one entry per mission:
    - `max_steps` — mission step budget
    - `output_mapping_resolution_factor` — requested factor
    - `resolution_cm` — the **actual** output resolution used
    - `resolution_request_status` — `ACCEPTED` | `IGNORED` | `IGNORED TOO SMALL`
    - `runs` — one entry per (drone × lidar), each with:
      - `output_map` — the run's output-map filename (its unique identifier)
      - `status` — `completed` | `max_steps` | `error`
      - `steps` — number of steps executed
      - `score` — IoU 0..100, or `-1` when the run errored
      - `error_ref: {code: ...}` — present only when the run errored

### `output_results/`

- One `.npy` output map per run: the drone's generated map.
- `errors.log` — the error log. **Every** error is appended here the moment it occurs (never
  deferred): recovered input-file errors, per-run errors, ignored resolution requests, and group
  failures. Format: `[ERROR_CODE] human-readable message`.

### `input_errors.txt` (created only if there are input-file errors)

A short description of every **recovered** input-file error (a missing or bad config value that was
replaced with a reasonable default), same `[ERROR_CODE] message` format, written under
`<output_path>`. If the input files are clean, this file is not created at all. (These errors also
appear in `errors.log`, since every error is logged there too.)

### Error handling

The program never crashes and never calls `exit()` — it always ends by returning from `main`.

- Input-file errors are recovered with sensible defaults and recorded in `input_errors.txt`
  (created only if such errors exist).
- A single run that fails (e.g. an unreadable map, or a drone step that would collide / leave the
  mission bounds) is logged immediately, scored `-1`, and the batch continues with the next run.
- If an entire group cannot run (e.g. a bad map file shared by several runs), every case in that
  group is scored `-1`.
- An unrecoverable error (e.g. a composition file that cannot be parsed at all) is logged, a message
  is printed to the screen, and the program finishes cleanly having run nothing (no
  `input_errors.txt` in this case).

## Testing

Component tests use GTest/GMock and live under `tests/components/`. Build first
(`cmake --build --preset default`), then run the test binary:

```bash
# Run ALL tests
./build/drone_mapper_simulation_test
```

Run a single component suite with `--gtest_filter=<Suite>.*`:

```bash
./build/drone_mapper_simulation_test --gtest_filter='SimulationManager.*'
./build/drone_mapper_simulation_test --gtest_filter='SimulationRun.*'      # also covers MockGPS + MockMovement
./build/drone_mapper_simulation_test --gtest_filter='MissionControl.*'
./build/drone_mapper_simulation_test --gtest_filter='DroneControl.*'
./build/drone_mapper_simulation_test --gtest_filter='MappingAlgorithm.*'
./build/drone_mapper_simulation_test --gtest_filter='MockLidar.*'
./build/drone_mapper_simulation_test --gtest_filter='MapsComparison.*'
```

Handy variations:

```bash
# Several suites at once (colon-separated)
./build/drone_mapper_simulation_test --gtest_filter='DroneControl.*:MockLidar.*'

# A single test
./build/drone_mapper_simulation_test --gtest_filter='MockLidar.BeamCounts'

# List every test without running
./build/drone_mapper_simulation_test --gtest_list_tests

# Or drive the suite through CTest
ctest --test-dir build --output-on-failure
```

## Bonus: supporting different resolution requests

The output map may be written at a resolution different from the input map (mission
`gps_resolution_cm` × `output_mapping_resolution_factor`), reported per mission as `resolution_cm`
/ `resolution_request_status`, and scored against the different-resolution ground truth. See
`bonus.txt`; check with:

```bash
./build/drone_mapper_simulation_test --gtest_filter=BonusResolution.*
```
