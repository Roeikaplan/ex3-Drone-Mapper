# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Read these first: `project_context.md` and `docs/HLD.md`

`project_context.md` (repo root) is the **baseline for Assignment 3** and takes precedence over this
file wherever the two disagree. Read it before answering a question, planning architecture, or writing
code here. It records what this file does not: the Assignment 3 requirements (simulator run modes, CLI,
report formats, threading rules), the plugin/registration design decisions and the reasoning behind
them, the constraints checklist every change is verified against, and the open decisions that are still
unresolved. If a request conflicts with something recorded there, say so and ask rather than silently
choosing. When one of its open decisions gets resolved, update it as part of that work.

`docs/HLD.md` is the **architecture** that satisfies that baseline — read it before designing or naming
anything new, and before writing a `.cpp` that has to fit into the object graph. It holds the class and
sequence diagrams (plugin load/registration, task-table dispatch, factory wiring, the mission loop,
scan-to-voxels, scoring, teardown, report writing), the component catalogue with the class names to use,
the ownership/lifetime model, the error-containment ladder, and a rationale section explaining *why* each
non-obvious choice was made. Precedence: `project_context.md` (requirements) > `docs/HLD.md`
(architecture) > this file. If an implementation choice contradicts the HLD, either follow the HLD or
update it in the same change — do not let the two drift.

## What this is

A C++20 skeleton for "Assignment 3 - Drone Mapper": a drone flies through a 3D voxel map, scanning with
a simulated lidar, and a pluggable mapping algorithm builds an occupancy map as the mission runs. A
separate simulator drives many (simulation config x mission config) combinations against ground-truth maps
and scores the results. Required project namespaces are lowercase: `common`, `algorithm`, `mission_control`,
`simulator`.

**Status: this is a skeleton, not a working build.** `common/` (the interfaces/types library) is fully
defined and header-only. `Algorithm/`, `MissionControl/`, and `Simulator/` have their public headers defined
under `include/` and `common_mission_control/`/`common_simulator/` but their `CMakeLists.txt` files and all
`src/` implementation files are TODO placeholders (see the comments in each `CMakeLists.txt`). Expect to be
implementing `.cpp` files and finishing the CMake wiring described below, not just editing existing logic.

## Build system

- CMake >= 3.15, C++20, Ninja generator, dependencies via vcpkg manifest mode (`vcpkg.json`).
- Dependencies: `mp-units` (>=2.3.0, compile-time-checked physical units), `yaml-cpp` (>=0.9.0, config
  parsing), `tinynpy` (reading `.npy` ground-truth maps), `gtest`.
- Requires `VCPKG_ROOT` to be set in the environment (used by `CMakePresets.json`); the provided
  `.devcontainer` image sets this to `/usr/local/vcpkg` automatically, so inside it the commands below work
  with no extra setup.
- Every target must go through the `drone_warnings(target)` CMake function defined in the root
  `CMakeLists.txt` (`-Wall -Wextra -Werror -pedantic`) — treat compiler warnings as build-breaking.

```bash
# configure + build (Ninja, from repo root)
cmake --preset default
cmake --build --preset default

# equivalent manual invocation if presets aren't picked up
cmake -S . -B build/default -G Ninja -DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake
cmake --build build/default
```

There is currently no `enable_testing()`/`add_test()` wiring anywhere in the tree even though `gtest` is a
declared dependency — that needs to be set up as part of implementation work, likely per-subproject.

## Architecture

> This section describes the **skeleton as provided**. For the design being built on top of it — the
> orchestration layer above `ISimulation`, the plugin-pair-bound run factory, the task table and executor,
> the report writers, and the diagrams tying them together — see `docs/HLD.md`.

### Layering and plugin model

`common` is the shared contract layer that everything else depends on: pure interfaces (`I*.h`), value types
(`common/include/Common/types/*.h`), physical-unit aliases (`Units.h`, built on `mp-units`), and two
factory/registration pairs used for runtime plugin discovery:

- `MappingAlgorithmFactory` / `REGISTER_MAPPING_ALGORITHM(class_name)` (in `MappingAlgorithmRegistration.h`)
- `MissionControlFactory` / `REGISTER_MISSION_CONTROL(class_name)` (in `MissionControlRegistration.h`)

The registration macros instantiate a static `*Registration` object whose constructor is expected to record
the factory lambda somewhere a loader can find it. Per the root `Simulator/CMakeLists.txt` TODO, algorithm
and mission-control implementations are built as **shared libraries** and are expected to be **dynamically
loaded** (`dlopen`-style, hence linking `${CMAKE_DL_LIBS}` into the simulator) so their registration-
constructor global objects run and self-register at load time. `common::common` is a header-only
(`INTERFACE`) library that every other target links against.

Key interfaces and where they plug in:

- `IMappingAlgorithm` (constructed with `MappingAlgorithmDependencies`: mission/lidar/drone config + a
  read-only `IMap3D&`) — implementations live under `Algorithm/`, decide the drone's next move/scan via
  `nextStep(DroneState, LidarScanResult*)`, and must not mutate the map directly.
- `IMissionControl` (constructed with `MissionControlDependencies`: configs + `ILidar&`/`IGPS&`/
  `IDroneMovement&`/`IMutableMap3D&`/`IMappingAlgorithm&` + output path) — implementations live under
  `MissionControl/`, drive the mission loop end-to-end via `runMission()`.
- `IDroneControl` (in `MissionControl/common_mission_control/include/MissionControl/IDroneControl.h`) —
  MissionControl-internal drone stepping abstraction (`step()` / `state()`), separate from the
  simulator-facing `IDroneMovement`/`IGPS`/`ILidar` sensors/actuators in `common`.
- `IMap3D` (read-only) / `IMutableMap3D` (adds `set()`/`save()`) — the occupancy map; `VoxelOccupancy` has
  five states (`Unmapped`, `Empty`, `Occupied`, `PotentiallyOccupied`, `OutOfBounds`).
- `ISimulation` / `ISimulationRun` / `ISimulationRunFactory` (under `Simulator/common_simulator/include/`) —
  the simulator's own composition-running abstractions: `ISimulation::run()` takes a whole
  `SimulationCompositionData` (many simulation configs x mission configs x drone/lidar configs) and produces
  a `SimulationManagerReport`; `ISimulationRunFactory::create()` builds one `ISimulationRun` per
  (simulation, mission, drone, lidar) combination.

### Naming: lowercase namespaces, ID-suffixed artifacts

Two separate conventions that are easy to conflate:

- **Namespaces are lowercase**, per `README.md`: `common`, `algorithm`, `mission_control`, `simulator`
  (and `user_common` if a `UserCommon/` folder is added). The assignment PDF asks instead for
  ID-suffixed namespaces (`Algorithm_<id1>_<id2>`); the skeleton README wins — see `project_context.md`
  §2.2 for the reasoning. `common` was never open to choice anyway, since the frozen registration
  macros hard-code `::common::`.
- **Build artifacts keep the PDF's ID suffixes**, because comparative/competitive mode enumerates a
  folder holding many teams' libraries at once: `simulator_323998450_211633813`,
  `MissionControl_323998450_211633813.so`, `Algorithm_323998450_211633813.so`.

One consequence for the `REGISTER_*` macros: they token-paste their argument into an identifier, so a
qualified name like `algorithm::MappingAlgorithmImpl` cannot be passed directly. Declare a global-scope
alias (`using MappingAlgorithmImpl_323998450_211633813 = algorithm::MappingAlgorithmImpl;`) and register that.

### Units

Physical quantities use `mp-units` compile-time-checked types (`common::PhysicalLength`, `HorizontalAngle`,
`AltitudeAngle`, etc., in `Units.h`), not raw doubles — lengths are centimeters, angles are degrees. `Position3D`
uses distinct `XLength`/`YLength`/`ZLength` quantity specs (not interchangeable with each other or with
`PhysicalLength`) so axes can't be accidentally swapped at compile time.

### Config/data flow (YAML -> structs)

Runtime inputs live under `inputs/` as YAML, parsed (via `yaml-cpp`, not yet implemented) into the
`common::types`/`simulator::types` config structs:

- `inputs/sim_compose.yaml` — top-level composition: lists of simulation configs x mission configs to run,
  plus the drone/lidar config sets to cross with them.
- `inputs/simulation/*.yaml` -> `SimulationConfigData` (ground-truth map file/resolution/offset, initial
  drone pose).
- `inputs/mission/*.yaml` -> `MissionConfigData` (max steps, mission boundaries, GPS resolution).
- `inputs/drone/*.yaml` -> `DroneConfigData` (drone radius, max rotate/advance/elevate per command).
- `inputs/lidar/*.yaml` -> `LidarConfigData` (z_min/z_max, beam spacing `d`, `fov_circles`).
- `inputs/map/*.npy` — ground-truth voxel maps (read via `tinynpy`); `inputs/map/npy_to_cw.py` is a
  standalone Python helper (needs `numpy`, `nbtlib`) that converts a `.npy` voxel array into a ClassicWorld
  `.cw` file for visualizing maps in a Minecraft-Classic-compatible viewer — it is not part of the C++ build.

### Repo layout

Each of `Algorithm/`, `MissionControl/`, `Simulator/` follows the same split: a public `include/<Name>/`
for that subproject's own headers, private `src/` for implementation, plus (for MissionControl and
Simulator) a `common_<name>/include/<Name>/` directory holding interfaces shared *within* that subsystem
but not exposed to `common`. `common/` has no `src/` — it's interface/type definitions only.

### `ex2/` — prior assignment, reference only

`ex2/` is a separate, self-contained previous assignment (its own git repo, `CMakeLists.txt`, and
`CLAUDE.md`) checked into this tree, not part of the Assignment 3 build (nothing in the root
`CMakeLists.txt` references it). It's a fully implemented single-static-lib version of essentially the
same drone/lidar/mission/mapping problem (`include/drone_mapper/I*.h`, `MissionControlImpl`,
`DroneControlImpl`, `MappingAlgorithmImpl`, `ScanResultToVoxels`, etc.), useful as a reference for mission
loop / drone-stepping / scan-to-voxel logic. Two things differ from this assignment and shouldn't be
copied verbatim: it uses a single `drone_mapper` namespace/static-lib instead of the required lowercase
`common`/`algorithm`/`mission_control`/`simulator` split, and it has no dynamic-loading/plugin-registration
model (everything is linked directly rather than `dlopen`ed via the `REGISTER_*` macros).
