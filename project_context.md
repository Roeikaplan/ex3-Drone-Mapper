# Project Context — Assignment 3 (TAU Advanced Topics in Programming, 2026B)

> Internal baseline document for **Assignment 3 — Drone Mapper Simulator (multithreaded, plugin-based)**.
> Sources: `Advanced Topics TAU 2026B - Assignment 3.pdf`, the Ex3 skeleton in this repo, the Ex2
> context (`ex2/project_context.md`), the Ex2 HLD (`ex2/docs/HLD.md`), and the course's
> design-reasoning note on the Simulator / MissionControl / MappingAlgorithm / common split.
>
> **Submission deadline: Sep 6th, 2026, 23:30.**

---

## ⚠️ CRITICAL INSTRUCTION FOR CLAUDE

**This file is the core baseline. Before answering any question, planning any architecture, or
writing any code in this repository, review this document and align the response with it.**

It is internal context, not a work order: treat it as the source of truth for *requirements and
agreed decisions*, but act only on the user's explicit instruction for the current task. If a request
conflicts with something written here, say so and ask — do not silently pick one.

When a decision listed in [§8 Open Decisions](#8-open-decisions--conflicts-to-resolve) gets resolved,
update this file as part of that work.

---

## 🎯 CURRENT STATE & WORKING DIRECTIVE

> **Read this section first in every new session and adopt its mindset immediately.**

1. **FOCUS — Assignment 3 only.** Ex1/Ex2 material appears here purely as carried-over domain
   background. The active deliverable is the three-project, plugin-based, multithreaded simulator.

2. **THE REPO IS A SKELETON, NOT A BUILD.** `common/` is complete and header-only. `Algorithm/`,
   `MissionControl/`, and `Simulator/` have *no* `src/` implementations and their `CMakeLists.txt`
   files are one-line TODO comments. Expect to author `.cpp` files and finish the CMake wiring, not
   to edit existing logic. See [§7](#7-repo-state--whats-done-vs-todo).

3. **IMPLEMENTATION STRATEGY — port the *understanding*, rewrite the *code*.** `ex2/` is a fully
   working single-static-library solution to the same drone/lidar/mission/mapping problem. Use it as
   a reference for algorithmic and geometric logic (mission loop, drone stepping, scan-to-voxel,
   ray marching, `.npy` I/O, YAML parsing, scoring). Do **not** copy files verbatim: Ex2 lives in one
   `drone_mapper` namespace and one static lib, and has no dynamic-loading or registration model.
   Every Ex3 file must be written fresh against the Ex3 namespaces, the three-project split, and the
   `.so` plugin boundary.

4. **NEVER MODIFY `common/`.** The PDF states the course-staff `common` files are used *as-is* and
   that you may not add your own files there. The registration headers in particular are frozen.

5. **STRICT INTERFACE ADHERENCE.** Interfaces are unchanged from Assignment 2. Use the exact provided
   types and signatures; do not add parameters to `ISimulation`, `ISimulationRun`,
   `ISimulationRunFactory`, `IMissionControl`, `IMappingAlgorithm`, `IMap3D`/`IMutableMap3D`,
   `ILidar`, `IGPS`, `IDroneMovement`, or `IDroneControl`. Anything Ex3 needs beyond them
   (plugin identity, run mode, threading, reports) lives in **new Simulator-side classes wrapped
   around** those interfaces — see [§5](#5-the-key-architectural-gap-and-how-we-close-it).

---

## 1. What Assignment 3 Adds (the delta in one page)

Assignment 2 produced **one executable** that ran a cartesian product of simulation configs against
**one** statically-linked MissionControl and **one** statically-linked MappingAlgorithm.

Assignment 3 splits that executable into **three independently built projects** and makes the
MissionControl and the Algorithm **dynamically loaded `.so` plugins**, so that *any team's* Simulator
can run *any team's* MissionControl with *any team's* Algorithm. The Simulator then runs many
(plugin × configuration) combinations **concurrently on a thread pool**, in one of two modes:

| Mode | Fixed | Varied | Question it answers |
|---|---|---|---|
| **Comparative** | one Algorithm `.so` | every MissionControl `.so` in a folder | Which mission controls behave *identically*? |
| **Competitive** | one MissionControl `.so` | every Algorithm `.so` in a folder | Which algorithm scores *best*? |

The three genuinely new problems, in order of difficulty:

1. **Plugin lifecycle** — `dlopen` → self-registration via global constructors → factory capture →
   instance creation → complete teardown → `dlclose` in the correct order.
2. **Concurrency** — a lock-free-as-possible task table over (config × plugin) cells, with a thread
   budget defined by an unusual `num_threads` rule.
3. **Two new aggregate report formats** — including an equivalence-grouping ("same_results") pass
   for comparative mode that Ex2 had no analogue for.

Everything else — the drone, the lidar, the maps, the scoring, the YAML config layout — carries over
from Ex2 essentially unchanged.

---

## 2. Mandated Structure, Naming, and Namespaces

### 2.1 Folders (5 required by the PDF)

| Folder | Build product | Contains |
|---|---|---|
| `Simulator/` | executable `simulator_323998450_211633813` | Simulator class, mocks, loaders, registrars, reports |
| `MissionControl/` | shared lib `MissionControl_323998450_211633813.so` | MissionControl + its own DroneControl |
| `Algorithm/` | shared lib `Algorithm_323998450_211633813.so` | the mapping algorithm |
| `common/` | *(no makefile)* — header-only `common::common` | course-staff files, **used as-is, never extended** |
| `UserCommon/` | *(no makefile)* | **our** files needed by more than one project |

`UserCommon/` **does not exist in the skeleton yet** and must be created if we share code across
projects. The skeleton instead ships two *subsystem-scoped* common folders, which are a different
thing and are **not** substitutes for it:

- `MissionControl/common_mission_control/include/MissionControl/` — `IDroneControl.h`; shared *within*
  the MissionControl subsystem.
- `Simulator/common_simulator/include/Simulator/` — `ISimulation.h`, `ISimulationRun.h`,
  `ISimulationRunFactory.h`, `SimulationTypes.h`; shared *within* the Simulator subsystem.

The design-reasoning note explains why: interfaces used by *more than one project* (`IMissionControl`,
`IMappingAlgorithm`, `IMap3D`, sensors) live in `common/`; interfaces used by *one* subsystem's
internals (`IDroneControl`, `ISimulation*`) live in that subsystem's own common folder. The mocks
(`MockLidar`, `MockGPS`, `MockMovement`, `Map3DImpl`) live in `Simulator/src/` **on purpose** — they
are simulation fictions that a real deployment would replace with real drivers.

### 2.2 Namespaces — **DECIDED: lowercase** (`common`, `algorithm`, `mission_control`, `simulator`)

There were two conflicting instructions:

- The **PDF** asks for unique, ID-suffixed namespaces (`Algorithm_<id1>_<id2>`,
  `MissionControl_<id1>_<id2>`, `UserCommon_<id1>_<id2>`) so two teams' plugins can coexist.
- The **skeleton `README.md`** says: *"Use the lowercase project namespaces `common`, `algorithm`,
  `mission_control`, and `simulator` in your implementation."*

**Resolved in favour of the README's lowercase scheme.** The skeleton is the artifact the PDF itself
links to and is the more recent instruction, and `common` was never negotiable anyway — the frozen
registration macros hard-code `::common::`. If we create `UserCommon/`, its namespace follows the same
convention as `user_common`.

```cpp
namespace algorithm       { /* Algorithm/       — MappingAlgorithm implementation */ }
namespace mission_control { /* MissionControl/  — MissionControl + its DroneControl */ }
namespace simulator       { /* Simulator/       — executable, mocks, loaders, reports */ }
namespace common          { /* common/          — frozen, course-provided */ }
```

**Filenames keep their ID suffixes — this decision does not touch them.** The README speaks only to
namespaces; the PDF's unique *file* names remain mandatory and are a genuine functional requirement,
since comparative/competitive mode enumerates a folder that will hold many teams' `.so` files at once:

```
simulator_323998450_211633813
MissionControl_323998450_211633813.so
Algorithm_323998450_211633813.so
```

**Uniqueness of the registration symbol** (the concern the PDF's namespace rule was addressing) is
handled two ways instead, both sufficient: plugins are `dlopen`ed with `RTLD_LOCAL`, so their global
registration objects do not collide across libraries; and if extra insurance is wanted, the ID suffix
can ride on the *class* name — `algorithm::MappingAlgorithmImpl_323998450_211633813` — which is exactly the shape
the PDF's own `REGISTER_*` examples use. See [§5.1](#51-registration-mechanics-the-fiddly-part).

### 2.3 Submitter IDs — **CONFIRMED PAIR: `323998450_211633813`**

`students.txt` holds two submitters:

```
Roei Kaplan, 323998450
Saar Guy, 211633813
```

This matches the PDF's two-ID convention directly (`simulator_<id1>_<id2>`, `ex3_<id1>_<id2>.zip`).
**Every ID-bearing name in this project is therefore fixed**, and all of them are spelled out
concretely throughout this document — no `<ids>` placeholder remains:

| Artifact | Name |
|---|---|
| Simulator executable | `simulator_323998450_211633813` |
| MissionControl plugin | `MissionControl_323998450_211633813.so` |
| Algorithm plugin | `Algorithm_323998450_211633813.so` |
| Submission zip | `ex3_323998450_211633813.zip` |

**ID order** follows `students.txt` (Roei first, Saar second) and is applied consistently to all four
names. The PDF mandates no particular order, so this is our convention rather than a requirement —
but it must stay uniform, since the grader matches the zip name against the folder contents.

Namespaces do **not** carry the IDs — they are lowercase, see
[§2.2](#22-namespaces--decided-lowercase-common-algorithm-mission_control-simulator). The one place
the IDs legitimately appear inside code is the global-scope registration alias in
[§5.1](#51-registration-mechanics-the-fiddly-part), which keeps the emitted `register_me_…` symbol
unique.

---

## 3. The Simulator — CLI, Modes, and Outputs

### 3.1 Command line

```
./simulator_323998450_211633813 -comparative simulation=<composition.yaml> \
                                mission_control_folder=<folder> algorithm=<algo.so> \
                                [num_threads=<num>] [-verbose]

./simulator_323998450_211633813 -competition simulation=<composition.yaml> \
                                mission_control=<mc.so> algorithms_folder=<folder> \
                                [num_threads=<num>] [-verbose]
```

Parsing rules (all mandatory unless noted):

- Arguments may appear **in any order**; `=` has **no surrounding spaces**.
- All arguments are mandatory except `num_threads` and `-verbose`.
- **Unsupported argument(s)** → print usage + an error naming **every** unsupported argument, then finish.
- **Missing argument(s)** → print usage + an error detailing **every** missing argument, then finish.
- **File argument** that does not exist / cannot be opened → usage + specific error, then finish.
- **Folder argument** that does not exist, cannot be traversed, **or contains zero `.so` files of the
  expected kind** → usage + specific error, then finish.
- Wording of usage and error text is our choice. All of these "finish" cleanly — **never `exit()`**,
  always return from `main`.

### 3.2 Output directory

| Mode | Created directly under | Named |
|---|---|---|
| Comparative | the given `mission_control_folder` | `comparative_results_<time>` |
| Competitive | the given `algorithms_folder` | `competition_<time>` |

`<time>` must be generated so a fresh run never collides with existing files. If the directory cannot
be created, write a proper error to the screen (and do not crash).

Each results folder contains:

- **All output map files**, uniquely named, with a pattern that makes the owning run obvious.
  Suggested: `<plugin>__<simulation>__<mission>__<drone>__<lidar>.npy`.
- **Error log file(s).**
- **One mode-level YAML report** (formats below).
- **Plus** one Ex2-style `simulation_output.yaml` **per plugin**, with the plugin's name embedded in
  the filename, in the same folder.

### 3.3 Comparative report format

```yaml
comparative_report:
  composition_file: "simulation_compositions.yaml"
  mission_control_folder: "folder"
  generated_at_utc: "2026-05-30T23:31:10Z"

  results_summary:            # sorted by number of agreeing managers, DESCENDING
    - same_results: ["manager1.so", "manager2.so", "manager5.so"]
      total_score: 495
      total_steps: 100
    - same_results: ["manager3.so", "manager6.so"]
      total_score: 502
      total_steps: 124
    - same_results: ["manager4.so"]
      total_score: 495
      total_steps: 101

  errors: ["manager7.so", "manager8.so"]   # could not be loaded / run
```

Note `total_score: 495` appears in two different groups — **grouping is by behavioural equality across
the whole run set, not by total score.** Two managers agree only if they match run-by-run. Our
definition of "same result" is a decision, see [§8.2](#82-definition-of-same_results-comparative-grouping).

### 3.4 Competitive report format

```yaml
competitive_report:
  composition_file: "simulation_compositions.yaml"
  mission_control: "mission_control_filename.so"
  generated_at_utc: "2026-05-30T23:31:10Z"

  results_summary:            # sorted by score DESCENDING, then steps ASCENDING
    - algorithm: "algorithm1.so"
      total_score: 495
      total_steps: 100
    - algorithm: "algorithm3.so"
      total_score: 490
      total_steps: 97
    - algorithm: "algorithm4.so"
      total_score: 490
      total_steps: 113

  errors: ["algorithm2.so", "algorithm5.so"]   # could not be loaded / run
```

`total_score` / `total_steps` are sums over **all** runs of that plugin across the whole composition.
A plugin that failed to load or failed to run is named in `errors:` instead of appearing in
`results_summary`.

---

## 4. Threading Model

The `num_threads` rule is deliberately unusual — read it carefully:

- **Missing, or `num_threads=1`** → single-threaded: the **main thread does the work**.
- **`num_threads=N` where N ≥ 2** → spawn **N threads *in addition to* the main thread**.
- Consequently **the total thread count is never exactly 2.** (1 alone, or 1 + N where N ≥ 2.)
- Never spawn a thread that has nothing to run: with `T` tasks, cap the pool at `min(N, T)`.
- The main thread blocking in `join()` while workers run is explicitly fine.

Design guidance straight from the PDF:

- **Loading all required `.so` files up front is explicitly endorsed.** Doing so on the main thread
  before any worker starts removes all locking from the registration path. (An on-demand
  load-once/unload-when-idle scheme, *without ever reloading*, is worth a **bonus** — but it is
  strictly harder and is not the default plan.)
- **Avoid locking where possible; lock where necessary.** Necessary: appending to a shared error log,
  and any `std::cout` diagnostics. Unnecessary if designed right: results storage.
- **The result table is knowable in advance** — build the full dense
  `plugins × simulations × missions × drones × lidars` table up front and let each task write only
  its own cell. No sparse structure, no mutex on results.
- **Creating Algorithm/MissionControl instances from their factories must be cheap** — create fresh
  instances per run, never cache them. This is *not* the same as reloading the `.so`, which must not
  happen.

**Planned scheduling:** pre-computed dense task vector + a single `std::atomic<size_t>` cursor that
workers fetch-and-increment. That is a natural fit for "table known in advance" and needs no queue,
no condition variable, and no result mutex.

---

## 5. The Key Architectural Gap (and how we close it)

**The provided interfaces have no plugin dimension.** Look at the two signatures:

```cpp
// Simulator/common_simulator/include/Simulator/ISimulation.h
types::SimulationManagerReport run(const types::SimulationCompositionData& composition,
                                   const std::filesystem::path& output_path);

// Simulator/common_simulator/include/Simulator/ISimulationRunFactory.h
std::unique_ptr<ISimulationRun> create(const types::SimulationConfigData&,
                                       const common::types::MissionConfigData&,
                                       const common::types::DroneConfigData&,
                                       const common::types::LidarConfigData&,
                                       const std::filesystem::path& output_path);
```

Neither mentions a mode, a MissionControl, or an Algorithm. Since `common_simulator` interfaces must
not change, **the plugin dimension is closed outside them**, as follows:

> **A `SimulationRunFactoryImpl` instance is *bound at construction* to one
> `(MissionControlFactory, MappingAlgorithmFactory)` pair.** Each `ISimulation::run(...)` call then
> covers the whole composition for exactly one plugin pair, returning one
> `SimulationManagerReport`. A new orchestration layer above `ISimulation` owns the plugin registry,
> the mode, the thread pool, and the aggregation of N reports into the mode-level YAML.

Planned new Simulator-side classes (all under `Simulator/include/Simulator/` + `Simulator/src/`,
none of them touching `common/` or `common_simulator/`):

| Class | Responsibility |
|---|---|
| `CommandLineArgs` | order-independent parsing, multi-error collection, usage text, file/folder validation |
| `PluginLibrary` | RAII around one `dlopen` handle; `dlclose` **only** in its destructor |
| `PluginLoader` | enumerates a folder or takes a single path; loads all `.so` up front on the main thread |
| `Registrar` (singleton) | receives factories from the frozen registration constructors |
| `CompositionLoader` | Ex2's YAML loader, ported to `simulator::types` |
| `SimulationTaskTable` | dense pre-built `(plugin-pair × sim × mission × drone × lidar)` cells |
| `ThreadPool` / `TaskRunner` | atomic-cursor work distribution honouring the `num_threads` rule |
| `ComparativeReportWriter` | equivalence grouping + `comparative_report:` YAML |
| `CompetitiveReportWriter` | score/steps ranking + `competitive_report:` YAML |
| `SimulationOutputWriter` | Ex2's per-plugin `score_report:` YAML, one file per plugin |
| `ErrorLogger` | thread-safe, **logs immediately**, never defers |
| `MapsComparison` | Ex2 scoring, now Simulator-internal (no standalone executable in Ex3) |

### 5.1 Registration mechanics (the fiddly part)

The frozen headers declare a constructor and define a macro:

```cpp
struct MappingAlgorithmRegistration { explicit MappingAlgorithmRegistration(MappingAlgorithmFactory); };
struct MissionControlRegistration  { explicit MissionControlRegistration(MissionControlFactory);  };
```

- **The `.cpp` files implementing those constructors belong to the Simulator project only.** They will
  push the incoming `std::function` into the `Registrar` singleton.
- **The plugin `.so` has an undefined symbol** for that constructor, resolved against the executable at
  `dlopen` time. This **only works if the executable exports its dynamic symbols** — that is exactly
  what the `Simulator/CMakeLists.txt` TODO means. In CMake:
  `set_target_properties(simulator_323998450_211633813 PROPERTIES ENABLE_EXPORTS ON)` (equivalently `-rdynamic`).
  Forgetting this produces a runtime `dlopen` "undefined symbol" failure, not a build error.
- **Load-then-claim pattern:** the registrar's factory list is empty → `dlopen(one .so)` → its global
  registration object's constructor runs and appends exactly one factory → the loader associates that
  newly appended factory with the file it just opened. Doing this serially on the main thread keeps it
  race-free with no locking.
- **Macro name-pasting caveat:** `REGISTER_MAPPING_ALGORITHM(x)` builds an identifier
  `register_me_##x`, so `x` **cannot be a qualified name** — `algorithm::MappingAlgorithmImpl` would
  paste `register_me_algorithm::MappingAlgorithmImpl`, which is not an identifier. With the class in
  the lowercase `algorithm` namespace ([§2.2](#22-namespaces--decided-lowercase-common-algorithm-mission_control-simulator)),
  declare a global-scope alias first and register that:
  ```cpp
  using MappingAlgorithmImpl_323998450_211633813 = algorithm::MappingAlgorithmImpl;
  REGISTER_MAPPING_ALGORITHM(MappingAlgorithmImpl_323998450_211633813);
  ```
  The ID suffix on the alias is free and keeps the emitted `register_me_…` symbol unique, matching the
  shape of the PDF's own examples. (Invoking the macro *inside* the namespace also works — the object
  still has static storage duration — but the PDF's wording says global scope, so prefer the alias.)

### 5.2 Teardown order (`dlclose` is a footgun)

The PDF: *do not call `dlclose` if objects related to the `.so` are still alive.* Objects "related to"
a plugin include not only its instances but the **`std::function` factories themselves**, whose target
objects live in the plugin's code. Required order:

1. Join all worker threads.
2. Destroy every `IMissionControl` / `IMappingAlgorithm` instance (they die with their
   `SimulationRunImpl`, so destroy all runs).
3. **Clear the `Registrar`'s stored factories.**
4. Only then let `PluginLibrary` destructors run `dlclose`.

Declaring the `PluginLoader` *before* the registrar-clearing step in the same scope will get this
wrong; sequence it explicitly.

### 5.3 Per-run object graph (mostly unchanged from Ex2)

`SimulationRunImpl` still owns everything for one run via `unique_ptr`, with inner components holding
non-owning references to siblings. The one structural change is **where DroneControl lives**:

- **Ex2:** `SimulationRunImpl` built `DroneControlImpl` and handed it to `MissionControlImpl`.
- **Ex3:** `MissionControlDependencies` carries raw sensors plus the algorithm, and the header comments
  *"mission will create its own drone controller"*. **`DroneControlImpl` now lives inside
  `MissionControl_323998450_211633813.so`**, built by the MissionControl from its dependencies.

```cpp
struct MissionControlDependencies {
    const types::MissionConfigData& mission_config;
    const types::DroneConfigData&   drone_config;   // mission creates its own drone controller
    ILidar& lidar;  IGPS& gps;  IDroneMovement& movement;
    IMutableMap3D& output_map;   IMappingAlgorithm& mapping_algorithm;
    std::filesystem::path output_map_file;
    bool verbose = false;                            // ← the -verbose plumbing point
};
```

Two consequences worth internalising:

- **MissionControl no longer receives the hidden map.** In Ex2 it did. That is correct and deliberate:
  a plugin must not be able to peek at ground truth. Scoring stays Simulator-side.
- **`verbose` is already in the struct** — the `-verbose` CLI flag flows Simulator → dependencies →
  MissionControl, which writes verbose files *iff* it is set.
- **`ScanResultToVoxels` was course-provided in Ex2 and is absent from Ex3's `common/`.** Whoever owns
  drone stepping owns it — i.e. it moves into `MissionControl/`. Its underlying world↔voxel geometry
  helpers are also needed by `MockLidar` (Simulator side), which is the strongest candidate for the
  first genuine `UserCommon/` content.

---

## 6. Carried-Over Domain Foundation (Ex1/Ex2, unchanged)

The PDF is explicit: **"interfaces are not changed from assignment 2."** Everything here still holds.

### 6.1 Drone and world

- Angle convention: **`0° = east (+X)`, `90° = south (+Y)`, `180° = west (−X)`, `270° = north (−Y)`**.
- Drone is a perfect sphere (`DroneConfigData::radius`); movement is `rotate` / `advance` / `elevate`,
  each capped per command by `max_rotate` / `max_advance` / `max_elevate`.
- No battery, no recharging.
- **The drone must never fly into walls**; collision is a mission failure.
- The hidden ground-truth map is visible **only** to the Simulator's mocks — never to the plugins, and
  never as a source of output-map values.

### 6.2 Lidar

- Circle 0 is a single central beam; each outer circle has **4× the beams** of the previous one, evenly
  spread, with circle spacing `d` measured at `z_min` (so `fov_circles = 5` ⇒ 341 beams).
- `z_min`: below it, a hit is detected but its distance is reported as `0`.
- `z_max`: beyond it, nothing is detected.
- Each beam returns at most one hit (the nearest); blind spots between beams must not exceed the
  required mapping resolution.

### 6.3 Types (`common/include/Common/`)

```
Units.h      PhysicalLength / XLength / YLength / ZLength (cm), HorizontalAngle / AltitudeAngle (deg),
             Position3D{x,y,z}, Orientation{horizontal, altitude}   — mp-units, compile-time checked
MapTypes     VoxelOccupancy{PotentiallyOccupied=-3, OutOfBounds=-2, Unmapped=-1, Empty=0, Occupied=1}
             MappingBounds, MappedVoxel, MapConfig{boundaries, offset, resolution}
MissionTypes MissionConfigData{max_steps, gps_resolution, output_mapping_resolution_factor,
                               mission_bounds}
             MissionRunStatus{Completed,MaxSteps,Error}, ErrorRef{code,message}, MissionRunResult
DroneTypes   DroneConfigData{radius, max_rotate, max_advance, max_elevate}
             MovementCommand, MappingStepCommand{movement?, scan_orientation?, status}
             AlgorithmStatus{Working, Finished, FinishedWithUnmappableVoxels}
             DroneState{position, heading, step_index}, DroneStepResult, MovementResult
LidarTypes   LidarConfigData{z_min, z_max, d, fov_circles}, LidarHit, LidarScanResult = vector<LidarHit>
```

`XLength` / `YLength` / `ZLength` are **distinct quantity specs** — the compiler rejects accidental
axis mixing. Values only leave the type system through the deliberate unit-stripping idiom
(`force_numerical_value_in(cm)` → scale the plain scalar → re-attach the axis quantity spec).

`simulator::types` (in `common_simulator/SimulationTypes.h`) additionally holds
`SimulationConfigData` (moved out of `common` since Ex2), `SimulationCompositionData` (now carrying
`composition_file`), `ResolutionRequestStatus`, `SimulationResult`, and `SimulationManagerReport`.

⚠️ **`output_mapping_resolution_factor` is a `double` in this skeleton**, though the Ex2 PDF described
it as an integer (missing → 1, `< 1` → ignored with an error log). Follow the header's type.

### 6.4 Mission step loop (per Ex2 HLD, still correct)

1. `MissionControl::runMission()` drives the loop; each iteration calls `IDroneControl::step()`.
2. `step()` reads `DroneState` from GPS, then calls `IMappingAlgorithm::nextStep(state, latest_scan)`.
3. **The first step passes `nullptr`** for `latest_scan` — no scan exists yet.
4. A `MappingStepCommand` may request movement, a scan, both, or neither.
5. **If both, movement is validated and executed first**, and the scan is taken from the *updated*
   state.
6. Scan results become voxels and are written into the **output** map; the algorithm may *read* that
   map for planning but must never mutate it directly.
7. On completion the output map is saved to `.npy`; the Simulator scores it against the hidden map.

### 6.5 Input files (`inputs/` — identical dataset to Ex2)

The composition file **references config files by path**; it does not inline them.

```yaml
simulation_compositions:
  simulations:
    - simulation_config: "simulation/house_simulation.yaml"
      mission_configs: ["mission/house_mission_lower.yaml", "mission/house_mission_full.yaml"]
  drone_configs: ["drone/drone_small.yaml", "drone/drone_large.yaml"]
  lidar_configs: ["lidar/lidar_long.yaml", "lidar/lidar_short.yaml"]
```

- Each referenced file wraps its body under its own top-level key: `simulation_config:`,
  `mission_config:`, `drone_config:`, `lidar_config:`.
- **All relative paths — referenced configs *and* each `map_filename` — resolve against the
  composition file's parent directory**, not against the referencing file's own directory.
  (`inputs/simulation/house_simulation.yaml` says `map_filename: "map/scenario_house.npy"`, which is
  `inputs/map/scenario_house.npy`.)
- Runs are the cartesian product `[mission_configs] × [drone_configs] × [lidar_configs]` **within each
  simulation** — missions are bound to their own simulation.
- Ex3 ships **only** the file-reference layout. The inline layout Ex2 also supported (repo-root
  `simulation.yaml`) has no Ex3 equivalent; inline support is optional and not required.
- Ground-truth maps are `.npy` voxel arrays read with `tinynpy`. `inputs/map/npy_to_cw.py` converts
  them to Minecraft-Classic `.cw` for visual inspection and is **not part of the C++ build**.

### 6.6 Error handling (carried over, still binding)

- **The Simulator must never crash.** It need not survive a *crash* inside a MissionControl or
  Algorithm plugin, but every other failure must be contained.
- **Never call `exit()`** or equivalent — always return from `main`.
- **Log every error immediately at the point it occurs; never defer.**
- A scenario that errors but leaves the simulator able to continue scores `-1` and the run proceeds.
- If an entire group cannot run (bad map file, unloadable plugin), fill `-1` for every case in that
  group and name the plugin under the report's `errors:` list.

---

## 7. Repo State — What's Done vs TODO

### Complete and frozen

```
common/CMakeLists.txt                    drone_common INTERFACE + common::common alias → mp-units
common/include/Common/*.h                all interfaces, factories, registration macros, types, units
Simulator/common_simulator/include/…     ISimulation, ISimulationRun, ISimulationRunFactory,
                                         SimulationTypes
MissionControl/common_mission_control/…  IDroneControl
inputs/                                  full sample dataset (5 simulations × missions × 2 drones ×
                                         2 lidars) + 3 .npy maps
CMakePresets.json                        Ninja preset wired to $VCPKG_ROOT
```

### TODO — everything below is unwritten

```
Algorithm/CMakeLists.txt        one-line TODO comment → SHARED lib, link common::common,
                                strip the "lib" output prefix
MissionControl/CMakeLists.txt   one-line TODO comment → SHARED lib, link common::common,
                                private include of common_mission_control/include
Simulator/CMakeLists.txt        one-line TODO comment → executable, link common::common +
                                ${CMAKE_DL_LIBS}, ENABLE_EXPORTS for registration symbols
Algorithm/src/                  empty (.gitkeep only)
MissionControl/src/             empty (.gitkeep only)
Simulator/src/                  empty (.gitkeep only)
Algorithm/include/Algorithm/    empty
MissionControl/include/…        empty
Simulator/include/Simulator/    empty
```

Root `CMakeLists.txt` currently calls `find_package(mp-units CONFIG REQUIRED)` only — **`yaml-cpp`,
`tinynpy`, and `gtest` are declared in `vcpkg.json` but not yet found by CMake.** There is also **no
`enable_testing()` / `add_test()` anywhere**, so test wiring must be added (likely per subproject).

**Every target must go through `drone_warnings(target)`** (`-Wall -Wextra -Werror -pedantic`) — a
warning is a build break.

### Build

```bash
cmake --preset default          # requires VCPKG_ROOT (the devcontainer sets it to /usr/local/vcpkg)
cmake --build --preset default
```

### `ex2/` — reference only, gitignored

`ex2/` is a self-contained, fully implemented previous assignment with its own git repo and CMake. It
is **not** part of the Ex3 build (the root `CMakeLists.txt` never references it) and is excluded via
`.gitignore` alongside `.claude/` and `CLAUDE.md`. Its most reusable logic:

| `ex2/src/` file | What to mine it for |
|---|---|
| `MockLidar.cpp` | ray marching against the hidden map — the trickiest geometry |
| `ScanResultToVoxels.cpp` | scan → voxel conversion and the unit-stripping idiom |
| `Map3DImpl.cpp` | `.npy` read/write, world↔voxel index mapping |
| `CompositionLoader.cpp` | file-reference YAML layout + relative-path resolution |
| `MapsComparison.cpp` | 0–100 scoring, including across differing resolutions |
| `DroneControlImpl.cpp` / `MissionControlImpl.cpp` | step loop, movement validation, ordering rules |
| `MappingAlgorithmImpl.cpp` | BFS frontier exploration, rotation splitting by `max_rotate` |
| `SimulationOutputWriter.cpp` | the `score_report:` YAML that Ex3 still needs per plugin |

---

## 8. Open Decisions & Conflicts to Resolve

### 8.1 ~~Namespace scheme~~ — RESOLVED

Settled in favour of the skeleton `README.md`'s lowercase namespaces (`common`, `algorithm`,
`mission_control`, `simulator`), with ID suffixes retained on filenames and available on class names.
Full reasoning in [§2.2](#22-namespaces--decided-lowercase-common-algorithm-mission_control-simulator).
The remaining naming blocker is the submitter-ID question in [§2.3](#23-submitter-ids--one-submitter-listed-solo-submission-not-yet-confirmed).

### 8.2 Definition of `same_results` (comparative grouping)

The PDF shows two groups with an identical `total_score` of 495, proving equality is **not** score
equality. Candidate definitions, cheapest first:

1. The ordered vector of per-run `(score, steps)` across the whole composition.
2. The above **plus** a digest of each output map.
3. Full output-map equality only.

**Leaning toward (1)**, with (2) as a refinement if it proves too coarse. Whatever we choose gets
documented in `README.md`.

### 8.3 `UserCommon/` — create it, and with what in it?

It does not exist yet. Strongest candidate content is the world↔voxel geometry used by both
`MockLidar` (Simulator) and `ScanResultToVoxels` (MissionControl). Decide once the first genuine
cross-project duplication appears rather than speculatively.

### 8.4 External libraries vs. the submission rules

The PDF forbids submitting external libraries and permits only the standard library plus
forum-approved libraries. We depend on `mp-units`, `yaml-cpp`, `tinynpy`, and `gtest` — all introduced
*by the course skeleton itself*, so they are implicitly sanctioned, **but they must not be vendored
into the zip.** Ship `vcpkg.json` / `vcpkg-configuration.json` and let the grader's toolchain fetch
them. Confirm on the forum if there is any doubt.

### 8.5 Testing scope

Ex3 does not restate Ex2's component/integration test mandate or its bug-detection thresholds, yet
`gtest` remains a declared dependency and no test wiring exists. Assume tests are still expected and
plan per-subproject test targets; confirm the grading rules on the forum.

### 8.6 Bonuses available

- **Class competition** for the best algorithm.
- **Load-once / unload-when-idle plugin management** (never reloading the same `.so`) — explicitly
  offered in the threading section.
- Any bonus request needs a `bonus.txt` describing the addition, how to verify it, and the relevant
  test filter.

---

## 9. Hard Constraints Checklist

Verify every change against this list before calling it done:

- [ ] **No `new` / `delete`** anywhere.
- [ ] **`unique_ptr` by default**; `shared_ptr` only where sharing is real and lifetime is unknown.
- [ ] **Never modify `common/`**, and never add files to it.
- [ ] **Never change a provided interface signature.**
- [ ] **`drone_warnings(target)` on every target**; zero warnings, since they are errors.
- [ ] **The Simulator never crashes** and never calls `exit()`; `main` always returns.
- [ ] **Errors are logged the moment they occur.**
- [ ] **`dlclose` every handle**, and only after all plugin-derived objects *and* the registrar's
      factories are gone.
- [ ] **No cached plugin instances** — recreate from the factory per run; never reload a `.so`.
- [ ] **mp-units types throughout**; no raw `double` for physical quantities.
- [ ] **Thread count obeys the rule**: 1, or 1 + N (N ≥ 2), never exactly 2, never idle threads.
- [ ] **Algorithm minimums**: never fly into walls; map everything in the configured boundaries;
      be efficient and exact.

### Submission package (`ex3_323998450_211633813.zip`)

- 5 folders: `Simulator/`, `Algorithm/`, `MissionControl/`, `common/`, `UserCommon/`
- 4 build files: one inside each of the three project folders + one at the zip root building all three
- `students.txt` — one line per submitter, name and ID
- `README.md` — implementation notes and remarks, at the zip root
- **Excluded:** binary files, external libraries

---

## 10. Documentation Conventions

**Carried over from Ex2 unchanged** (`ex2/CLAUDE.md` §"Documentation Conventions"). Every file
written for Ex3 follows this house style. Ex2 exemplars to match: `ex2/include/drone_mapper/MockGPS.h`
and `MockMovement.h` for headers; `ex2/src/MockMovement.cpp`, `ScanResultToVoxels.cpp`, and
`Map3DImpl.cpp` for sources; `ex2/src/maps_comparison_main.cpp` for file-local helpers.

### 10.1 The five rules

- **Doxygen blocks only — never `//`.** Every comment is a `/** ... */` block carrying `@brief`
  and, where there is a boundary or assumption to state, `@note`. This holds **everywhere**: file
  headers (`@file` + `@brief` + `@note`), classes, methods, file-local helpers, and explanatory
  comments inside a function body (a bare `/** @note ... */` block). `//` line comments are not
  used in this project.
- **Headers** — a Doxygen block on every class and on each public method or notable member:
  `@brief`, `@param`, `@return`, and a `@note` calling out the architectural boundary (who owns a
  responsibility, and why). The `@note` is the part that carries the design, e.g. *"DroneControl owns
  all movement validation because `MockMovement` never validates."*
- **Sources** — concise, *why*-focused blocks explaining intent, never the obvious mechanics.
- **File-local helpers** — free functions and anonymous-namespace helpers get the **same full
  Doxygen block** as header methods, even short throwaway ones: `@brief`, `@param`, `@return`, and a
  `@note` for any assumption or boundary.
- **Density** — match the surrounding file; never restate what the code plainly says.

### 10.2 Things that must always be documented

Carried from Ex2, and still binding wherever the code appears:

- The **unit-stripping idiom**: `force_numerical_value_in(cm)` → scale the plain scalar → re-attach
  the `x_extent[cm]` quantity spec.
- The **angle convention**: `0° = +X east`, `90° = +Y south`.
- The **movement-before-scan ordering**, and the **first-step `nullptr` bootstrap**.
- Any non-obvious branch or invariant: BFS frontier selection and termination, rotation splitting by
  `max_rotate`, evidence-priority voxel merging.

**New in Ex3** — these are the Ex3-specific invariants that a reader cannot infer from the code, so
every file touching them says so explicitly:

- **The plugin's undefined symbol is the mechanism, not a defect.** Any file involved in
  registration states that the constructor is declared in `common/`, defined only in the Simulator,
  and resolved at `dlopen` time — and that this requires `ENABLE_EXPORTS`.
- **Teardown ordering.** Anything holding a plugin-derived object or a factory documents that it must
  be destroyed before `dlclose`, and *why* scope-based ordering cannot be relied on.
- **Load-then-claim.** The loader documents that the file↔factory association is inferred temporally
  and is valid only because loading is serial.
- **Thread-safety posture.** Every shared object states whether it is synchronised and why: the error
  log and `std::cout` are locked; the results table is not, because cells are disjoint; the registrar
  is not, because it is only touched during serial loading.
- **Plugin isolation.** Wherever the hidden map is reachable, note that it must never cross the `.so`
  boundary.

### 10.3 Ex3 exemplars

As Ex3 files land, these become the reference for new code in each layer:

| Layer | Exemplar |
|---|---|
| Simulator header | `Simulator/include/Simulator/Registrar.h` |
| Simulator source | `Simulator/src/PluginLoader.cpp` |
| Plugin implementation | `MissionControl/src/MissionControlImpl.cpp` |
| File-local helpers | `Simulator/src/SimulationRunFactoryImpl.cpp` |
