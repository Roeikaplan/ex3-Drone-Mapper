# Assignment 3 — Drone Mapper Simulator

A drone flies through a hidden 3D voxel map, scanning with a simulated lidar, while a pluggable
mapping algorithm builds an occupancy map as the mission runs. The simulator drives every
(plugin × simulation × mission × drone × lidar) combination concurrently and scores the results
against ground truth.

Students: see `students.txt`.

---

## Building

Requires CMake ≥ 3.15, a C++20 compiler, Ninja, and vcpkg with `VCPKG_ROOT` set. The provided
`.devcontainer` sets `VCPKG_ROOT` to `/usr/local/vcpkg`, so inside it these commands work unchanged.

```bash
cmake --preset default
cmake --build --preset default
```

Dependencies come from `vcpkg.json` in manifest mode: `mp-units` (compile-time-checked physical
units), `yaml-cpp`, `tinynpy`, and `gtest`.

Every target is compiled with `-Wall -Wextra -Werror -pedantic` via the `drone_warnings()` function
in the root `CMakeLists.txt`. **Warnings break the build**, by design.

### Artifacts

| Artifact | Location under `build/default/` |
|---|---|
| Simulator executable | `Simulator/simulator_323998450_211633813` |
| Algorithm plugin | `plugins/algorithms/Algorithm_323998450_211633813.so` |
| MissionControl plugin | `plugins/mission_controls/MissionControl_323998450_211633813.so` |

The two plugins are built into separate folders on purpose: both run modes take a *folder* of plugins
to enumerate, so each folder can be handed to the simulator directly.

---

## Running

Arguments are `key=value` and order-free. All are mandatory except `num_threads` and `-verbose`.
Every problem found is reported at once rather than one per invocation.

**Comparative** — one algorithm against every mission control in a folder:

```bash
./build/default/Simulator/simulator_323998450_211633813 -comparative \
  simulation=inputs/sim_compose.yaml \
  mission_control_folder=build/default/plugins/mission_controls \
  algorithm=build/default/plugins/algorithms/Algorithm_323998450_211633813.so \
  num_threads=4
```

**Competition** — one mission control against every algorithm in a folder:

```bash
./build/default/Simulator/simulator_323998450_211633813 -competition \
  simulation=inputs/sim_compose.yaml \
  mission_control=build/default/plugins/mission_controls/MissionControl_323998450_211633813.so \
  algorithms_folder=build/default/plugins/algorithms \
  num_threads=4
```

Results are written into a timestamped directory inside the *varied* plugin folder: one output `.npy`
map per run, one `score_report` YAML per plugin, one mode-level report comparing them, and
`errors.log`. Adding `-verbose` asks the mission control to write a per-run movement trace.

`main` returns on every path and never calls `exit()`. A bad command line, an unreadable composition,
or a plugin that fails to load are all reported rather than fatal.

### Running several plugins at once

Both modes take a *folder*, so "several plugins" just means several `.so` files in it — there is no
extra flag. The build populates `plugins/fixtures/mission_controls/` with four ready to use: two
byte-identical copies of the real mission control, plus the two stubs.

```bash
./build/default/Simulator/simulator_323998450_211633813 -comparative \
  simulation=inputs/sim_compose.yaml \
  mission_control_folder=build/default/plugins/fixtures/mission_controls \
  algorithm=build/default/plugins/algorithms/Algorithm_323998450_211633813.so \
  num_threads=4
```

`MissionControl_Alpha.so` and `MissionControl_Beta.so` are the same library under two names, so they
must land in one `same_results` group while the two stubs separate on step count — an end-to-end check
that grouping works. A plugin's identity is its **filename**; nothing inside the library distinguishes
the copies, which is the property being demonstrated.

Competition mode is the mirror image — `plugins/fixtures/algorithms/` holds three: two copies of the
real algorithm plus the stub.

```bash
./build/default/Simulator/simulator_323998450_211633813 -competition \
  simulation=inputs/sim_compose.yaml \
  mission_control=build/default/plugins/mission_controls/MissionControl_323998450_211633813.so \
  algorithms_folder=build/default/plugins/fixtures/algorithms \
  num_threads=4
```

`Algorithm_Alpha.so` and `Algorithm_Beta.so` are again the same library twice, so they must rank level
on both score and steps, with the stub last.

Note the run count multiplies: with the shipped composition each plugin costs 6 simulation/mission
pairs × 2 drones × 2 lidars = 24 runs, so the comparative example above is 96 runs and the competition
one 72. Point `simulation=` at a smaller composition to iterate quickly.

---

## Architecture

```
simulator_323998450_211633813 (executable, the plugin host)
 ├── SimulationOrchestrator   mode, plugin set, task table, reports
 ├── SimulationManager        : ISimulation      one per plugin pair
 ├── SimulationRunImpl        : ISimulationRun   one per cell
 ├── MockGPS · MockMovement · MockLidar · Map3DImpl · MapsComparison
 └── Registrar                receives plugin self-registrations

MissionControl_….so   MissionControlImpl · DroneControlImpl · ScanResultToVoxels
Algorithm_….so        MappingAlgorithmImpl
common/               frozen interfaces, types, units, registration macros
UserCommon/           our own code needed by more than one project
```

`docs/HLD.md` carries the full class and sequence diagrams and the rationale behind each non-obvious
choice. `project_context.md` records the requirements baseline and every resolved design decision.

### The plugin mechanism

`common/` declares the two registration constructors but never defines them. The Simulator defines
them in `src/RegistrationEntryPoints.cpp`, so a compiled plugin carries an **undefined** symbol that
the dynamic linker resolves against the host executable at `dlopen` time — visible as
`U common::MappingAlgorithmRegistration::…` under `nm -DC`.

This requires the host target to export its dynamic symbols (`ENABLE_EXPORTS` / `-rdynamic`). Without
it the build is completely clean and `dlopen` fails at runtime with "undefined symbol" — a failure no
compiler or linker check catches. The unit-test binary sets the same property, because
`PluginLifecycleTest` loads real fixture plugins.

Loading is **load-then-claim**: a library's static initialisers push factories into the `Registrar`
during `dlopen`, and the registry immediately takes ownership of whatever appeared. A library that
opens cleanly but registers nothing is a *failure*, not a success — it is named in the report's
`errors:` list. The claim compares the registrar's factory count before the `dlopen` with the count
after, so exactly one load may be in flight at a time; `PluginRegistry` holds one mutex for that.

### Plugin lifecycle — loaded on demand, unloaded when done

**No `.so` is loaded up front.** Discovery lists the files in a folder and builds the entire task
table from filenames; a library is `dlopen`ed by the first run that actually needs it, and `dlclose`d
the moment the last run that needs it finishes — usually on a worker thread, mid-batch. No file is
ever loaded twice, and none is ever reloaded after being unloaded. This is the assignment's plugin
bonus; `bonus.txt` describes it and how to check it.

The mechanism is one number. The task table is complete before any thread starts, so the number of
runs that will ever need a given file is known and reserved on its slot up front. Each run gives its
uses back through a `PluginUseGuard` declared *before* the run it guards — so the run, and both plugin
instances, are destroyed first. The release that takes a slot's count to zero destroys the factory and
unmaps the library, and a count of zero *proves* no future run can need it. The slot's state machine
runs one way only — `NotLoaded → Loaded → Unloaded`, or `NotLoaded → Failed` — so a reload is not
something the scheduler avoids, it is something no code path can express.

Every run ends with a summary line, and each results directory gets a `plugin_lifecycle.log` with one
timestamped line per load and unload:

```
plugin lifecycle: discovered=3 loaded=3 dlopen=3 dlclose=3 peak_mapped=2 mapped_at_end=0
```

`peak_mapped` is the number to look at: single-threaded it is 2 — the plugin running and the fixed one
it is paired with — however many plugins the folder holds. Loading everything up front would make it
the size of the folder.

### Teardown order — not negotiable

`Simulator/src/main.cpp` performs four steps in a fixed order:

1. **Join the workers.** Guaranteed structurally: `ThreadPoolExecutor::forEach` joins its pool before
   returning, so no thread can still be inside plugin code afterwards.
2. **Destroy every run.** Each `SimulationRunImpl` is a local of `SimulationManager::runCell`, so
   every plugin instance is gone when its cell finishes.
3. **Destroy the orchestrator**, so nothing is left that could still ask the registry for a factory.
4. **Clear the `Registrar`, then sweep the registry** — `~PluginLibrary` is the only place `dlclose`
   is ever called.

Most libraries are already gone by then: each one is unmapped by its own last run, in the same order
at a smaller scale — the run and its plugin instances die, then the slot's factory, then the handle.
The final sweep catches only a library no run ever needed.

The factory step matters more than it looks. "Objects related to the `.so`" includes the
`std::function` factories themselves, whose callable targets are compiled into the plugin's code
segment; `dlclose` while one is alive unmaps the code its destructor is about to run. This is why
`SimulationRunFactoryImpl` borrows a factory from its slot per call rather than holding a copy — a
copy would pin its library in memory for the whole batch. Getting the order wrong does not fail at the
call site; it segfaults during static destruction *after* `main` has returned 0, with a stack naming
nothing in this project. `PluginLifecycleTest` exists to catch exactly that.

### Threading

| `num_threads` | Workers | Who works | Total live threads |
|---|---|---|---|
| absent or `1` | 0 | main | 1 |
| `N ≥ 2`, ≥ 2 tasks | `min(N, tasks)` | workers; main blocks in `join()` | `1 + min(N, tasks)` |
| `N ≥ 2`, ≤ 1 task | 0 | main | 1 |

`N` counts threads *in addition to* main, so the total is never exactly 2.

The third row resolves an ambiguity the assignment leaves open. "Cap at `min(N, tasks)`" and "the
total is never exactly 2" collide at exactly one task, where the cap alone would give one worker plus
a blocked main. Falling back to the calling thread satisfies both and costs nothing — a lone worker
while main waits in `join()` does no more work than main would have done itself. The invariant that
falls out, asserted directly in the tests, is that **the worker count is never exactly 1**.

The whole rule lives in one predicate, `ThreadPoolExecutor::workerCountFor`, so it can be tested as a
table rather than inferred from behaviour; `main` never branches on `num_threads`.

Work is distributed by a single `std::atomic<size_t>` cursor over a dense task table built before
execution begins — no queue, no condition variable, no result mutex. Because every cell of every
plugin pair goes into **one** table, no barrier falls between plugins and the pool stays saturated to
the end. Results are written to index-addressed slots, so **reports are byte-identical at any thread
count**; this is verified by diffing a full competitive run at 4 threads against a serial baseline.

Shared mutable state is small and deliberate: the `ErrorLogger` and the `PluginLifecycleLog` (each
mutex-protected, all sinks under one lock), `std::cout` (only written before and after execution), and
the plugin registry's load path — one mutex taken once per plugin, not once per run, because the
load-then-claim inference needs exactly one load in flight. Everything else is per-run and reachable
from one `SimulationRunImpl`.

---

## How `same_results` is defined

Comparative mode groups mission controls that behaved identically. **Two mission controls share a
group only when their ordered sequence of per-run `(score, steps)` is identical** — that is, they
match run by run, in the order the composition expands to.

Equality is deliberately *not* based on the total score. The assignment's own sample shows
`total_score: 495` appearing in two different groups, so an aggregate comparison would merge mission
controls the report is meant to distinguish. Comparing run by run also separates two mission controls
that reach the same total by different routes.

Groups are listed by member count descending. Ties in group size are broken by the first member's
name, purely so that two runs over the same data produce byte-identical reports.

A stronger key — adding a digest of each output map — would additionally separate two mission
controls that produce different maps while reporting identical scores and step counts. It is not
implemented: no observed case requires it, and `SimulationResult` is fixed by the provided interfaces,
so carrying a digest would need a parallel side-channel through the whole run pipeline.

---

## Design decisions

**ID-suffixed namespaces and artifacts.** The three projects the assignment names carry the submitter
IDs in their namespaces — `algorithm_323998450_211633813`, `mission_control_323998450_211633813`,
`user_common_323998450_211633813` — so two teams' plugins can coexist in one process. Two are
deliberately bare: `common` was never open to choice, since the frozen registration macros hard-code
`::common::`, and `simulator` is unconstrained because the assignment fixes only the executable's name
and the Simulator is never loaded as a plugin. Build artifacts keep the same ID suffixes, because both
run modes enumerate a folder that may hold many teams' libraries at once.

The isolation this protects against is also enforced at load time: `dlopen` uses `RTLD_LOCAL`, so even
a plugin that ignored the convention cannot collide with ours. The two `StubMissionControl` fixtures are
built from one source and load together as a standing check of exactly that.

**`UserCommon/` holds only real cross-project duplication.** Two headers earned their place:
`BeamGeometry.h` (the beam/heading geometry shared by `MockLidar`, `ScanResultToVoxels`,
`DroneControlImpl` and `MockMovement`) and `VoxelGrid.h` (the single definition of
`ceil(span / resolution)`, shared by the Algorithm and the Simulator). It is header-only and pulled in
by include path rather than being a CMake target, which keeps the submission at four build files.

**Scoring is occupied-voxel IoU × 100, computed simulator-side.** The hidden map never crosses the
plugin boundary, so only the simulator *can* score. Both maps are sampled by world position rather
than by matching indices, so an output map at a different resolution or offset is still scored
correctly. Empty–empty agreement is excluded: most of a voxel world is empty, and counting it would
push every score toward 100 and destroy the metric's ability to rank algorithms. A failed run scores
`-1`, a sentinel outside the metric's own 0–100 range so it can never be mistaken for a poor result.

**The algorithm carries the run's collision safety.** The frontier search traverses only cells already
proven `Empty` by an actual scan, so every path it emits is through space the drone has seen to be
free. `Unmapped` and `PotentiallyOccupied` are both excluded — the latter exists precisely because the
sensor could not rule out an obstacle. `DroneControlImpl` additionally checks the whole swept path
rather than only the destination, since a 30 cm advance over a 5 cm grid crosses six voxels.

**The frontier search reuses its working memory.** `FrontierSearchScratch` is not premature
optimisation: with locally-declared arrays the search allocated and zero-filled ~2 MB per call, which
is nearly free serially (the allocator recycles one warm block) but becomes `mmap` churn under a
thread pool — 4 threads ran *slower* than 1. The visit stamp is deliberately one byte and cleared only
when it wraps at 255; a 32-bit stamp measured 47% slower because that array is probed for every
neighbour of every expanded node.

---

## Testing

Three suites, 179 tests, wired into CTest:

```bash
ctest --test-dir build/default --output-on-failure     # all suites

# or run the binaries directly, which is much faster
./build/default/Simulator/simulator_unit_test        --gtest_brief=1   # 133
./build/default/MissionControl/mission_control_unit_test --gtest_brief=1   #  30
./build/default/Algorithm/algorithm_unit_test        --gtest_brief=1   #  16
```

`PluginLifecycleTest` is the only suite that `dlopen`s real libraries; it uses purpose-built fixture
plugins, including one that loads cleanly and registers nothing. The fixtures live under
`build/default/plugins/fixtures/`, with the deliberately-broken one in its own folder so it can never
pollute a folder an end-to-end run enumerates. Both `fixtures/mission_controls/` and
`fixtures/algorithms/` additionally receive two copies of the corresponding real plugin at build time,
so each folder doubles as the multi-plugin target described under
[Running several plugins at once](#running-several-plugins-at-once).

---

## Known limitations

**Drone radius is not enforced.** `DroneConfigData::radius` is parsed and stored but never used; the
drone is validated as a dimensionless point, so it can pass within less than its own radius of a known
wall. Widening the swept-path check to a sphere would be correct but costs ~27× the lookups and may
refuse gaps the algorithm deliberately planned through. Deliberately deferred.

**Thin walls can go unscored at differing resolutions.** Scoring samples ground-truth cell centres
while lidar beams stop at a surface's near face, so a wall thinner than the resolution gap between the
hidden map and the output map may never be registered from one side. This affects absolute scores, not
the ranking between algorithms.

**Timing measurements on a shared container are noisy.** Wall-clock benchmarks here vary by roughly
20% run to run. Comparisons should be made back to back in a single pass, and CPU time is a steadier
signal than wall time.

---

## Repository layout

```text
.
├── Algorithm/                     MappingAlgorithmImpl → Algorithm_….so
│   ├── CMakeLists.txt
│   ├── include/Algorithm/ · src/ · tests/
├── MissionControl/                MissionControlImpl, DroneControlImpl, ScanResultToVoxels
│   ├── CMakeLists.txt
│   ├── common_mission_control/include/MissionControl/IDroneControl.h
│   ├── include/MissionControl/ · src/ · tests/
├── Simulator/                     the plugin host
│   ├── CMakeLists.txt
│   ├── common_simulator/include/Simulator/   ISimulation, ISimulationRun,
│   │                                         ISimulationRunFactory, SimulationTypes
│   ├── include/Simulator/ · src/
│   └── tests/                     unit tests + fixtures/ (stub plugins)
├── UserCommon/include/UserCommon/ BeamGeometry.h, VoxelGrid.h   (header-only)
├── common/                        provided, frozen — never modified
│   ├── CMakeLists.txt
│   └── include/Common/            I*.h, types/, Units.h, registration macros
├── inputs/                        sim_compose.yaml + simulation/ mission/ drone/ lidar/ map/
├── docs/                          HLD.md, BUILD_ORDER.md
├── CMakeLists.txt                 builds all four subprojects
├── CMakePresets.json
├── README.md
├── students.txt
└── vcpkg.json
```
