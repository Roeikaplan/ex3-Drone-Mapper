# Ex3 Build Order

A dependency-ordered path from empty `src/` folders to a submittable plugin-based, multithreaded
simulator — sequenced so the riskiest unknown gets solved on day one, not week three.

> **Companions:** `project_context.md` is the requirements baseline; `docs/HLD.md` is the architecture
> this plan builds. This file is the *order of work*, not a spec — when a phase says "port X", the shape
> of X is in the HLD.

---

## Order by risk, not by layer

The instinct is to build bottom-up: types, then maps, then the mission loop, then plugins last. Resist it.

Almost everything in the lower layers is a **known quantity** — Ex2 already solved it and you can port the
understanding at a predictable pace. The genuinely unknown parts are the plugin lifecycle, the teardown
ordering, and the concurrency. Those are also the parts that fail at **runtime** rather than compile time,
which makes them expensive to discover late.

So: get an empty plugin loading, registering, running, and unloading *before you write a single line of
domain logic*. Once that skeleton walks, every later phase is filling in a slot you have already proven works.

---

## The ten phases

### 00 — Build: make all four targets compile, empty

- Add `find_package` for `yaml-cpp`, `tinynpy`, `GTest` to the root `CMakeLists.txt` (only `mp-units` is
  there now).
- Add `enable_testing()` at the root and plan per-subproject test targets.
- Write the three subproject `CMakeLists.txt` files against their TODO comments: two `SHARED` libs, one
  executable.
- Set `PREFIX ""` and `OUTPUT_NAME` on the plugins so you get `Algorithm_323998450_211633813.so`, not
  `libAlgorithm_….so`.
- Set `ENABLE_EXPORTS ON` on the simulator executable. Do it now, while you remember why.
- `drone_warnings(target)` on every single target.

```cmake
set_target_properties(Algorithm_323998450_211633813 PROPERTIES
    PREFIX "" LIBRARY_OUTPUT_NAME "Algorithm_323998450_211633813")
set_target_properties(simulator_323998450_211633813 PROPERTIES
    ENABLE_EXPORTS ON)   # -rdynamic: plugins resolve the registration ctor here
```

**Done when** — `cmake --build --preset default` produces one executable and two `.so` files with the exact
mandated names, all from trivial placeholder sources.

---

### 01 — The spike: walking plugin skeleton

This is the highest-value phase in the project. Nothing here touches drones, maps, or YAML.

- `Registrar` — a Simulator-side singleton holding two vectors of factories.
- The `.cpp` defining `MappingAlgorithmRegistration::MappingAlgorithmRegistration` and its MissionControl
  twin. These live **only** in the Simulator and push into the Registrar.
- `PluginLibrary` — RAII over one `dlopen` handle, `dlclose` in the destructor and nowhere else.
- `PluginLoader` — enumerate a folder, load each `.so` serially, claim the newly-appended factory.
- A do-nothing `MappingAlgorithmImpl` that returns `Hover` + `Finished`, and a do-nothing
  `MissionControlImpl` that returns `Completed`. Register both.
- A `main` that loads the folder, prints how many factories arrived, calls each one once, destroys the
  instances, clears the Registrar, and returns.

**Done when** — the run prints the right count, exits cleanly, and shows zero leaks and zero invalid reads
under `valgrind` or ASan. Keep these stub plugins forever; they become your integration-test doubles.

---

### 02 — Shell: CLI parsing and the error log

- `CommandLineArgs`: order-independent `key=value`, no spaces around `=`, mode flag, and **collecting**
  error reporting — every unsupported argument and every missing one named in one pass, not the first failure.
- Validate that file arguments open and folder arguments both traverse *and* contain at least one `.so`.
- Timestamped output directory: `comparative_results_<time>` inside `mission_control_folder`, or
  `competition_<time>` inside `algorithms_folder`.
- `ErrorLogger`, ported from Ex2 but **mutex-guarded from the start** — retrofitting thread safety later is
  how you get interleaved log lines.

**Done when** — every malformed invocation in §3.1 of `project_context.md` prints usage plus a specific
error and returns from `main`. Write these as table-driven unit tests; they are free points.

---

### 03 — Port: Simulator data layer

Four straight ports from Ex2 — rewritten against `simulator::` and `common::`, not copied.

- `CompositionLoader` → `simulator::types`. Drop the inline-layout branch; Ex3 only needs the
  file-reference form. Keep the composition-relative path rebasing, including for the nested `map_filename`.
- `Map3DImpl` — `.npy` read/write, world↔voxel indexing. Note Ex3's `IMap3D` also requires `isInBounds`.
- `MockGPS`, `MockMovement`, `MockLidar` — Simulator-side by design; they are the only things that touch
  ground truth.
- `MapsComparison` — same occupied-voxel IoU. No standalone executable this time.

**Done when** — a unit test loads `inputs/sim_compose.yaml` and asserts 5 groups, 6 sim/mission pairs,
2 drones, 2 lidars, with `map_filename` resolved to a path that actually exists.

---

### 04 — Spine: one plugin pair, end to end

Now join the two halves: real Simulator pipeline, stub plugins from phase 01.

- `SimulationRunFactoryImpl` — **bound at construction to one
  `(MissionControlFactory, MappingAlgorithmFactory)` pair**. This is the whole trick for closing the plugin
  dimension without touching the frozen interfaces.
- Port Ex2's factory logic: full-extent hidden-map bounds, output-resolution resolution, and the
  `map_offset` translation of both the start pose and the mission bounds.
- `SimulationRunImpl` owning everything by `unique_ptr`; `SimulationManager` implementing `ISimulation` over
  the four nested config loops.
- Port `SimulationOutputWriter` — the per-plugin `score_report:` YAML.

**Done when** — the simulator runs all 24 combinations against the stub plugins, writes 24 `.npy` files and
one report, and scores everything near zero without crashing.

---

### 05 — Plugin: the real MissionControl

- `MissionControlImpl` — the step loop, ported. Note it no longer receives the hidden map; that reference is
  gone from `MissionControlDependencies` on purpose.
- `DroneControlImpl` — now lives *inside this `.so`*, built by the MissionControl from its dependencies.
  Movement validation, the per-command limits, and the movement-before-scan ordering all come with it.
- `ScanResultToVoxels` — course-provided in Ex2, absent from Ex3's `common/`. It belongs here, with whoever
  owns drone stepping.
- Honour `dependencies.verbose`: write verbose files if and only if it is set.

**Done when** — a hand-scripted mock algorithm driving the real MissionControl produces the exact voxels you
expect on a 5×5×5 map.

---

### 06 — Plugin: the real Algorithm

- Port the frontier BFS: sweep-scan the current cell, BFS through already-observed `Empty` space to the
  nearest frontier, compile the path into per-step micro-commands, split rotations by `max_rotate`.
- Keep the invariant that makes it safe: **only ever traverse cells observed `Empty`**. Collision safety
  lives here, because nothing downstream can see the walls.
- Read `output_map_` for planning; never mutate it.

**Done when** — full runs on all five `inputs/` scenarios finish with real scores, no collisions, and no run
hitting `max_steps` unnecessarily. This is now feature-complete for one plugin pair.

---

### 07 — Modes: many plugins, two modes

- `SimulationTaskTable` — the dense, pre-computed `plugin × sim × mission × drone × lidar` cell vector.
  Build it whole up front; every cell is knowable in advance.
- Run it serially first. Correct results before concurrent results.
- `CompetitiveReportWriter` — sum score and steps per plugin, sort score DESC then steps ASC, failed plugins
  into `errors:`.
- `ComparativeReportWriter` — the equivalence grouping. Start with the ordered vector of per-run
  `(score, steps)` as the equality key; add an output-map digest only if that proves too coarse. Whichever
  you pick, document it in `README.md`.

**Done when** — pointing competitive mode at a folder holding your real algorithm plus two
deliberately-broken stubs produces a correct ranking and a correct `errors:` list.

**Status: done.** Split in two. 07a shipped both report writers over the existing per-plugin loop;
07b then restructured execution into `SimulationTaskTable` + `ITaskExecutor` + `SimulationOrchestrator`
without changing a single byte of output. The restructure was verified by diffing a full competitive and
a full comparative run against reports captured beforehand: all 96 output maps and both `errors.log`
files identical, and the three YAML reports differing only in `generated_at_utc`.

---

### 08 — Threads: concurrency

Deliberately last among the functional work. A correct serial pipeline plus a dense task table makes this a
small change.

- A single `std::atomic<size_t>` cursor; workers fetch-and-increment. No queue, no condition variable, no
  result mutex.
- The thread rule: missing or `num_threads=1` → the main thread does the work. `N ≥ 2` → `N` threads *in
  addition to* main. Total is never exactly 2.
- Cap the pool at `min(N, task_count)` so no thread is spawned idle.
- Audit for shared mutable state. Expect exactly three: the error log, `std::cout`, and nothing else.

**Done when** — the same composition run at 1, 4, and 16 threads produces byte-identical reports, and TSan
is clean.

---

### 09 — Ship: teardown, tests, packaging

- Sequence teardown explicitly: join workers → destroy all runs → **clear the Registrar's factories** → let
  `PluginLibrary` destructors `dlclose`.
- Walk the hard-constraints checklist in §9 of `project_context.md` line by line.
- Create `UserCommon/` if — and only if — real cross-project duplication appeared. The likely content is the
  world↔voxel geometry shared by `MockLidar` and `ScanResultToVoxels`.
- Write `README.md` (including your `same_results` definition); `students.txt` is already correct; add
  `bonus.txt` if you claim any bonus.
- Zip as `ex3_323998450_211633813.zip`: five folders, four build files, no binaries, no vendored libraries.

**Done when** — you unzip into a clean container, run
`cmake --preset default && cmake --build --preset default`, and both modes run without touching anything.

---

## Pitfalls

### Dynamic linking

**Missing `ENABLE_EXPORTS` fails at runtime, not build time.**
The plugin carries an *undefined* symbol for the registration constructor, resolved against the executable
at `dlopen` time. Without `ENABLE_EXPORTS ON` (`-rdynamic`) everything compiles and links perfectly, then
`dlopen` returns null with an "undefined symbol" message. Always read `dlerror()` and print it.

**The macro token-pastes, so it cannot take a qualified name.**
`REGISTER_MAPPING_ALGORITHM(x)` builds the identifier `register_me_##x`. Passing
`algorithm::MappingAlgorithmImpl` pastes something that is not an identifier. Declare a global-scope alias
and register that: `using MappingAlgorithmImpl_323998450_211633813 = algorithm::MappingAlgorithmImpl;`

**`dlclose` while a `std::function` is still alive.**
The factories the Registrar holds are `std::function`s whose target objects live in the plugin's code
segment. They are "objects related to the `.so`" just as much as the instances are. Clearing the Registrar
is a mandatory step before any `dlclose` — and scope-declaration order alone will get it wrong.

**Load serially, or you inherit a locking problem.**
The load-then-claim pattern — registrar empty, `dlopen` one file, exactly one factory appears, associate it
with that file — is only race-free on a single thread. Load everything on the main thread before any worker
starts and the whole registration path needs no locks at all.

**Static registration objects vanish inside static libraries.**
If you ever build plugin sources into a static library for a test target, the linker drops the registration
object because nothing references it, and the plugin silently registers nothing. Test against the real
`.so`, or use `--whole-archive`.

### Concurrency

**Two total threads is always wrong.**
The rule reads oddly on purpose: `N ≥ 2` means `N` threads *plus* main. If your pool ever totals exactly 2,
you have implemented "N including main" and it is a spec violation.

**Caching plugin instances.**
Create a fresh `IMappingAlgorithm` and `IMissionControl` from the factory for *every* run. Construction must
stay cheap. This is separate from — and does not license — reloading a `.so`, which must never happen.

**A mutex on the results table.**
If you find yourself locking around results, the task table is not dense enough. Every cell is known before
the first thread starts; each task writes only its own cell.

**Interleaved log lines.**
The error log and `std::cout` genuinely need a lock. Build it in at phase 02, before there is anything to
interleave, rather than debugging shredded output at phase 08.

**An exception escaping a worker thread terminates the process.**
Wrap every task body in `try`/`catch(...)`. There is no unwinding, no report, and no second chance.

### Porting from Ex2

**Handing MissionControl the hidden map.**
Ex2's `MissionControlImpl` held `const IMap3D& hidden_map_`. Ex3's `MissionControlDependencies` deliberately
does not, because a third-party plugin must not see ground truth. If your port has that member, you have
copied rather than rewritten.

**Dangling references in the dependencies structs.**
Both dependency structs hold *references* to configs. Whatever the Simulator passes must outlive the run —
keep the configs owned by `SimulationRunImpl` or by the composition, never construct them as temporaries at
the factory call.

**A hidden map with default bounds scores a false 100.**
Scoring walks the origin map's grid. Give the loaded ground-truth array real full-extent boundaries from
`Shape() × resolution`, or an empty grid yields a perfect score for a blank output map.

**Forgetting the `map_offset` translation.**
Configs express the start pose and mission bounds relative to the map origin; the grids are anchored at
`map_axes_offset`. Translate both into world coordinates or the drone spawns outside the map —
`house_simulation.yaml`'s `height_offset: 150` will catch you.

**`output_mapping_resolution_factor` is a `double` here.**
Ex2's PDF described it as an integer. The Ex3 header says `double`. Follow the header.

**A run-index counter for output filenames.**
Ex2 used `run_index_++` on the factory. Under threading that is both a data race and a determinism hazard.
Derive the filename from config identity instead — see `docs/HLD.md` §11.8.

### Assignment rules

**Touching `common/`.** Never modify it, never add files to it. If something feels like it belongs there, it
belongs in `UserCommon/`.

**`exit()` anywhere.** Every failure path returns from `main`. Bad arguments, missing files, unloadable
plugins, failed runs — all of them log, degrade, and continue or return.

**Warnings.** `-Werror` on every target via `drone_warnings`. `mp-units` plus `-pedantic` can be noisy on
template-heavy code; deal with it early rather than at packaging time.

---

## If you split the work

Phases 00–02 are hard to parallelize and worth doing together — they set conventions everything else
inherits. After that the seam is clean, because the `.so` boundary is a real interface contract:

| Track | Phases | Owns |
|---|---|---|
| Simulator | 03, 04, 07, 08 | composition loading, maps, mocks, run factory, task table, modes, reports, threading |
| Plugins | 05, 06 | MissionControl + DroneControl + scan-to-voxels, and the mapping algorithm |

The stub plugins from phase 01 are what make this work: the Simulator track keeps building against them
while the plugin track develops independently, and neither blocks on the other.
