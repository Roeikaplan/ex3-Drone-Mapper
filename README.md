# Assignment 3 - Drone Mapper

Use the lowercase project namespaces `common`, `algorithm`, `mission_control`, and `simulator` in your implementation.

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

## Provided file tree

```text
.
|-- .devcontainer/...
|-- Algorithm/
|   |-- CMakeLists.txt
|   |-- include/Algorithm/
|   `-- src/
|-- MissionControl/
|   |-- CMakeLists.txt
|   |-- common_mission_control/include/MissionControl/IDroneControl.h
|   |-- include/MissionControl/
|   `-- src/
|-- Simulator/
|   |-- CMakeLists.txt
|   |-- common_simulator/include/Simulator/
|   |   |-- ISimulation.h
|   |   |-- ISimulationRun.h
|   |   |-- ISimulationRunFactory.h
|   |   `-- SimulationTypes.h
|   |-- include/Simulator/
|   `-- src/
|-- common/
|   |-- CMakeLists.txt
|   `-- include/Common/
|       |-- types/
|       |   |-- DroneTypes.h
|       |   |-- LidarTypes.h
|       |   |-- MapTypes.h
|       |   `-- MissionTypes.h
|       |-- IDroneMovement.h
|       |-- IGPS.h
|       |-- ILidar.h
|       |-- IMap3D.h
|       |-- IMappingAlgorithm.h
|       |-- IMissionControl.h
|       |-- IMutableMap3D.h
|       |-- MappingAlgorithmFactory.h
|       |-- MappingAlgorithmRegistration.h
|       |-- MissionControlFactory.h
|       |-- MissionControlRegistration.h
|       |-- Types.h
|       `-- Units.h
|-- .gitignore
|-- CMakeLists.txt
|-- CMakePresets.json
|-- README.md
|-- students.txt
|-- vcpkg-configuration.json
`-- vcpkg.json
```
