# Assignment 3 - Drone Mapper

This is the core skeleton for assignment 3. You should update this README file.

Use the lowercase project namespaces `common`, `algorithm`, `mission_control`, and `simulator` in your implementation.

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
