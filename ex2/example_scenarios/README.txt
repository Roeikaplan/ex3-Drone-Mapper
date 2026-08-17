Example input sets
==================

Three self-contained example input sets for ./drone_mapper_simulation. Each set is drawn VERBATIM
from the provided inputs/ dataset (real simulation/mission/drone/lidar config values and real maps),
arranged in the mandated file-reference composition layout so it can be run on its own:

  <set>/
    sim_compose.yaml           the composition (references the files below by path)
    simulation/scene.yaml      simulation_config  (from inputs/simulation/...)
    mission/mission.yaml       mission_config     (from inputs/mission/...)
    drone/drone.yaml           drone_config       (inputs/drone/drone_small.yaml)
    lidar/lidar.yaml           lidar_config       (inputs/lidar/lidar_long.yaml)
    map/<name>.npy             the hidden input map (.npy binary, from inputs/map/...)
    original_output/           the output WE got when running this set

original_output/ holds the result of our own run:
    original_output/simulation_output.yaml            the score report
    original_output/output_results/<map>_run0.npy     the drone's generated output map

To reproduce a run, write to your OWN output directory so original_output/ is not overwritten:
    cd example_scenarios/<set>
    ../../build/drone_mapper_simulation sim_compose.yaml my_output

The three sets (chosen for distinct scenarios and quick runtime)
---------------------------------------------------------------
1_office_full_floor    - inputs' small_simulation_out + small_mission_out (scenario_small, 20^3).
                         Maps the whole floor; interior/occluded parts stay unmapped within the step
                         budget (status max_steps) - a building with inaccessible parts.

2_office_bounded_room  - inputs' small_simulation_room + small_mission_room (scenario_small).
                         Maps a BOUNDED sub-region (the mission's y-boundary starts at 90 cm, not 0)
                         - a complicated (offset) mapping region rather than the whole map.

3_warehouse_room       - inputs' large_simulation_room + large_mission_room (scenario_big, 30^3).
                         A bounded region (x-boundary starts at 90 cm) inside a larger building.

Note: these missions request gps_resolution_cm 5 while the maps are 10 cm, so the output is written
at 5 cm (resolution_cm: 5, resolution_request_status: ACCEPTED) - a different output resolution than
the input map (see bonus.txt).
