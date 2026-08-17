Scenario: office floor, full-region mapping (inputs' small "_out" scenario, verbatim)
-------------------------------------------------------------------------------------
Source : inputs/simulation/small_simulation_out.yaml + inputs/mission/small_mission_out.yaml
Map    : scenario_small.npy (20x20x20 @ 10 cm); output written at 5 cm (gps_resolution 5)
Result : status max_steps, 2000 steps, score ~11.2 (occupied-voxel IoU). See original_output/.
Shows  : mapping a whole office floor; parts stay unmapped within the step budget (inaccessible/occluded).
