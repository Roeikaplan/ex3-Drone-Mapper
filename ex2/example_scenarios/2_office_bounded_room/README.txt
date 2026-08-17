Scenario: office, BOUNDED sub-region (inputs' small "_room" scenario, verbatim)
-------------------------------------------------------------------------------
Source : inputs/simulation/small_simulation_room.yaml + inputs/mission/small_mission_room.yaml
Map    : scenario_small.npy (20x20x20 @ 10 cm); output at 5 cm
Note   : the mission maps a bounded region whose y-boundary starts at 90 cm (a non-zero minimum),
         i.e. a sub-volume of the map rather than the whole thing - a "complicated boundaries" case.
Result : status max_steps, 1000 steps, score ~4.9. See original_output/.
