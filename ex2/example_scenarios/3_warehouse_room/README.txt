Scenario: larger building, bounded region (inputs' large "_room" scenario, verbatim)
------------------------------------------------------------------------------------
Source : inputs/simulation/large_simulation_room.yaml + inputs/mission/large_mission_room.yaml
Map    : scenario_big.npy (30x30x30 @ 10 cm); output at 5 cm
Note   : bounded region whose x-boundary starts at 90 cm (non-zero minimum), inside a larger map.
Result : status max_steps, 500 steps, score ~2.6. See original_output/.
