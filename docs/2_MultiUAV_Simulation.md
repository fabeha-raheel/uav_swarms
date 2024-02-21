# Multi UAV Simulation using ArduPilot

## Modifying Iris Drone Model for Multi-UAV Simulation

Copy the iris drone model files and rename the model directories to distinguish between multiple drones (drones can be numbered as 1, 2, 3, etc).

In each model directory, open the model.config file and modify the name of the model enclosed within the <name></name> tags. This will be name which will be used to search for the model.

In each model directoryy, open the model.sdf file and search for <fdm_port_in></fdm_port_in> and <fdm_port_out></fdm_port_out> tags. You can keep 9002/9003 for drone 1, 9022/9023 for drone 2, 9032/9033 for drone 3 etc.