# Changelog

## 2022-11-21

### General
- Renamed bash files to prepend a number for easy indexing.
- Updated documentation to reflect changes and added module documentation.
- Cleaned Readme.md

### Python side
- Added a module for weights, see jackal_settings.py for how to use it. *By configuring weights as a module the objective function is now the same for all systems, just the modules change.*
    - Breaking changes: 
        - Objective modules should contain an extra parameter `weight_list` in which modules can add weights that are updated through rqt_reconfigure. For example, the contouring_weight is now added in `ContouringObjective`.
        - Weights on variables need to be configured in the `<system>_settings.py`. You can configure custom weight functions as a lambda.
        - In `<system>_solver.py`, `solver.objective` should be set to the general objective `objective.objective(...)`. Other objectives should be removed.
- All inequalities and objectives now retrieve variables by name. An example is in `ContouringObjective.get_value()`. If the `required` flag is set to `True` then the code will throw an error if the variable is not part of the selected dynamical model.
- The robot and system used should be set in the `<system>_settings.py` instead of `<system>_solver.py` file.
- The `generate_cpp_files.py` has been updated to generate `cpp` and `h` code instead of only the header.
- Added a `solver_timeout` (see parameter in C++/yaml) that limits the time that the solver can take.


### C++ Side
- Removed dependencies on `BaseModel` and `BaseState`. *This makes the code much more flexible, but also requires that there are no dependencies on the dynamical model in lmpcc_controller*
- Moved `DeployEmergencyStrategy` to `Interface` and implemented a default constant deceleration version in the baseclass.
- Changed the implementation of `Vehicle`. The new version is `VehicleRegion`, which specifies a collision region for one time step. The vehicle regions for all timesteps can be obtained through `OptimizedVehiclePredictions` or `InitialVehiclePrediction` in the generated solver code. *This makes retrieving and working with the vehicle region easier and accomodates different collision regions than a set of discs*.
- Added a `GetPredictedCollisionRegions()` to `DynamicObstacle` which works the same as `VehicleRegion` except that it contains a single disc for each step in the horizon.
- Modules:
    - Added a `rqt_reconfigure()` callback in case you want to customize the rqt_reconfiguration through your module.
    - Added a `GetMethodName()` method for determining the name of the method (for saving data)
    - `OnDataReceived("init")` is now called after initialization, in case you need to update anything after the controller is initialized.
    - Added an `MPCBase` controller module for the weights.
    - `ReferencePath` supports `n_segments` segments, rather than the hardcoded 3 previously used.
    - Scenario modules:
        - Renamed `stage_scenario` to `smpcc` and `trajectory_disc` to `safe_horizon`.
- Added a monitoring class for checking if the interface receives data, see `tools` in the documentation.
- Added a `SignalPublisher` class for publishing signals that can be visualized with JSK plots, see `jackalsimulator_interface.cpp`.
- Set `nr_stages_to_draw` to compute `indices_to_draw` based on the horizon length and the number of stages.