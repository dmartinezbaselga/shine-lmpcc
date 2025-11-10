import sys

import numpy as np

import control_modules
import dynamics
import helpers
import systems

if len(sys.argv) > 2:
    configuration = sys.argv[2]
else:
    # Default!
    configuration = "simple-sim"

interfaces = ['Prius', 'Carla', 'SimpleSim']  # Define the interfaces that this solver can run with
multi_solver = False
use_sqp_solver = False  # Note: SQP for scenario has max_it = 1
follower_solver = False
use_floating_license = False

# Set variables that hold for all configurations
n_segments = 5
n_discs = 3  # Number of discs modeling the vehicle collision region
max_obstacles = 8 # Also change in interface_parameters.yaml
n_modes = 1
max_obstacles *= n_modes

params, modules, weight_list, weights = control_modules.initialize_solver()

# Set the MPC weights (Here: same for all configurations)
weights.add("a", weight_names="acceleration")
weights.add("w", weight_names="angular_velocity")
# weights.add("v", weight_names="velocity")
# weights.add("delta", weight_names="steering_angle")

robot = systems.Prius()

# Some of the settings below can be modified in each configuration!
model = dynamics.BicycleModel2ndOrder(system=robot) # Controls: ax, omega
# model = dynamics.SimpleBicycleModelSteering(system=robot) # Controls: vx, delta
# model = dynamics.UnicycleModel(system=robot)


N = 60  # MPC Horizon
integrator_stepsize = 0.05  # Timestep of the integrator
# dynamic_velocity_reference = False  # Can the velocity reference change over the horizon?

if configuration == "basic": # THIS IS A WORKING CONFIGURATION ON THE PRIUS!
    N = 20  # MPC Horizon
    integrator_stepsize = 0.2  # Timestep of the integrator

    weights.add("v", weight_names=["velocity", "velocity_reference"],
                cost_function=lambda x, w: w[0] * (x - w[1]) ** 2)

    # Reference Path
    modules.add_module(control_modules.ContouringModule(params, weight_list, n_segments))
    params = control_modules.end_objectives_start_constraints(params, weight_list, n_discs)

    # Scenario Constraints
    modules.add_module(control_modules.EllipsoidalConstraintModule(params, n_discs=n_discs, max_obstacles=max_obstacles))

elif configuration == "simple-sim": 
    N = 20  # MPC Horizon
    integrator_stepsize = 0.2  # Timestep of the integrator
    # follower_solver = False

    weights.add("v", weight_names=["velocity", "velocity_reference"],
                cost_function=lambda x, w: w[0] * (x - w[1]) ** 2)

    # Reference Path
    modules.add_module(control_modules.ContouringModule(params, weight_list, n_segments))
    params = control_modules.end_objectives_start_constraints(params, weight_list, n_discs)

    # Scenario Constraints
    modules.add_module(control_modules.EllipsoidalConstraintModule(params, n_discs=n_discs, max_obstacles=max_obstacles))

elif configuration == "GMPCC": 
    multi_solver = True
    N = 30
    integrator_stepsize = 0.2  # Timestep of the integrator

    modules.add_module(control_modules.ContouringModule(params, weight_list, n_segments))
    weights.add("v", weight_names=["velocity", "velocity_reference"],
                cost_function=lambda x, w: w[0] * (x - w[1]) ** 2)
    # modules.add_module(control_modules.PathVelocityReferenceModule(params, weight_list, n_segments))

    params = control_modules.end_objectives_start_constraints(params, weight_list, n_discs)

    modules.add_module(
        control_modules.HomotopyGuidanceConstraintModule(
            params, n_discs, max_obstacles, static_obstacles=2, constraint_submodule=control_modules.EllipsoidalConstraintModule
        )
    )

elif configuration == "2020-demo": # THIS IS A WORKING CONFIGURATION ON THE PRIUS!
    N = 20  # MPC Horizon
    integrator_stepsize = 0.2  # Timestep of the integrator

    use_sqp_solver = True  # Note: SQP for scenario has max_it = 1
    bfgs_init = np.diag(np.array([0.299999, 3.99974, 0.34662, 0.0635453, 1.13043, 0.365221, 0.226986, 0.253899]))

    weights.add("v", weight_names=["velocity", "velocity_reference"],
            cost_function=lambda x, w: w[0] * (x - w[1]) ** 2)

    # Reference Path
    modules.add_module(control_modules.ContouringModule(params, weight_list, n_segments))
    params = control_modules.end_objectives_start_constraints(params, weight_list, n_discs)

    # Scenario Constraints
    modules.add_module(control_modules.ScenarioConstraintModule(params, n_discs=n_discs))


elif configuration == "Test-Config-2x": # Time steps twice as small to improve model accuracy
    T = 4.0
    N = 40  # MPC Horizon
    integrator_stepsize = T / N  # Timestep of the integrator

    use_sqp_solver = True  # Note: SQP for scenario has max_it = 1
    bfgs_init = np.diag(np.array([0.299999, 3.99974, 0.34662, 0.0635453, 1.13043, 0.365221, 0.226986, 0.253899]))

    weights.add("v", weight_names=["velocity", "velocity_reference"],
            cost_function=lambda x, w: w[0] * (x - w[1]) ** 2)

    # Reference Path
    modules.add_module(control_modules.ContouringModule(params, weight_list, n_segments))
    params = control_modules.end_objectives_start_constraints(params, weight_list, n_discs)

    # Scenario Constraints
    modules.add_module(control_modules.ScenarioConstraintModule(params, n_discs=n_discs))

elif configuration == "Test-Config-input-delay":  # Configuration 2 with steering input delay
    # --- Main MPC Parameters --- #
    T = 4.0
    N = 40  # MPC Horizon
    integrator_stepsize = T / N  # Timestep of the integrator

    use_sqp_solver = True  # Note: SQP for scenario has max_it = 1
    bfgs_init = np.diag(np.array([0.299999, 3.99974, 0.34662, 0.0635453, 1.13043, 0.365221, 0.226986, 0.226986, 0.253899]))

    model = dynamics.BicycleModel2ndOrderWithDelay(system=robot)
    model.steering_delay = 1

    weights.add("v", weight_names=["velocity", "velocity_reference"],
            cost_function=lambda x, w: w[0] * (x - w[1]) ** 2)

    # Reference Path
    modules.add_module(control_modules.ContouringModule(params, weight_list, n_segments))
    params = control_modules.end_objectives_start_constraints(params, weight_list, n_discs)

    # Scenario Constraints
    modules.add_module(control_modules.ScenarioConstraintModule(params, n_discs=n_discs))

elif configuration == "2022-dariu-demo":  # Configuration 4 with a complexer model including lateral acceleration
    # Note: Cannot be tested in Carla, acceleration measurements are lacking
    T = 4.5
    N = 30  # MPC Horizon
    integrator_stepsize = T / N  # Timestep of the integrator

    use_sqp_solver = True  # Note: SQP for scenario has max_it = 1
    bfgs_init = np.diag(
        np.array([0.299999, 3.99974, 0.34662, 0.0635453, 1.13043, 0.365221, 0.226986, 0.226986, 0.226986, 0.253899]))

    model = dynamics.BicycleModel2ndOrderWith2Delay(system=robot)
    model.steering_delay = 2

    # Reference Path
    modules.add_module(control_modules.ContouringModule(params, weight_list, n_segments))
    modules.add_module(control_modules.VelocityReferenceModule(params, weight_list))
    params = control_modules.end_objectives_start_constraints(params, weight_list, n_discs)

    # Scenario Constraints
    modules.add_module(control_modules.ScenarioConstraintModule(params, n_discs=n_discs))

elif configuration == "partition":  # Configuration for partitioned samples
    N = 20  # MPC Horizon
    integrator_stepsize = 0.2  # Timestep of the integrator

    weights.add("v", weight_names=["velocity", "velocity_reference"],
                cost_function=lambda x, w: w[0] * (x - w[1]) ** 2)

    # Reference Path
    modules.add_module(control_modules.ContouringModule(params, weight_list, n_segments))
    params = control_modules.end_objectives_start_constraints(params, weight_list, n_discs)

    # Scenario Constraints
    modules.add_module(control_modules.ScenarioConstraintModule(params, n_discs=n_discs))

else:
    raise IOError("Unknown configuration " + configuration)

# === Collect Constraint Data === #
print(params)
npar = params.n_par()
nh = modules.number_of_constraints()
