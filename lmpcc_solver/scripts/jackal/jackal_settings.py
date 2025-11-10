import sys

import control_modules
import dynamics 
import inequality
import systems

# trigger dvc

interfaces = ["JackalSimulator", "Jackal", "JackalSocnavbench"]  # Define the interfaces that this solver can run with
multi_solver = False
use_sqp_solver = False  # Note: SQP for scenario has max_it = 1
follower_solver = False

n_segments = -1

if len(sys.argv) <= 2:
    configuration = "ICRA-2023"
    # configuration = "LMPCC"
    # configuration = "GMPCC"  # Default configuration
else:
    configuration = sys.argv[2]

# Set variables that hold for all configurations
n_discs = 1  # Number of discs modeling the vehicle collision region
max_obstacles = 12
n_modes = 1
max_obstacles *= n_modes

params, modules, weight_list, weights = control_modules.initialize_solver()

# Set the MPC weights (Here: same for all configurations)
weights.add("a", weight_names="acceleration")
weights.add("w", weight_names="angular_velocity")

robot = systems.Jackal()  # Defines state bounds and the system name
model = dynamics.SecondOrderAccelerationUnicycleModel(system=robot)

after_dash = configuration.split("-")
if len(after_dash) > 1 and after_dash[1] == "05":
    N = 40  # MPC Horizon
    integrator_stepsize = 0.05  # Timestep of the integrator
    configuration = after_dash[0]
    print("Received " + after_dash[1] + " setting integrator_stepsize = " + str(integrator_stepsize) + " and N = " + str(N))
else:
    # Khaled Comparison
    N = 20  # MPC Horizon
    integrator_stepsize = 0.2  # Timestep of the integrator

# CONFIGURE A SOLVER BASED ON THE SELECTED CONFIGURATION (OBJECTIVES!)
if configuration == "Homotopy":
    multi_solver = True
    n_segments = 20

    # Homotopy guidance trajectory following
    modules.add_module(control_modules.HomotopyGuidanceObjectiveModule(params, weight_list, n_segments))
    params = control_modules.end_objectives_start_constraints(params, weight_list, n_discs)

    modules.add_module(control_modules.LinearizedEllipsoidalConstraintModule(params, n_discs, max_obstacles))

elif configuration == "GMPCC":
    multi_solver = True
    n_segments = 3
    N = 30

    # Objectives
    # weights.add("v", weight_names=["velocity", "velocity_reference"],
    #         cost_function=lambda x, w: w[0] * (x - w[1]) ** 2)

    modules.add_module(control_modules.ContouringModule(params, weight_list, n_segments))
    # modules.add_module(control_modules.GoalTrackingModule(params, weight_list))
    modules.add_module(control_modules.VelocityReferenceModule(params, weight_list))
    # modules.add_module(control_modules.HomotopyGuidanceObjectiveModule(params, weight_list, n_segments, n_discs, max_obstacles))
    params = control_modules.end_objectives_start_constraints(params, weight_list, n_discs)
    # modules.add_module(control_modules.LinearizedEllipsoidalConstraintModule(params, n_discs, max_obstacles))

    modules.add_module(control_modules.HomotopyGuidanceConstraintModule(params, n_discs, max_obstacles, static_obstacles=2, constraint_submodule=control_modules.EllipsoidalConstraintModule))
elif configuration == "GMPCC-Gaussian":
    multi_solver = True
    n_segments = 3
    N = 30

    modules.add_module(control_modules.ContouringModule(params, weight_list, n_segments))
    modules.add_module(control_modules.PathVelocityReferenceModule(params, weight_list, n_segments))

    params = control_modules.end_objectives_start_constraints(params, weight_list, n_discs)

    # Constraints
    modules.add_module(control_modules.HomotopyGuidanceConstraintModule(params, n_discs, max_obstacles, static_obstacles=2, constraint_submodule=control_modules.GaussianConstraintModule))

elif configuration == "GMPCC-Scenario":
    multi_solver = True
    n_segments = 3
    use_sqp_solver = False
    use_scenario_constraints = True
    print_init_bfgs = False

    modules.add_module(control_modules.ContouringModule(params, weight_list, n_segments))
    modules.add_module(control_modules.PathVelocityReferenceModule(params, weight_list, n_segments))

    params = control_modules.end_objectives_start_constraints(params, weight_list, n_discs)

    # Constraints
    # constraint_submodule = (
    #     lambda params, n_discs, max_obstacles: modules.ScenarioConstraintModule(params, n_discs, 24)
    # )
    modules.add_module(
        control_modules.HomotopyGuidanceConstraintModule(
            params, n_discs, max_obstacles, static_obstacles=2, constraint_submodule=control_modules.ScenarioConstraintModule
        )
    )
elif configuration == "ICRA-2023":
    n_segments = 10
    multi_solver = True
    N = 30

    modules.add_module(control_modules.GoalTrackingModule(params, weight_list))
    # modules.add_module(control_modules.VelocityReferenceModule(params, weight_list))

    modules.add_module(
        control_modules.HomotopyGuidanceObjectiveModule(params, weight_list, n_segments, n_discs, max_obstacles)
    )  # Track a reference path

    params = control_modules.end_objectives_start_constraints(params, weight_list, n_discs)

    # modules.add_module(control_modules.EllipsoidalConstraintModule(params, n_discs=n_discs, max_obstacles=max_obstacles))
    modules.add_module(control_modules.LinearizedEllipsoidalConstraintModule(params, n_discs=n_discs, max_obstacles=max_obstacles))

elif configuration == "LMPCC":
    n_segments = 10
    multi_solver = True
    N = 30

    modules.add_module(control_modules.GoalTrackingModule(params, weight_list))
    # modules.add_module(control_modules.VelocityReferenceModule(params, weight_list))

    # modules.add_module(
    #     control_modules.HomotopyGuidanceObjectiveModule(params, weight_list, n_segments, n_discs, max_obstacles)
    # )  # Track a reference path

    params = control_modules.end_objectives_start_constraints(params, weight_list, n_discs)

    # modules.add_module(control_modules.EllipsoidalConstraintModule(params, n_discs=n_discs, max_obstacles=max_obstacles))
    modules.add_module(control_modules.LinearizedEllipsoidalConstraintModule(params, n_discs=n_discs, max_obstacles=max_obstacles))

elif configuration == "Cyberzoo":
    n_segments = 10
    multi_solver = True
    N = 20
    max_obstacles = 5

    modules.add_module(control_modules.HomotopyGuidanceObjectiveModule(params, weight_list, n_segments))  # Track a reference path

    params = control_modules.end_objectives_start_constraints(params, weight_list, n_discs)

    # Note: Linearized version of the ellipsoids
    modules.add_module(control_modules.LinearizedEllipsoidalConstraintModule(params, n_discs=n_discs, max_obstacles=max_obstacles))

elif configuration == "Baseline-LinEllipsoid":
    n_segments = 3

    weights.add("v", weight_names=["velocity", "velocity_reference"], cost_function=lambda x, w: w[0] * (x - w[1]) ** 2, )

    modules.add_module(control_modules.ContouringModule(params, weight_list, n_segments))
    params = control_modules.end_objectives_start_constraints(params, weight_list, n_discs)
    modules.add_module(control_modules.LinearizedEllipsoidalConstraintModule(params, n_discs=n_discs, max_obstacles=max_obstacles))

elif configuration == "Baseline-Ellipsoid":
    multi_solver = True

    n_segments = 3  #

    # weights.add("v", weight_names=["velocity", "velocity_reference"],
    #         cost_function=lambda x, w: w[0] * (x - w[1]) ** 2)

    modules.add_module(control_modules.ContouringModule(params, weight_list, n_segments))
    modules.add_module(control_modules.PathVelocityReferenceModule(params, weight_list, n_segments))

    params = control_modules.end_objectives_start_constraints(params, weight_list, n_discs)
    #
    modules.add_module(control_modules.EllipsoidalConstraintModule(params, n_discs=n_discs, max_obstacles=max_obstacles))
    modules.add_module(control_modules.BoundaryYModule(params, n_discs=n_discs, width=6.0 + 0.65))

elif configuration == "Gaussian":
    n_segments = 3  #

    modules.add_module(control_modules.ContouringModule(params, weight_list, n_segments))
    modules.add_module(control_modules.PathVelocityReferenceModule(params, weight_list, n_segments))

    params = control_modules.end_objectives_start_constraints(params, weight_list, n_discs)
    #
    modules.add_module(control_modules.GaussianConstraintModule(params, n_discs=n_discs, max_obstacles=max_obstacles))
    modules.add_module(control_modules.BoundaryYModule(params, n_discs=n_discs, width=6.0 + 0.65))

elif configuration == "Safe-Horizon":
    N = 20  # MPC Horizon
    integrator_stepsize = 0.2  # Timestep of the integrator
    use_sqp_solver = True
    use_scenario_constraints = True
    print_init_bfgs = False
    n_segments = 3

    weights.add("v", weight_names=["velocity", "velocity_reference"], cost_function=lambda x, w: w[0] * (x - w[1]) ** 2, )

    modules.add_module(control_modules.ContouringModule(params, weight_list, n_segments))
    params = control_modules.end_objectives_start_constraints(params, weight_list, n_discs)

    modules.add_module(control_modules.ScenarioConstraintModule(params, n_discs=n_discs))

elif configuration == "TestGoal":
    N = 30  # MPC Horizon
    integrator_stepsize = 0.2  # Timestep of the integrator
    use_sqp_solver = False

    max_obstacles = 16

    weights.add("v", weight_names=["velocity", "velocity_reference"], cost_function=lambda x, w: w[0] * (x - w[1]) ** 2)

    modules.add_module(control_modules.GoalTrackingModule(params, weight_list))
    params = control_modules.end_objectives_start_constraints(params, weight_list, n_discs)

    modules.add_module(control_modules.EllipsoidalConstraintModule(params, n_discs=n_discs, max_obstacles=max_obstacles))
else:
    raise IOError("Unknown configuration " + configuration)

# === Collect Constraint Data === #
print(params)
npar = params.n_par()
nh = modules.number_of_constraints()

# modules.add_module(control_modules.EllipsoidalConstraintModule(params, n_discs=n_discs, max_obstacles=max_obstacles))
# modules.add_module(control_modules.GaussianConstraintModule(params, n_discs=n_discs, max_obstacles=max_obstacles))
# modules.add_module(control_modules.LinearizedConstraintsModule(params, n_discs=n_discs, max_obstacles=max_obstacles))
