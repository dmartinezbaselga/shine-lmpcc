import helpers
import control_modules

# --- Main MPC Parameters --- #
N = 20                      # MPC Horizon
integrator_stepsize = 0.05  # Timestep of the integrator
n_discs = 1                 # Number of discs modeling the vehicle collision region

use_sqp_solver = False      # Note: SQP for scenario has max_it = 1
enable_repulsive = False    # Enable repulsive fields in the objective?

# --- Constraint Selection --- #
use_scenario_constraints = False    # Scenario approach
use_ellipsoid_constraints = False   # Ellipsoid obstacle models
use_gaussian_constraints = False    # Hai's Gaussian Constraints

module_selection = 0

if module_selection == 1:
    use_scenario_constraints = True
    method_name_for_recording = 'scenario'
elif module_selection == 2:
    use_ellipsoid_constraints = True
    method_name_for_recording = 'ellipsoid'
elif module_selection == 3:
    use_gaussian_constraints = True
    method_name_for_recording = 'gaussian'

# --- Interface Selection --- #
interfaces = ['HovergamesGazebo']

# --- Scenario Settings --- #
if use_scenario_constraints:
    max_scenarios = 24      # Maximum linear constraints supported in the solver

# --- Ellipsoid/Gaussian Settings --- #
if use_ellipsoid_constraints or use_gaussian_constraints:
    modes = 1
    max_obstacles = 6       # Maximum dynamic obstacles to evade with the planner
    max_obstacles *= modes  # To account for the number of modes, we need an ellipsoid per mode!

# --- SQP Settings --- #
if use_sqp_solver:
    print_init_bfgs = False

# === Constraint Definition === #
# ! DO NOT CHANGE THE ORDER ANY PARTS BELOW ! (i.e., parameters first, then inequalities!)
params = helpers.ParameterStructure()
modules = control_modules.ModuleManager(params)

modules.add_module(control_modules.ContouringModule(params))  # Track a reference path

# All added weights must also be in the .cfg file to function with the lmpcc_controller.cpp code!
weight_list = list()
weight_list.append('input_angles')
weight_list.append('input_dpsi')
weight_list.append('input_thrust')
weight_list.append('contour')
weight_list.append('lag')
weight_list.append('velocity')
weight_list.append('velocity_reference')
weight_list.append('slack')

# Not used currently
weight_list.append('repulsive')
weights = helpers.WeightStructure(params, weight_list)

# --- Inequality Constraints --- #
# Parameters for the vehicle collision region
params.add_parameter("disc_r")
params.add_multiple_parameters("disc_offset", n_discs)

if use_scenario_constraints:
    modules.add_module(control_modules.ScenarioConstraintModule(params, n_discs=n_discs))
#
if use_ellipsoid_constraints:
    modules.add_module(control_modules.EllipsoidalConstraintModule(params, n_discs=n_discs, max_obstacles=max_obstacles))

if use_gaussian_constraints:
    modules.add_module(control_modules.GaussianConstraintModule(params, n_discs=n_discs, max_obstacles=max_obstacles))

# === Collect Constraint Data === #
print(params)
npar = params.n_par()
nh = modules.number_of_constraints()
