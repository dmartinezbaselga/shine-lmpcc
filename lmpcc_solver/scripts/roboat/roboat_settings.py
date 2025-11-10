import control_modules
import helpers

import copy

# ===== HIGH-LEVEL SETTINGS ============ #
# --- Main MPC Parameters --- #
N = 20                      # MPC Horizon
integrator_stepsize = 0.2   # Timestep of the integrator
n_discs = 3  # Number of discs modeling the vehicle collision region

enable_repulsive = False    # Enable repulsive fields in the objective?

# --- Constraint Selection --- #
use_scenario_constraints = False      # Scenario approach
use_ellipsoid_constraints = False    # Ellipsoid obstacle models
use_gaussian_constraints = False    # Hai's Gaussian Constraints

# ROBOAT
enable_ellipsoid_constraints = True
enable_linear_constraints = True
enable_waterregulations_costs = True
enable_rightofway_costs = True

# --- Interface Selection --- #
interfaces = ['Roboat']  # Define the interfaces that this solver can run with
# @Note: With more than one interface, the IDE cannot automatically resolve the interface, giving errors

# --- Ellipsoid Settings --- #
max_obstacles = 2      # Maximum dynamic obstacles to evade with the planner

# === Constraint Definition === #
# ! DO NOT CHANGE THE ORDER ANY PARTS BELOW ! (i.e., parameters first, then inequalities!)
params = helpers.ParameterStructure()
modules = control_modules.ModuleManager(params)

modules.add_module(control_modules.ContouringModule(params))  # Track a reference path

# All added weights must also be in the .cfg file to function with the lmpcc_controller.cpp code!
weight_list = list()
weight_list.append('Winput_forward')
weight_list.append('Winput_sideways')
weight_list.append('Margin_normal_ellipsoid')
weight_list.append('a_ellipse_factor')
weight_list.append('b_ellipse_factor')
weight_list.append('c_ellipse_factor')
weight_list.append('d_ellipse_factor')
weight_list.append('e_ellipse_factor')
weight_list.append('f_ellipse_factor')
weight_list.append('W_rightofway')
weight_list.append('W_ellipseregulation')
weight_list.append('Wcontour')
weight_list.append('Wlag')
weight_list.append('Wrepulsive')
weight_list.append('Kv')
weight_list.append('velocity_reference')
weight_list.append('ini_v0')
weight_list.append('Wslack')

weights = helpers.WeightStructure(params, weight_list)


# --- Inequality Constraints --- #
# Parameters for the vehicle collision region
# params.add_parameter("disc_r")
# params.add_multiple_parameters("disc_offset", n_discs)

    # modules.add_module(control_modules.ScenarioConstraintModule(params, n_discs=n_discs))

# === Collect Constraint Data === #
print(params)
npar = params.n_par()
nh = modules.number_of_constraints()

# OLD
# constraints = inequality.Constraints(param_idx)
# vehicle_param_idx = constraints.param_idx
#
# if enable_scenario_constraints:
#     constraints.add_constraint(inequality.LinearConstraints(n_discs, max_scenarios, constraints.param_idx, False))
#
# if enable_ellipsoid_constraints:
#     print("ellipse: {}".format(constraints.param_idx))
#     constraints.add_constraint(inequality.EllipsoidConstraints(n_discs, max_obstacles, constraints.param_idx, False))
#     # offset_param = offset_param + constraints.npar
#
# if enable_linear_constraints:
#     print("vehicle_idx: {}".format(vehicle_param_idx))
#     print("linear: {}".format(constraints.param_idx))
#     constraints.add_constraint(inequality.LinearConstraints(n_discs, max_linearconstraints, constraints.param_idx, True,
#                                                             vehicle_param_idx=vehicle_param_idx))
#
# param_idx = constraints.param_idx
#
# npar = 0
# npar += n_other_param
# npar += n_spline_param
#
# npar += constraints.npar
# nh = constraints.nh