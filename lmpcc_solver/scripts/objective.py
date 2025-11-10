"""
Objective.py defines the objectives for the solver. Selected objectives are configured through the ControlModules.
"""

import numpy as np
from helpers import SplineParameters, rotation_matrix
import casadi

def sigmoid(x, x0, k):
    return 1.0 / (1.0 + casadi.exp(-k * (x - x0)))


def glue_functions(x, x_glue, func_1, func_2, k=10):
    s = sigmoid(x, x_glue, k)
    return (1.0 - s) * func_1(x) + s * func_2(x)

def huber_loss(value, reference, delta):
    """
        value: symbolic value
        reference: reference value
        delta: treshold to switch from quadratic to linear
    """
    # Calculate the residual
    residual = value - reference
    
    # Define the conditions for Huber loss
    small_error_loss = lambda x: 0.5 * x**2
    large_error_loss = lambda x: delta * (casadi.fabs(x) - 0.5 * delta)
    
    # Use CasADi's conditional operation
    return glue_functions(residual, delta, small_error_loss, large_error_loss)

"""
Weight optimization variables
"""
class WeightsObjective:

    def __init__(self, params, weight_list):

        self.params = params
        self.weight_list = weight_list

        self.weights = []
        self.weighted_variables = []
        self.cost_functions = []

    # Weights w are a parameter vector
    # Only add weights if they are not also parameters!
    def add(self, variable_to_weight, weight_names, cost_function=lambda x, w: w[0] * x ** 2):

        # Make sure it's a list if it isn't yet
        if type(weight_names) != list:
            weight_names = [weight_names]

        # Add all weights in the list
        for weight_name in weight_names:
            self.weight_list.append(weight_name)

        self.weighted_variables.append(variable_to_weight)
        self.weights.append(weight_names)
        self.cost_functions.append(cost_function)

    def get_value(self, z, model, settings, stage_idx):
        cost = 0

        for idx, cost_function in enumerate(self.cost_functions):
            weights = []
            for cost_weight in self.weights[idx]:
                weights.append(getattr(settings.params, cost_weight))

            variable = model.get_input_or_state(z, self.weighted_variables[idx])
            cost += cost_function(variable, weights)

        return cost


class ContouringObjective:

    def __init__(self, params, weight_list, n_segments):
        self.weight_list = weight_list

        self.n_segments = n_segments
        self.define_parameters(params)

    def define_parameters(self, params):
        self.weight_list.append('contour')
        self.weight_list.append('lag')

        for i in range(self.n_segments):
            params.add_multiple_parameters("spline" + str(i), 8)
            params.add_parameter("spline_start" + str(i))

    def get_value(self, z, model, settings, stage_idx):
        cost = 0
        pos_x = model.get_state(z, 'x', True)
        pos_y = model.get_state(z, 'y', True)

        s = model.get_state(z, 'spline', True)

        contour_weight = getattr(settings.params, 'contour')
        lag_weight = getattr(settings.params, 'lag')

        splines = []  # Classes containing the splines
        lambdas = []  # Merges splines
        for i in range(self.n_segments):
            splines.append(SplineParameters(settings.params, i))
            splines[-1].compute_path(s)

            # No lambda for the first segment (it is not glued to anything prior)
            if i > 0:
                lambdas.append(1. / (1. + np.exp((s - splines[-1].s_start + 0.02) / 0.1)))  # Sigmoid

        # We iteratively glue paths together here (start with the last path)
        path_x = splines[-1].path_x
        path_y = splines[-1].path_y
        path_dx = splines[-1].path_dx
        path_dy = splines[-1].path_dy
        for k in range(len(splines) - 1, 0, -1):
            # Glue with the previous path
            path_x = lambdas[k - 1] * splines[k-1].path_x + (1. - lambdas[k - 1]) * path_x
            path_y = lambdas[k - 1] * splines[k-1].path_y + (1. - lambdas[k - 1]) * path_y
            path_dx = lambdas[k - 1] * splines[k-1].path_dx + (1. - lambdas[k - 1]) * path_dx
            path_dy = lambdas[k - 1] * splines[k-1].path_dy + (1. - lambdas[k - 1]) * path_dy

        path_norm = np.sqrt(path_dx ** 2 + path_dy ** 2)
        path_dx_normalized = path_dx / path_norm
        path_dy_normalized = path_dy / path_norm

        contour_error = path_dy_normalized * (pos_x - path_x) - path_dx_normalized * (pos_y - path_y)
        lag_error = -path_dx_normalized * (pos_x - path_x) - path_dy_normalized * (pos_y - path_y)

        cost += contour_weight * contour_error ** 2
        cost += lag_weight * lag_error ** 2

        return cost


class PreviewObjective:

    def __init__(self, params, weight_list, n_segments, T):
        self.weight_list = weight_list

        self.T = T
        self.n_segments = n_segments

        self.define_parameters(params)

    def define_parameters(self, params):
        self.weight_list.append('preview')

    def get_value(self, z, model, settings, stage_idx):
        # Terminal state only
        if stage_idx < settings.N_bar - 2:
            return 0.

        cost = 0

        preview_weight = getattr(settings.params, 'preview')

        # Integrate the trajectory to obtain the preview point

        z_constant_input = casadi.vertcat(np.zeros((model.nu)), z[model.nu:model.nu+model.nx])

        z_preview = model.integrate(z_constant_input, self.T) # Integrate the dynamics T seconds ahead
        z_preview = casadi.vertcat(np.zeros((model.nu)), z_preview)

        pos_x = model.get_state(z_preview, 'x', True)
        pos_y = model.get_state(z_preview, 'y', True)
        s = model.get_state(z_preview, 'spline', True)

        spline = SplineParameters(settings.params, self.n_segments - 1) # Get the last spline
        spline.compute_path(s)

        path_norm = np.sqrt(spline.path_dx ** 2 + spline.path_dy ** 2)
        path_dx_normalized = spline.path_dx / path_norm
        path_dy_normalized = spline.path_dy / path_norm

        contour_error = path_dy_normalized * (pos_x - spline.path_x) - path_dx_normalized * (pos_y - spline.path_y)
        lag_error = -path_dx_normalized * (pos_x - spline.path_x) - path_dy_normalized * (pos_y - spline.path_y)

        cost += preview_weight * contour_error ** 2
        cost += preview_weight * lag_error ** 2
        # cost += settings.weights.lag * lag_error ** 2

        return cost


class GoalTrackingObjective:

    def __init__(self, params, weight_list):
        self.weight_list = weight_list

        self.define_parameters(params)

    def define_parameters(self, params):
        self.weight_list.append('goal')

        params.add_parameter("goal_x")
        params.add_parameter("goal_y")

    def get_value(self, z, model, settings, stage_idx):
        cost = 0
        pos_x = model.get_state(z, 'x', True)
        pos_y = model.get_state(z, 'y', True)

        goal_weight = getattr(settings.params, 'goal')

        goal_x = getattr(settings.params, 'goal_x')
        goal_y = getattr(settings.params, 'goal_y')

        # cost += goal_weight * ((pos_x - goal_x) ** 2 + (pos_y - goal_y)**2)

        dist_to_goal = (pos_x - goal_x) ** 2 + (pos_y - goal_y) ** 2
        normalized_dist = dist_to_goal / (20.0)
        cost += goal_weight * huber_loss(normalized_dist, 0, 0.1)

        return cost

class ReferenceVelocityObjective:
    """
    Track a reference velocity that may change with time
    """

    def __init__(self, params, weight_list):
        self.weight_list = weight_list
        self.define_parameters(params)

    def define_parameters(self, params):
        params.add_parameter('velocity_reference') # Velocity reference is a parameter now!

        self.weight_list.append('velocity')

    def get_value(self, z, model, settings, stage_idx):

        # v = u[0] # Unicycle @todo
        v = model.get_state(z, 'v', True)
        velocity_reference = getattr(settings.params, 'velocity_reference')
        velocity_weight = getattr(settings.params, 'velocity')

        return velocity_weight * ((v - velocity_reference) ** 2)  # track the given velocity reference


class PathReferenceVelocityObjective:
    """
    Track a reference velocity along the path component that may change with time
    """

    def __init__(self, params, weight_list, n_segments):
        self.weight_list = weight_list
        self.define_parameters(params)
        self.n_segments = n_segments

    def define_parameters(self, params):
        params.add_parameter('velocity_reference') # Velocity reference is a parameter now!

        self.weight_list.append('velocity')

    def get_value(self, z, model, settings, stage_idx):

        # if stage_idx < settings.N: # @todo: Temporary for Khaled comparison!
        #     return 0.


        psi = model.get_state(z, 'psi', True)

        v = model.get_state(z, 'v', True)
        s = model.get_state(z, 'spline', True)

        velocity_reference = getattr(settings.params, 'velocity_reference')
        velocity_weight = getattr(settings.params, 'velocity')

        # return velocity_weight * (v - velocity_reference)**2 #  @todo: Temporary for Khaled comparison!

        # @todo: Put this somewhere general (now copied from contouring weights)
        splines = []  # Classes containing the splines
        lambdas = []  # Merges splines
        for i in range(self.n_segments):
            splines.append(SplineParameters(settings.params, i))
            splines[-1].compute_path(s)

            # No lambda for the first segment (it is not glued to anything prior)
            if i > 0:
                lambdas.append(1. / (1. + np.exp((s - splines[-1].s_start + 0.02) / 0.1)))  # Sigmoid

        # We iteratively glue paths together here (start with the last path)
        path_x = splines[-1].path_x
        path_y = splines[-1].path_y
        path_dx = splines[-1].path_dx
        path_dy = splines[-1].path_dy
        for k in range(len(splines) - 1, 0, -1):
            # Glue with the previous path
            # path_x = lambdas[k - 1] * splines[k - 1].path_x + (1. - lambdas[k - 1]) * path_x
            # path_y = lambdas[k - 1] * splines[k - 1].path_y + (1. - lambdas[k - 1]) * path_y
            path_dx = lambdas[k - 1] * splines[k - 1].path_dx + (1. - lambdas[k - 1]) * path_dx
            path_dy = lambdas[k - 1] * splines[k - 1].path_dy + (1. - lambdas[k - 1]) * path_dy

        path_norm = np.sqrt(path_dx ** 2 + path_dy ** 2)
        path_dx_normalized = path_dx / path_norm
        path_dy_normalized = path_dy / path_norm

        path_angle = casadi.atan2(path_dy_normalized, path_dx_normalized)
        diff_angle = path_angle - psi
        v_path = casadi.cos(diff_angle) * v

        return velocity_weight * ((v_path - velocity_reference) ** 2)  # track the given velocity reference
        # return velocity_weight * ((v - velocity_reference) ** 2)  # track the given velocity reference


def objective(z, param, model, settings, stage_idx):
    # initialise cost at 0
    cost = 0.0
    settings.params.load_params(param)

    # Objective modules
    for module in settings.modules.modules:
        if module.type == "objective":
            for module_objective in module.objectives:
                cost += module_objective.get_value(z, model, settings, stage_idx)

    return cost

# Deprecated!
def roboat_objective(z, param, model, roboat_settings):

    roboat_settings.weights.set_weights(param)

    # Retrieve variables
    x = z[model.nu:model.nu + model.nx]
    u = z[0:model.nu]

    # States
    pos_x = x[0]
    pos_y = x[1]
    pos = np.array([pos_x, pos_y])
    psi = x[2]
    u_velocity = x[3]
    v_velocity = x[4]
    r_yawrate = x[5]
    spline = x[6]

    # Inputs
    f1 = u[0]
    f2 = u[1]
    f3 = u[2]
    f4 = u[3]
    slack = u[4]

    # Parameters for the spline
    s01 = param[24]
    s02 = param[25]
    s03 = param[26]
    d = param[27]

    spline_1 = SplineParameters(param, s01, 0)
    spline_2 = SplineParameters(param, s02, 1)
    spline_3 = SplineParameters(param, s03, 2)

    # Reference velocity
    u_reference = roboat_settings.weights.velocity_reference

    # Derive contouring and lagging errors
    param_lambda = 1 / (1 + np.exp((spline - s02 + 0.02) / 0.1))
    param_lambda2 = 1 / (1 + np.exp((spline - s03 + 0.02) / 0.1))

    spline_1.compute_path(spline)
    spline_2.compute_path(spline)
    spline_3.compute_path(spline)

    path_x = param_lambda * spline_1.path_x + param_lambda2 * (1 - param_lambda) * spline_2.path_x + (
            1 - param_lambda2) * spline_3.path_x
    path_y = param_lambda * spline_1.path_y + param_lambda2 * (1 - param_lambda) * spline_2.path_y + (
            1 - param_lambda2) * spline_3.path_y
    path_dx = param_lambda * spline_1.path_dx + param_lambda2 * (1 - param_lambda) * spline_2.path_dx + (
            1 - param_lambda2) * spline_3.path_dx
    path_dy = param_lambda * spline_1.path_dy + param_lambda2 * (1 - param_lambda) * spline_2.path_dy + (
            1 - param_lambda2) * spline_3.path_dy

    path_norm = np.sqrt(path_dx ** 2 + path_dy ** 2)
    path_dx_normalized = path_dx / path_norm
    path_dy_normalized = path_dy / path_norm

    contour_error = path_dy_normalized * (pos_x - path_x) - path_dx_normalized * (pos_y - path_y)
    lag_error = -path_dx_normalized * (pos_x - path_x) - path_dy_normalized * (pos_y - path_y)

    # Repulsive cost
    repulsive_weight = roboat_settings.weights.Wrepulsive

    rotation_car = rotation_matrix(psi)
    cost_repulsive = 0
    max_obstacles = 6
    start_param = 46
    r_disc = param[start_param]

    Winput_forward = roboat_settings.weights.Winput_forward
    Winput_sideways = roboat_settings.weights.Winput_sideways
    regulations_repulsive_weight = roboat_settings.weights.W_ellipseregulation
    a_factor = roboat_settings.weights.a_ellipse_factor
    b_factor = roboat_settings.weights.b_ellipse_factor
    c = roboat_settings.weights.c_ellipse_factor
    d = roboat_settings.weights.d_ellipse_factor
    margin = roboat_settings.weights.Margin_normal_ellipsoid
    W_rightofway = roboat_settings.weights.W_rightofway
    e_ellipse = roboat_settings.weights.e_ellipse_factor
    f_ellipse = roboat_settings.weights.f_ellipse_factor

    if roboat_settings.enable_repulsive: # TODO: check parameters!
        if roboat_settings.enable_ellipsoid_constraints:
            for obstacle_it in range(0, max_obstacles):
                gamma = 0.001
                obst_x = param[start_param + 4 + 0 + obstacle_it * 7]
                obst_y = param[start_param + 4 + 1 + obstacle_it * 7]
                obst_psi = param[start_param + 4 + 2 + obstacle_it * 7]
                obst_major = param[start_param + 4 + 3 + obstacle_it * 7]
                obst_minor = param[start_param + 4 + 4 + obstacle_it * 7]

                # Compute ellipse matrix
                ab = np.array([[1 / ((obst_major + r_disc + margin) ** 2), 0],
                               [0, 1 / ((obst_minor + r_disc + margin) ** 2)]])

                obstacle_rotation = rotation_matrix(obst_psi)
                obstacle_ellipse_matrix = obstacle_rotation.transpose().dot(ab).dot(obstacle_rotation)

                dx = pos_x - obst_x
                dy = pos_y - obst_y
                dist_to_obstacle = np.array([dx, dy])
                ellipsoidal_distance = dist_to_obstacle.transpose().dot(obstacle_ellipse_matrix).dot(dist_to_obstacle)

                cost_repulsive += repulsive_weight * (1 / (gamma + ellipsoidal_distance))

        if roboat_settings.enable_waterregulations_costs:
            for obstacle_it in range(0, max_obstacles):
                obst_x = param[start_param + 4 + 0 + obstacle_it * 7]
                obst_y = param[start_param + 4 + 1 + obstacle_it * 7]
                obst_psi = param[start_param + 4 + 2 + obstacle_it * 7]
                obst_major = param[start_param + 4 + 3 + obstacle_it * 7]
                obst_minor = param[start_param + 4 + 4 + obstacle_it * 7]

                sigma_x = a_factor*(obst_major + r_disc)
                sigma_y = b_factor*(obst_minor + r_disc)

                obstacle_rotation = rotation_matrix(obst_psi)

                o_ellipse_body = np.array([c, -d])
                o_ellipse_world = obstacle_rotation.dot(o_ellipse_body) + np.array([obst_x, obst_y])
                x0 = o_ellipse_world[0]
                y0 = o_ellipse_world[1]

                k = np.cos(obst_psi)**2/(2*sigma_x**2) + np.sin(obst_psi)**2/(2*sigma_y**2)
                l = np.sin(2*obst_psi)/(4*sigma_x**2) - np.sin(2*obst_psi)/(4*sigma_y**2)
                m = np.sin(obst_psi)**2/(2*sigma_x**2) + np.cos(obst_psi)**2/(2*sigma_y**2)

                cost_repulsive += regulations_repulsive_weight * np.exp(- (k*(pos_x-x0)**2 + 2*l*(pos_x-x0)*(pos_y-y0)
                                                                           + m*(pos_y-y0)**2))

        if roboat_settings.enable_rightofway_costs:
            for obstacle_it in range(0, max_obstacles):
                priority = param[164 + obstacle_it] # 164 dependent on whether scenario_constraints, ellipsoid_constraints
                # and linear constraints are enabled in roboat_settings.py

                obst_x = param[start_param + 4 + 0 + obstacle_it * 7]
                obst_y = param[start_param + 4 + 1 + obstacle_it * 7]
                obst_psi = param[start_param + 4 + 2 + obstacle_it * 7]
                obst_major = param[start_param + 4 + 3 + obstacle_it * 7]
                obst_minor = param[start_param + 4 + 4 + obstacle_it * 7]

                sigma_x = e_ellipse
                sigma_y = obst_minor + r_disc

                obstacle_rotation = rotation_matrix(obst_psi)

                o_ellipse_body = np.array([f_ellipse, 0])
                o_ellipse_world = obstacle_rotation.dot(o_ellipse_body) + np.array([obst_x, obst_y])
                x0 = o_ellipse_world[0]
                y0 = o_ellipse_world[1]

                a = np.cos(obst_psi)**2/(2*sigma_x**2) + np.sin(obst_psi)**2/(2*sigma_y**2)
                b = np.sin(2*obst_psi)/(4*sigma_x**2) - np.sin(2*obst_psi)/(4*sigma_y**2)
                c = np.sin(obst_psi)**2/(2*sigma_x**2) + np.cos(obst_psi)**2/(2*sigma_y**2)

                cost_repulsive += priority * W_rightofway * np.exp(- (a*(pos_x-x0)**2 + 2*b*(pos_x-x0)*(pos_y-y0)
                                                                           + c*(pos_y-y0)**2))

    # Weights
    Wslack = roboat_settings.weights.Wslack
    Wcontour = roboat_settings.weights.Wcontour
    Wlag = roboat_settings.weights.Wlag
    Wu = roboat_settings.weights.Kv # velocity weight u

    # Cost function
    cost_contour = Wcontour * contour_error ** 2
    cost_lag = Wlag * lag_error ** 2
    cost_uvelocity_ref = Wu * ((u_velocity - u_reference) ** 2)
    cost_slack = Wslack*slack*slack
    cost_input = Winput_forward*(f1*f1 + f2*f2) + Winput_sideways*(f3*f3 + f4*f4)

    cost = cost_uvelocity_ref + cost_input + cost_contour + cost_lag + cost_slack + cost_repulsive

    return cost


# def hovergames_objective(z, param, model, settings, stage_idx):
#     # Initialise cost at 0
#     cost = 0.0
#
#     # Retrieve variables
#     x = z[model.nu:model.nu + model.nx]
#     u = z[0:model.nu]
#
#     vx = x[3]
#     vy = x[4]
#     vz = x[5]
#
#     # Weights
#     settings.weights.set_weights(param)
#
#     # Add other cost terms
#     for module in settings.modules.modules:
#         if module.type == "objective":
#             for module_objective in module.objectives:
#                 cost += module_objective.get_value(x, u, settings, stage_idx)
#
#     v_ref = settings.weights.velocity_reference
#     cost += settings.weights.velocity * ((vx - v_ref) ** 2 + (vy - v_ref) ** 2 + (vz - v_ref) ** 2) / (
#                 model.system.upper_bound['vx'] - model.system.lower_bound['vx'])
#
#     return cost


# Deprecated!
def hovergames_objective(z, param, model, settings, stage_idx):
    # Update parameters
    # -------------------------------------------------------------------------------
    settings.weights.set_weights(param)
    # -------------------------------------------------------------------------------

    # Obtain inputs and states
    # -------------------------------------------------------------------------------
    # if not is_terminal:
    u = z[0:model.nu]
    phi_c = u[0]
    theta_c = u[1]
    dpsi_c = u[2]
    thrust_c = u[3]

    x = z[model.nu:model.nu + model.nx]
    # else:
    #     state = z[0:model.nx]

    pos_x = x[0]
    pos_y = x[1]
    pos_z = x[2]
    vx = x[3]
    vy = x[4]
    vz = x[5]
    phi = x[6]
    theta = x[7]
    psi = x[8]
    # -------------------------------------------------------------------------------

    # Calculate errors
    # -------------------------------------------------------------------------------
    # Contour error
    # TODO: calculate contour error, possibly by making use of above (Prius/roboat) code and:
    # https://github.com/GoldenZephyr/videography-mpc/blob/master/src/videography_drone/mpc/mpc_contour_costs.py
    contour_err = 0.0
    # Temp functionality for custom goal location [1, 1, 2]
    contour_err += (1 - pos_x) ** 2 + (1 - pos_y) ** 2 + (2 - pos_z) ** 2

    # Lag error
    # TODO: calculate lag error, possibly by making use of above (Prius/roboat) code and:
    # https://github.com/GoldenZephyr/videography-mpc/blob/master/src/videography_drone/mpc/mpc_contour_costs.py
    lag_err = 0.0
    # Temp functionality
    lag_err += (0 - phi) ** 2 + (0 - theta) ** 2 + (0 - psi) ** 2

    # Velocity error
    # v = vx ** 2 + vy ** 2 + vz ** 2
    v_ref = settings.weights.velocity_reference
    velocity_err = (vx - v_ref) ** 2 + (vy - v_ref) ** 2 + (vz - v_ref) ** 2
    # velocity_err = 0.0
    # -------------------------------------------------------------------------------

    # Cost function
    # -------------------------------------------------------------------------------
    # Initialize cost
    cost = 0.0

    # Add cost terms
    # if not is_terminal:
    # cost += settings.weights.w_contour * contour_err
    # cost += settings.weights.w_lag * lag_err
    # cost += settings.weights.w_velocity * velocity_err
    # cost += settings.weights.w_input_angles * phi_c ** 2 + \
    #         settings.weights.w_input_angles * theta_c ** 2 + \
    #         settings.weights.w_input_dpsi * dpsi_c ** 2 + \
    #         settings.weights.w_input_thrust * (thrust_c - 9.81) ** 2

    # Stabilizing cost function
    # ------------------------------- #
    cost += 0.5 * (1 - x[0]) ** 2 + \
            0.5 * (1 - x[1]) ** 2 + \
            0.5 * (2 - x[2]) ** 2 + \
            1 * (0 - x[3]) ** 2 + \
            1 * (0 - x[4]) ** 2 + \
            1 * (0 - x[5]) ** 2 + \
            1 * (0 - x[6]) ** 2 + \
            1 * (0 - x[7]) ** 2 + \
            1 * (0 - x[8]) ** 2
    cost += 1 * (0 - u[0]) ** 2 + \
            1 * (0 - u[1]) ** 2 + \
            1 * (0 - u[2]) ** 2 + \
            2 * (9.81 - u[3]) ** 2
    # ------------------------------- #

    # if is_terminal:
        # cost += settings.weights.w_contour * contour_err ** 2
        # cost += settings.weights.w_lag * lag_err ** 2
    # -------------------------------------------------------------------------------

    return cost
