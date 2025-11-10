import casadi
import forcespro.nlp
import numpy as np


# Returns discretized dynamics of a given model (see below)
def discrete_dynamics(z, model, integrator_stepsize):
    """

    @param z: state vector (u, x)
    @param model: Model of the system
    @param integrator_stepsize: Integrator stepsize in seconds
    @return:
    """
    # We use an explicit RK4 integrator here to discretize continuous dynamics

    result = forcespro.nlp.integrate(
        model.continuous_model,
        z[model.nu:model.nu+model.nx],
        z[0:model.nu],
        integrator=forcespro.nlp.integrators.RK4,
        stepsize=integrator_stepsize)

    # Steering delay:
    if hasattr(model, 'steering_delay'):  # Todo: code this properly

        if model.steering_delay == 1:
            result[model.get_state_idx('delta')] = model.get_state(z, 'delta_in', True)  # delta_{k+1} = delta^in_k
        elif model.steering_delay == 2:
            result[model.get_state_idx('delta')] = model.get_state(z, 'delta_in2', True)  # delta = delta_in2
            result[model.get_state_idx('delta_in2')] = model.get_state(z, 'delta_in', True)  # delta_in2 = delta_in

    return result

# Dynamics, i.e. equality constraints #
# This class contains models to choose from
# They can be coupled with physical limits using Systems defined in systems.py
# See Bicycle model for an example of the required parameters
class DynamicModel:

    def __init__(self, system):
        self.nvar = self.nu + self.nx
        self.system = system
        self.control_inputs = dict()
        self.possible_inputs_to_vehicle = []

    def __str__(self):
        result = 'Dynamical Model: ' + str(type(self)) + '\n' +\
               'System: ' + str(self.system) + '\n'

        if hasattr(self, 'interfaces'):
            result += 'Interfaces: '

            for interface in self.interfaces:
                result += interface + " "

            result += "\n"

        result += 'States: ' + str(self.states) + '\n'
        result += 'Inputs: ' + str(self.inputs) + '\n'
        return result

    # Appends upper bounds from system
    def upper_bound(self):
        result = np.array([])

        for input in self.inputs:
            result = np.append(result, self.system.upper_bound[input])

        for state in self.states:
            result = np.append(result, self.system.upper_bound[state])

        return result

    # Appends lower bounds from system
    def lower_bound(self):
        result = np.array([])

        for input in self.inputs:
            result = np.append(result, self.system.lower_bound[input])

        for state in self.states:
            result = np.append(result, self.system.lower_bound[state])

        return result

    def integrate(self, z, duration):
        return discrete_dynamics(z, self, duration)

    def get_state(self, z, state_name, required=False):
        if state_name not in self.states:
            # print("Unknown state {} was indexed! Setting to zero".format(state_name))
            assert not required, "Required state {} was not part of the model".format(state_name)
            return 0.

        return z[self.nu + self.states.index(state_name)]

    def get_input(self, z, input_name, required=False):
        if input_name not in self.inputs:
            # print("Unknown state {} was indexed! Setting to zero".format(input_name))
            assert not required, "Required input {} was not part of the model".format(input_name)

            return 0.

        return z[self.inputs.index(input_name)]

    def get_input_or_state(self, z, name, required=False):
        if (name not in self.inputs) and (name not in self.states):
            # print("Unknown variable {} was indexed! Setting to zero".format(name))
            assert not required, "Required variable {} was not part of the model".format(name)

            return 0.

        if name in self.inputs:
            return z[self.inputs.index(name)]
        else:
            return z[self.nu + self.states.index(name)]

    def get_state_idx(self, name):
        if name not in self.states:
            return -1
        else:
            return self.states.index(name)



# Unicycle model
class UnicycleModel(DynamicModel):

    def __init__(self, system):
        self.nu = 2  # number of control variables
        self.nx = 4  # number of states

        super(UnicycleModel, self).__init__(system)

        self.states = ['x', 'y', 'psi', 'spline']                           # Name of the states
        self.states_from_sensor = [True, True, True, False]                 # Are the states read from the sensors?
        self.states_from_sensor_at_infeasible = [True, True, True, False]   # If true, are reset to sensor values when infeasible

        self.inputs = ['v', 'w']                                   # Name of the inputs
        self.control_inputs['velocity'] = 'v'
        self.control_inputs['rot_velocity'] = 'w'

    def continuous_model(self, x, u):

        v = u[0]
        w = u[1]
        psi = x[2]

        return np.array([v * casadi.cos(psi),
                         v * casadi.sin(psi),
                         w,
                         v])


class SecondOrderUnicycleModel(DynamicModel):

    def __init__(self, system):
        self.nu = 2  # number of control variables
        self.nx = 6  # number of states

        super(SecondOrderUnicycleModel, self).__init__(system)

        self.states = ['x', 'y', 'psi', 'v', 'w', 'spline']                           # Name of the states
        self.states_from_sensor = [True, True, True, True, True, False]                 # Are the states read from the sensors?
        self.states_from_sensor_at_infeasible = [True, True, True, True, True, False]   # If true, are reset to sensor values when infeasible

        self.inputs = ['a', 'alpha']                                   # Name of the inputs
        self.control_inputs['velocity'] = 'v'
        self.control_inputs['acceleration'] = 'a'

        self.control_inputs['rot_velocity'] = 'w'
        self.control_inputs['rot_acceleration'] = 'alpha'

    def continuous_model(self, x, u):

        a = u[0]
        alpha = u[1]
        psi = x[2]
        v = x[3]
        w = x[4]

        return np.array([v * casadi.cos(psi),
                         v * casadi.sin(psi),
                         w,
                         a,
                         alpha,
                         v])


class SecondOrderAccelerationUnicycleModel(DynamicModel):

    def __init__(self, system):
        self.nu = 2  # number of control variables
        self.nx = 5  # number of states

        super(SecondOrderAccelerationUnicycleModel, self).__init__(system)

        self.states = ['x', 'y', 'psi', 'v', 'spline']                           # Name of the states
        self.states_from_sensor = [True, True, True, True, False]                 # Are the states read from the sensors?
        self.states_from_sensor_at_infeasible = [True, True, True, True, False]   # If true, are reset to sensor values when infeasible

        self.inputs = ['a', 'w']                                   # Name of the inputs
        self.control_inputs['velocity'] = 'v'
        self.control_inputs['acceleration'] = 'a'

        self.control_inputs['rot_velocity'] = 'w'

    def continuous_model(self, x, u):

        a = u[0]
        w = u[1]
        psi = x[2]
        v = x[3]

        return np.array([v * casadi.cos(psi),
                         v * casadi.sin(psi),
                         w,
                         a,
                         v])

# First-order Bicycle model
class SimpleBicycleModel(DynamicModel):

    def __init__(self, system):
        self.nu = 3  # number of control variables
        self.nx = 4  # number of states

        super(SimpleBicycleModel, self).__init__(system)

        self.states = ['x', 'y', 'psi', 'spline']                           # Name of the states
        self.states_from_sensor = [True, True, True, False]                 # Are the states read from the sensors?
        self.states_from_sensor_at_infeasible = [True, True, True, False]   # If true, are reset to sensor values when infeasible

        self.inputs = ['v', 'w', 'slack']                                   # Name of the inputs
        self.control_inputs['velocity'] = 'v'
        self.control_inputs['rot_velocity'] = 'w'

    # Return x_dot(x, u)!
    def continuous_model(self, x, u):
        v = u[0]
        w = u[1]
        psi = x[2]

        return np.array([v * casadi.cos(psi),
                         v * casadi.sin(psi),
                         w,
                         v])

# Vic: Steering kinematic first order
class SimpleBicycleModelSteering(DynamicModel):

    def __init__(self, system):
        self.nu = 2  # number of control variables
        self.nx = 4  # number of states

        super(SimpleBicycleModelSteering, self).__init__(system)

        self.states = ['x', 'y', 'psi', 'spline']                                 # Name of the states
        self.states_from_sensor = [True, True, True, False]                    # Are the states read from the sensors?
        self.states_from_sensor_at_infeasible = [True, True, True, False]      # If true, are reset to sensor values when infeasible

        self.inputs = ['v', 'delta']                                     # Name of the inputs
        self.control_inputs['velocity'] = 'v'
        self.control_inputs['steering'] = 'delta'

    def continuous_model(self, z, u):   
        # States
        x, y, psi = z[0], z[1], z[2]
        
        # Inputs
        vx, delta = u[0], u[1]

        # Sideslip angle
        beta = casadi.atan(self.system.lr/(self.system.lf+self.system.lr) * casadi.tan(delta))
        self.beta = beta

        # State derivatives
        x_dot = vx * casadi.cos(psi + beta)
        y_dot = vx * casadi.sin(psi + beta)
        psi_dot = (vx / self.system.lr) * casadi.sin(beta)

        return np.array([x_dot, y_dot, psi_dot, vx])


# Second-order Bicycle model
class BicycleModel(DynamicModel):

    def __init__(self, system):
        self.nu = 3 # number of control variables
        self.nx = 5 # number of states

        super(BicycleModel, self).__init__(system)

        self.states = ['x', 'y', 'psi', 'v', 'w']  # , 'ax', 'ay'
        self.states_from_sensor = [True, True, True, True, True]  # , True, True
        self.states_from_sensor_at_infeasible = [True, True, True, True, True]  # False variables are guessed 0 at infeasible

        self.inputs = ['a', 'alpha', 'slack']
        self.control_inputs['velocity'] = 'v'
        self.control_inputs['acceleration'] = 'a'

        self.control_inputs['rot_velocity'] = 'w'
        self.control_inputs['rot_acceleration'] = 'alpha'

    def continuous_model(self, x, u):
        a = u[0]
        alpha = u[1]
        psi = x[2]
        v = x[3]
        w = x[4]

        return np.array([v * casadi.cos(psi),
                         v * casadi.sin(psi),
                         w,
                         a,
                         alpha])


# Bicycle model with dynamic steering
class BicycleModel2ndOrder(DynamicModel):

    def __init__(self, system):
        self.nu = 2
        self.nx = 6
        super(BicycleModel2ndOrder, self).__init__(system)

        self.states = ['x', 'y', 'psi', 'v', 'delta', 'spline']  # , 'ax', 'ay'
        self.states_from_sensor = [True, True, True, True, True, False]  # , True, True
        self.states_from_sensor_at_infeasible = [True, True, True, True, True, False]

        self.inputs = ['a', 'w']
        self.control_inputs['steering'] = 'delta'
        self.control_inputs['rot_velocity'] = 'w'
        self.control_inputs['velocity'] = 'v'
        self.control_inputs['acceleration'] = 'a'

    def continuous_model(self, x, u):
        a = u[0]
        w = u[1]
        psi = x[2]
        v = x[3]
        delta = x[4]

        #lr = 1.38
        #lf = 1.61
        # L = lr + lf
        # ratio = lr/(lr + lf)
        ratio = self.system.ratio
        lr = self.system.lr

        beta = casadi.arctan(ratio * casadi.tan(delta))

        return np.array([v * casadi.cos(psi + beta),
                         v * casadi.sin(psi + beta),
                         (v/lr) * casadi.sin(beta),
                         a,
                         w,
                         v])

# Bicycle model with dynamic steering
class BicycleModel2ndOrderWithDelay(DynamicModel):

    def __init__(self, system):
        self.nu = 2
        self.nx = 7
        super(BicycleModel2ndOrderWithDelay, self).__init__(system)

        self.states = ['x', 'y', 'psi', 'v', 'delta', 'delta_in', 'spline']  # , 'ax', 'ay'
        self.states_from_sensor = [True, True, True, True, False, True, False]  # , True, True
        self.states_from_sensor_at_infeasible = [True, True, True, True, False, True, False]

        self.inputs = ['a', 'w']
        self.control_inputs['steering'] = 'delta_in'
        self.control_inputs['velocity'] = 'v'
        self.control_inputs['acceleration'] = 'a'
        self.control_inputs['rot_velocity'] = 'w'

    def continuous_model(self, x, u):
        a = u[0]
        w = u[1]
        psi = x[2]
        v = x[3]
        delta = x[4] # affects the model
        delta_in = x[5] # = w (i.e., is updated with the input)

        ratio = self.system.ratio
        lr = self.system.lr

        beta = casadi.arctan(ratio * casadi.tan(delta))

        return np.array([v * casadi.cos(psi + beta),
                         v * casadi.sin(psi + beta),
                         (v/lr) * casadi.sin(beta),
                         a,
                         0., # set in the discrete dynamics
                         w,
                         v])

# Bicycle model with dynamic steering
class BicycleModel2ndOrderWith2Delay(DynamicModel):

    def __init__(self, system):
        self.nu = 2
        self.nx = 8
        super(BicycleModel2ndOrderWith2Delay, self).__init__(system)

        self.states = ['x', 'y', 'psi', 'v', 'delta', 'delta_in2', 'delta_in', 'spline']
        self.states_from_sensor = [True, True, True, True, False, False, True, False]
        self.states_from_sensor_at_infeasible = [True, True, True, True, False, False, True, False]

        self.inputs = ['a', 'w']
        self.control_inputs['steering'] = 'delta_in'
        self.control_inputs['velocity'] = 'v'
        self.control_inputs['acceleration'] = 'a'
        self.control_inputs['rot_velocity'] = 'w'

    def continuous_model(self, x, u):
        a = u[0]
        w = u[1]
        psi = x[2]
        v = x[3]
        delta = x[4] # affects the model ( = delta_in2)
        delta_in2 = x[5] # = delta_in
        delta_in = x[6] # = w (i.e., is updated with the input)

        ratio = self.system.ratio
        lr = self.system.lr

        beta = casadi.arctan(ratio * casadi.tan(delta))

        return np.array([v * casadi.cos(psi + beta),
                         v * casadi.sin(psi + beta),
                         (v/lr) * casadi.sin(beta),
                         a,
                         0., # set in the discrete dynamics
                         0., # set in the discrete dynamics
                         w,
                         v])

# Bicycle model with dynamic steering and jerk
class BicycleModel2ndOrderWithJerk(DynamicModel):

    def __init__(self, system):
        self.nu = 2
        self.nx = 7
        super(BicycleModel2ndOrderWithJerk, self).__init__(system)

        self.states = ['x', 'y', 'psi', 'v', 'a', 'delta', 'spline']  # , 'ax', 'ay'
        self.states_from_sensor = [True, True, True, True, True, True, False]  # , True, True
        self.states_from_sensor_at_infeasible = [True, True, True, True, True, True, False]

        self.inputs = ['j', 'w']    # Jerk instead of accel
        self.control_inputs['velocity'] = 'v'
        self.control_inputs['acceleration'] = 'a'
        self.control_inputs['jerk'] = 'j'

        self.control_inputs['steering'] = 'delta'
        self.control_inputs['rot_velocity'] = 'w'

    def continuous_model(self, x, u):
        j = u[0]
        w = u[1]
        psi = x[2]
        v = x[3]
        a = x[4]
        delta = x[5]
        # lr = 1.38
        # lf = 1.61
        # L = lr + lf
        # ratio = lr/(lr + lf)
        ratio = self.system.ratio
        lr = self.system.lr

        beta = casadi.arctan(ratio * casadi.tan(delta))

        return np.array([v * casadi.cos(psi + beta),
                         v * casadi.sin(psi + beta),
                         (v/lr) * casadi.sin(beta), #v/lr*casadi.sin(beta),
                         a,
                         j,
                         w,
                         v])

# Model with lateral acceleration
class BicycleWithLateralAcceleration(DynamicModel):

    def __init__(self, system):
        self.nu = 2 #3
        self.nx = 8
        super(BicycleWithLateralAcceleration, self).__init__(system)
        self.states = ['x', 'y', 'psi', 'v', 'delta', 'spline', 'a', 'ay']
        self.states_from_sensor = [True, True, True, True, True, False, True, True]  # , True, True
        self.states_from_sensor_at_infeasible = [True, True, True, True, False, True, True, True]

        self.inputs = ['j', 'w']
        self.control_inputs['velocity'] = 'v'
        self.control_inputs['acceleration'] = 'a'
        self.control_inputs['jerk'] = 'j'

        self.control_inputs['steering'] = 'delta'
        self.control_inputs['rot_velocity'] = 'w'

    def continuous_model(self, x, u):
        # Direct copy of old prius model
        jerk = u[0]
        w = u[1]
        psi = x[2]
        v = x[3]
        delta = x[4]
        # x[5] Spline
        a_x = x[6]
        a_y = x[7]

        lr = 1.577
        lf = 1.123
        L = lr + lf

        return np.array([v * casadi.cos(psi),
                         v * casadi.sin(psi),
                         v * casadi.tan(delta) / L,
                         a_x,
                         w,
                         v,
                         jerk,
                         (2 * a_x * delta + v * w) * (v / L)])

# Model with lateral acceleration
class BicycleWithLateralAccelerationAndDelay(DynamicModel):

    def __init__(self, system):
        self.nu = 2 #3
        self.nx = 9
        super(BicycleWithLateralAccelerationAndDelay, self).__init__(system)
        self.states = ['x', 'y', 'psi', 'v', 'delta', 'delta_in', 'spline', 'a', 'ay']
        self.states_from_sensor = [True, True, True, True, False, True, False, False, False]  # , True, True
        self.states_from_sensor_at_infeasible = [True, True, True, True, False, True, True, False, False]

        self.inputs = ['j', 'w']
        self.control_inputs['velocity'] = 'v'
        self.control_inputs['acceleration'] = 'a'
        self.control_inputs['jerk'] = 'j'

        self.control_inputs['steering'] = 'delta_in'
        self.control_inputs['rot_velocity'] = 'w'

    def continuous_model(self, x, u):
        # Direct copy of old prius model
        jerk = u[0]
        w = u[1]
        psi = x[2]
        v = x[3]
        delta = x[4]
        delta_in = x[5]
        # x[6] Spline
        a = x[7]
        a_y = x[8]

        # lr = 1.577
        # lf = 1.123
        # L = lr + lf
        L = self.system.L

        return np.array([v * casadi.cos(psi),
                         v * casadi.sin(psi),
                         v * casadi.tan(delta) / L,
                         a,
                         0.,
                         w,
                         v,
                         jerk,
                         (2 * a * delta + v * w) * (v / L)])

# class BicycleModel3ndOrder(DynamicModel):

#     def __init__(self, system):
#         self.nu = 2
#         self.nx = 8
#         super(BicycleModel2ndOrder, self).__init__(system)

#         self.states = ['x', 'y', 'psi', 'v', 'delta', 'spline']  # , 'ax', 'ay'
#         self.states_from_sensor = [True, True, True, True, True, False]  # , True, True
#         self.states_from_sensor_at_infeasible = [True, True, True, True, True, False]

#         self.inputs = ['jx', 'dw']
#         self.control_inputs['steering'] = 'delta'
#         self.control_inputs['rot_velocity'] = 'w'
#         self.control_inputs['velocity'] = 'v'
#         self.control_inputs['acceleration'] = 'a'

#     def continuous_model(self, x, u):
#         a = u[0]
#         w = u[1]
#         psi = x[2]
#         v = x[3]
#         delta = x[4]

#         #lr = 1.38
#         #lf = 1.61
#         # L = lr + lf
#         # ratio = lr/(lr + lf)
#         ratio = self.system.ratio
#         lr = self.system.lr

#         beta = casadi.arctan(ratio * casadi.tan(delta))

#         return np.array([v * casadi.cos(psi + beta),
#                          v * casadi.sin(psi + beta),
#                          (v/lr) * casadi.sin(beta),
#                          a,
#                          w,
#                          v])

class RoboatModel(DynamicModel):
    def __init__(self, system):
        self.nu = 5
        self.nx = 7
        super(RoboatModel, self).__init__(system)

        # State: x position, y position, heading, surge velocity, sway velocity, yaw rate
        self.states = ['x', 'y', 'psi', 'u', 'v', 'r', 'spline']
        self.states_from_sensor = [True, True, True, True, True, True, False]  # , True, True
        self.states_from_sensor_at_infeasible = [True, True, True, True, True, True, False]

        # Input: forces sent to the four thrusters
        self.inputs = ['f1', 'f2', 'f3', 'f4', 'slack']
        # self.inputs_to_vehicle = [True, True, True, True, False]

    def continuous_model(self, x, u):
        # States
        psi = x[2]
        u_velocity = x[3]
        v_velocity = x[4]
        r_yawrate = x[5]

        # Inputs
        f1 = u[0]
        f2 = u[1]
        f3 = u[2]
        f4 = u[3]

        # Parameters Quarter scale
        # Diagonal mass matrix
        m11 = 12
        m22 = 16
        m33 = 1.5
        # Diagonal drag matrix
        d11 = 6
        d22 = 8
        d33 = 0.6
        # Dimensions boat
        aa = 0.45
        bb = 0.9

        # # Parameters Full scale
        # # Diagonal mass matrix
        # m11 = 1200
        # m22 = 2400
        # m33 = 900
        # # Diagonal drag matrix
        # d11 = 36
        # d22 = 48
        # d33 = 20
        # # Dimensions boat
        # aa = 1.4
        # bb = 2.05

        #  Calculate position and orientation derivative in the ENU coordinate frame
        dy = u_velocity * casadi.sin(psi) - v_velocity * casadi.cos(psi)
        dx = u_velocity * casadi.cos(psi) + v_velocity * casadi.sin(psi)
        dpsi = r_yawrate

        # Calculate velocity vector derivative
        du = 1 / m11 * ((f1 + f2) - d11 * u_velocity) # + m22 * v_velocity * r_yawrate) # u is the velocity forward, the y axis in body frame with ENU coordinates
        dv = 1 / m22 * ((f3 + f4) - d22 * v_velocity) # - m11 * u_velocity * r_yawrate) # v is the velocity to the right, the x axis in body frame with ENU coordinates
        dr = 1 / m33 * (-(f1 - f2) * aa / 2 + -(f3 - f4) * bb / 2 - d33 * r_yawrate) # - m11 * u_velocity * v_velocity + m22 * v_velocity * u_velocity)
        dspline = u_velocity

        return np.array([dx, dy, dpsi, du, dv, dr, dspline])

# A simplified version of the Roboat.
# Only the f1 and f2 thrusters are working. There is no damping or Coriolis.
class RoboatModelSimple(DynamicModel):
    def __init__(self, system):
        self.nu = 5
        self.nx = 7
        super(RoboatModelSimple, self).__init__(system)

        # State: x position, y position, heading, surge velocity, sway velocity, yaw rate
        self.states = ['x', 'y', 'psi', 'u', 'v', 'r', 'spline']
        self.states_from_sensor = [True, True, True, True, True, True, False]  # , True, True
        self.states_from_sensor_at_infeasible = [True, True, True, True, True, True, False]

        # Input: forces sent to the four thrusters
        self.inputs = ['f1', 'f2', 'f3', 'f4', 'slack']
        self.inputs_to_vehicle = [True, True, True, True, False]

    def continuous_model(self, x, u):
        # States
        psi = x[2]
        u_velocity = x[3]
        r_yawrate = x[5]

        # Inputs
        f1 = u[0]
        f2 = u[1]

        # Parameters
        # Diagonal mass matrix
        m11 = 12
        m33 = 1.5
        # Dimensions boat
        aa = 0.45  # This should be 2 for the fullscale model.

        #  Calculate position and orientation derivative
        dx = u_velocity * casadi.cos(psi)
        dy = u_velocity * casadi.sin(psi)
        dpsi = r_yawrate

        # Calculate velocity vector derivative
        du = 1 / m11 * (f1 + f2)
        dv = 0
        dr = 1 / m33 * ((f1 - f2) * aa / 2)
        dspline = u_velocity

        return np.array([dx, dy, dpsi, du, dv, dr, dspline])


# Nonlinear drone model with linearized attitude dynamics
class DroneModel(DynamicModel):

    def __init__(self, system):
        self.nu = 4
        self.nx = 10
        super(DroneModel, self).__init__(system)

        self.states = ['x', 'y', 'z', 'vx', 'vy', 'vz', 'phi', 'theta', 'psi', 'spline']
        self.states_from_sensor = [True, True, True, True, True, True, True, True, True, False]
        self.states_from_sensor_at_infeasible = [True, True, True, True, True, True, True, True, True, False]

        self.inputs = ['phi_c', 'theta_c', 'dpsi_c', 'thrust_c']
        self.control_inputs = ['phi_c', 'theta_c', 'dpsi_c', 'thrust_c']

    def continuous_model(self, x, u):
        # States
        vx = x[3]
        vy = x[4]
        vz = x[5]
        phi = x[6]
        theta = x[7]
        psi = x[8]

        # Inputs
        phi_c = u[0]     # Commanded roll angle around body frame x-axis [rad]
        theta_c = u[1]   # Commanded pitch angle around body frame y-axis [rad]
        dpsi_c = u[2]    # Commanded yaw rate around body frame z-axis [rad/s]
        thrust_c = u[3]  # Commanded acceleration along body frame z-axis [m/s^2]
        # Note: thrust_c = desired thrust (N) / mass (kg)
        # The final commanded thrust to PX4 is T_c in [0, 1]
        # Find out relation between thrust_c and T_c by:
        # - Hovering experiments with different mass: (m+delta_m)*g = T_c' = m*(g+thrust_c)
        # - Measuring PWM-thrust relation with force sensor: [0, 1] maps linearly to PWM values

        # Identified constants
        kD_x = 0.02  # TODO: minimum value from ETH paper
        kD_y = 0.02  # TODO: minimum value from ETH paper
        k_phi = 1.04
        tau_phi = 0.17
        k_theta = 1.04
        tau_theta = 0.17

        # Constants
        m = 1.9
        g = 9.81

        # Calculate derivatives of linear model
        # dx = vx
        # dy = vy
        # dz = vz
        # dvx = g * theta
        # dvy = -g * phi
        # dvz = thrust_c - g
        # dphi = (k_phi * phi_c - phi) / tau_phi
        # dtheta = (k_theta * theta_c - theta) / tau_theta
        # ddpsi = dpsi_c
        # dspline = vx  # TODO Change this value to something else

        # Calculate derivatives of nonlinear model
        dx = vx
        dy = vy
        dz = vz
        dvx = thrust_c * (casadi.sin(phi) * casadi.sin(psi) + casadi.cos(phi) * casadi.sin(theta) * casadi.cos(psi))
        dvy = thrust_c * (-casadi.sin(phi) * casadi.cos(psi) + casadi.cos(phi) * casadi.sin(theta) * casadi.sin(psi))
        dvz = thrust_c * (casadi.cos(phi) * casadi.cos(theta)) - g
        dphi = (k_phi * phi_c - phi) / tau_phi
        dtheta = (k_theta * theta_c - theta) / tau_theta
        ddpsi = dpsi_c
        dspline = vx  # TODO Change this value to something else

        return np.array([dx, dy, dz, dvx, dvy, dvz, dphi, dtheta, ddpsi, dspline])
