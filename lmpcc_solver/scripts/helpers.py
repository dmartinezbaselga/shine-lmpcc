import numpy as np
import casadi
import os, sys
import inspect

def load_forces_path():

    print_paths = ["PYTHONPATH"]
    # Is forces in the python path?
    try:
        import forcespro.nlp
        print('Forces found in PYTHONPATH')

        return
    except:
        pass

    paths = [os.path.join(os.path.expanduser("~"), "forces_pro_client"),
             os.path.join(os.getcwd(), "forces"),
             os.path.join(os.getcwd(), "../forces"),
             os.path.join(os.getcwd(), "forces/forces_pro_client"),
             os.path.join(os.getcwd(), "../forces/forces_pro_client")]
    for path in paths:
        if check_forces_path(path):
            return
        print_paths.append(path)

    print('Forces could not be imported, paths tried:\n')
    for path in print_paths:
        print('{}'.format(path))
    print("\n")


def check_forces_path(forces_path):
    # Otherwise is it in a folder forces_path?
    try:
        if os.path.exists(forces_path) and os.path.isdir(forces_path):
            sys.path.append(forces_path)
        else:
            raise IOError("Forces path not found")

        import forcespro.nlp
        print('Forces found in: {}'.format(forces_path))

        return True
    except:
        return False


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class ParameterStructure:

    def __init__(self):
        self.parameters = dict()
        self.organization = dict() # Lists parameter grouping and indices
        self.param_idx = 0

    def add_parameter(self, name):
        self.organization[self.param_idx] = 1
        self.parameters[self.param_idx] = name
        setattr(self, name+ "_index", self.param_idx)
        self.param_idx += 1

    def add_multiple_parameters(self, name, amount):
        self.organization[self.param_idx] = amount
        for i in range(amount):
            self.parameters[self.param_idx] = name + "_" + str(i)
            setattr(self, name + "_" + str(i) + "_index", self.param_idx)
            self.param_idx += 1

    def has_parameter(self, name):
        return hasattr(self, name)

    def n_par(self): # Does not need + 1, because it is always increased
        return self.param_idx

    def __str__(self):
        result = "--- Parameter Structure ---\n"
        for idx, amount in self.organization.items():
            if amount == 1:
                result += "{}\t:\t{}\n".format(idx, self.parameters[idx])
            else:
                result += "{}\t:\t{} x{}\n".format(idx, '_'.join(self.parameters[idx].split('_')[:-1]), amount)

        result += "--------------------\n"
        return result

    # When operating, retrieve the weights from param
    def load_params(self, params):
        for key, name in self.parameters.items(): # This is a parameter name
            setattr(self, name, params[getattr(self, name+ "_index")]) # this is an index

    # This makes a new parameter structure where the given weights in the weight_list are at the start
    def prepend_weights_to_parameters(self, weight_list):
        resulting_params = ParameterStructure()
        for weight in weight_list: # Add all the weights
            resulting_params.add_parameter(weight)

        for param_idx in self.organization: # Add all the parameters
            if self.organization[param_idx] > 1: # (As multiple when there were multiple)
                resulting_params.add_multiple_parameters(self.parameters[param_idx].split("_")[0], # The split removes the _0 part
                                                       self.organization[param_idx])
            else:
                resulting_params.add_parameter(self.parameters[param_idx])

        # Return the
        return resulting_params

class WeightStructure:

    # When defining the structure we define a structure with the weights as variables
    def __init__(self, parameters, weight_list):
        self.weight_list = weight_list

        for idx, weight in enumerate(weight_list):
            setattr(self, weight + "_index", parameters.param_idx)
            parameters.add_parameter(weight + "_weight")

        self.npar = len(weight_list)
        # parameters

    # When operating, retrieve the weights from param
    def set_weights(self, param):
        for weight in self.weight_list:
            # print(weight + ": " + str(getattr(self, weight+"_index")))
            setattr(self, weight , param[getattr(self, weight+"_index")])

    def has_weight(self, name):
        return name in self.weight_list


def rotation_matrix(angle):
    return np.array([[casadi.cos(angle), -casadi.sin(angle)],
                      [casadi.sin(angle), casadi.cos(angle)]])

class SplineParameters:

    def __init__(self, param, spline_nr):

        # Retrieve spline values from the parameters (stored as multi parameter by name)
        self.x_a = getattr(param, 'spline' + str(spline_nr) + '_0')
        self.x_b = getattr(param, 'spline' + str(spline_nr) + '_1')
        self.x_c = getattr(param, 'spline' + str(spline_nr) + '_2')
        self.x_d = getattr(param, 'spline' + str(spline_nr) + '_3')

        self.y_a = getattr(param, 'spline' + str(spline_nr) + '_4')
        self.y_b = getattr(param, 'spline' + str(spline_nr) + '_5')
        self.y_c = getattr(param, 'spline' + str(spline_nr) + '_6')
        self.y_d = getattr(param, 'spline' + str(spline_nr) + '_7')

        self.s_start = getattr(param, 'spline_start' + str(spline_nr))

    def compute_path(self, spline_index):
        self.path_x = self.x_a * (spline_index - self.s_start) ** 3 + \
                      self.x_b * (spline_index - self.s_start) ** 2 + \
                      self.x_c * (spline_index - self.s_start) + \
                      self.x_d

        self.path_y = self.y_a * (spline_index - self.s_start) ** 3 + \
                      self.y_b * (spline_index - self.s_start) ** 2 + \
                      self.y_c * (spline_index - self.s_start) + \
                      self.y_d

        self.path_dx = 3 * self.x_a * (spline_index - self.s_start) ** 2 + \
                       2 * self.x_b * (spline_index - self.s_start) + \
                       self.x_c

        self.path_dy = 3 * self.y_a * (spline_index - self.s_start) ** 2 + \
                       2 * self.y_b * (spline_index - self.s_start) + \
                       self.y_c