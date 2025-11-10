import helpers
import inequality
import objective


def initialize_solver():
    params = helpers.ParameterStructure()
    weight_list = list()  # Anything in the weight list is updated through rqt_reconfigure

    modules = ModuleManager(params)
    modules.add_module(MPCBaseModule(params, weight_list))  # Adds weights to the overall weight list
    weights = modules.get_last_added_module().weights

    return params, modules, weight_list, weights


def end_objectives_start_constraints(params, weight_list, n_discs):
    params = params.prepend_weights_to_parameters(weight_list)

    # Always load the disc radius and offsets
    # params.add_parameter("disc_r")
    # params.add_multiple_parameters("disc_offset", n_discs)
    return params


class ModuleManager:
    """
    The idea of modules is that they can include multiple constraint sets if necessary
    In addition, they are directly linked to the c++ code module
    """

    def __init__(self, params):
        self.constraint_manager = inequality.Constraints(params)
        self.modules = []

        self.params = params

    def add_module(self, module):
        self.modules.append(module)

        if module.type == "constraint":
            for constraint in module.constraints:
                self.constraint_manager.add_constraint(constraint)

    def inequality_constraints(self, z, param, settings, model):
        return self.constraint_manager.inequality(z, param, settings, model)

    def number_of_constraints(self):
        return self.constraint_manager.nh

    def get_last_added_module(self):
        return self.modules[-1]

    def contains_module(self, class_name):
        for module in self.modules:
            if type(module) == class_name:
                return True

        return False

    def __str__(self):
        result = "--- MPC Modules ---\n"
        for module in self.modules:
            result += str(module) + "\n"

        return result


class Module:

    def __init__(self):
        self.module_name = "UNDEFINED"
        self.description = ""
        self.submodules = []

    def write_to_solver_interface(self, header_file):
        return

    def __str__(self):
        result = self.type.capitalize() + ": " + self.module_name + " - " + self.description
        return result

    def add_submodule(self, submodule):

        if not hasattr(self, "submodules"):
            self.submodules = []

        self.submodules.append(submodule)
        if hasattr(submodule, "constraints"):
            self.constraints += submodule.constraints

        if hasattr(submodule, "objectives"):
            self.objectives += submodule.objectives


""" MODULE DEFINITIONS """

""" OBJECTIVE MODULES """
class MPCBaseModule(Module):

    """
    Weight states and inputs of an MPC problem
    """

    def __init__(self, params, weight_list):
        self.module_name = "MPCBaseModule"  # Needs to correspond to the c++ name of the module
        self.import_name = "modules_objectives/mpc_base.h"
        self.type = "objective"
        self.description = "Contains input and state penalties with weights that can be tuned in rqt_reconfigure"

        self.objectives = []
        self.objectives.append(objective.WeightsObjective(params, weight_list))

        self.weights = self.objectives[-1]

class ContouringModule(Module):

    """
    Track a reference path with contouring control
    """

    def __init__(self, params, weight_list, n_segments):
        self.module_name = "ReferencePath"  # Needs to correspond to the c++ name of the module
        self.import_name = "modules_objectives/reference_path.h"
        self.type = "objective"
        self.description = "Tracks a 2D reference path with contouring costs"

        self.n_segments = n_segments

        self.objectives = []
        self.objectives.append(objective.ContouringObjective(params, weight_list, self.n_segments))

    # def write_to_solver_interface(self, header_file, cpp_file):
        # header_file.write("\tint n_segments_ = " + str(self.n_segments) + ";\n")

class GoalTrackingModule(Module):

    """
    Track a reference path with contouring control
    """

    def __init__(self, params, weight_list):
        self.module_name = "GoalTracking"  # Needs to correspond to the c++ name of the module
        self.import_name = "modules_objectives/goal_tracking.h"
        self.type = "objective"
        self.description = "Tracks a goal in 2D"

        self.objectives = []
        self.objectives.append(objective.GoalTrackingObjective(params, weight_list))

class PreviewContouringModule(Module):

    """
    Track a reference path with contouring control
    """

    def __init__(self, params, weight_list, n_segments, T):
        self.module_name = "PreviewContouring"  # Needs to correspond to the c++ name of the module
        # self.import_name = "modules_objectives/reference_path.h"
        self.depends = [ContouringModule]
        self.type = "objective"
        self.description = "Terminal cost for tracking a 2D reference path after the horizon"

        self.n_segments = n_segments
        self.T = T  # How much seconds ahead?

        self.objectives = []
        self.objectives.append(objective.PreviewObjective(params, weight_list, self.n_segments, T))

class HomotopyGuidanceObjectiveModule(Module):

    """
    Homotopic path search in the state space for generating guidance trajectories
    """

    def __init__(self, params, weight_list, n_segments, n_discs, max_obstacles, constraint_submodule=None):

        if constraint_submodule is None:
            constraint_submodule = EllipsoidalConstraintModule

        self.module_name = "GuidanceObjective"  # Needs to correspond to the c++ name of the module
        self.import_name = "modules_objectives/guidance_objective.h"
        self.type = "objective"

        self.n_segments = n_segments

        self.objectives = []  # No explicit objective for now
        # self.objectives.append(objective.GoalTrackingObjective(params, weight_list))
        self.objectives.append(
            objective.ContouringObjective(params, weight_list, n_segments)
        )  # We have a contouring objective for tracking the path
        self.objectives.append(objective.ReferenceVelocityObjective(params, weight_list))  # Velocity tracking for trajectory tracking

        # self.constraints = []
        # self.add_submodule(constraint_submodule(params, n_discs, max_obstacles=max_obstacles))

        self.description = "Experimental"
        # (
        # "Tracks multiple guidance trajectories in parallel (with multiple MPC)\n"
        # + "\t\t- Underlying Constraint: "
        # + self.submodules[0].description
        # )

    def write_to_solver_interface(self, header_file):
        pass
        # assert len(self.submodules) == 1, "GuidanceObjective can use one type of submodule for collision avoidance constraints"
        # header_file.write("#define GUIDANCE_CONSTRAINTS_TYPE " + self.submodules[0].module_name + "\n")


class VelocityReferenceModule(Module):
    """
    Track a reference path with contouring control
    """

    def __init__(self, params, weight_list):
        self.module_name = "ReferenceVelocityModule"  # Needs to correspond to the c++ name of the module
        self.import_name = "modules_objectives/reference_velocity.h"
        self.type = "objective"
        self.description = "Tracks a dynamic velocity reference."

        self.objectives = []
        self.objectives.append(objective.ReferenceVelocityObjective(params, weight_list))


class PathVelocityReferenceModule(Module):
    """
    Track a reference velocity in the direction of the path
    """

    def __init__(self, params, weight_list, n_segments):
        self.module_name = "ReferenceVelocityModule"  # Needs to correspond to the c++ name of the module
        self.import_name = "modules_objectives/reference_velocity.h"
        self.type = "objective"
        self.description = "Tracks a static velocity reference in the direction of the path."

        self.objectives = []
        self.objectives.append(objective.PathReferenceVelocityObjective(params, weight_list, n_segments))


""" CONSTRAINT MODULES """
# For now keep this as parameters in the settings itself
# class CollisionRegionModule(Module):
#
#     def __init__(self, params, n_discs):
#         params.add_parameter("disc_r")
#         params.add_multiple_parameters("disc_offset", n_discs)


class ScenarioConstraintModule(Module):

    """
    Linear constraints for scenario-based motion planning
    """

    def __init__(self, params, n_discs, max_obstacles=0):
        self.module_name = "ScenarioConstraints"  # Needs to correspond to the c++ name of the module
        self.import_name = "modules_constraints/scenario_constraints.h"
        self.type = "constraint"
        self.description = "Avoid dynamic obstacles under motion uncertainty using scenario optimization."

        self.constraints = []
        self.constraints.append(inequality.LinearConstraints(params, n_discs, 24, "scenario_constraint"))


class HomotopyGuidanceConstraintModule(Module):

    """
    Homotopic path search in the state space for generating guidance trajectories
    In the "Constraint" version, we use these trajectories to initialize the planner and
    to linearize the collision avoidance constraints
    """

    def __init__(self, params, n_discs, max_obstacles, static_obstacles, constraint_submodule=None):
        if constraint_submodule is None:
            constraint_submodule = EllipsoidalConstraintModule

        self.module_name = "GuidanceConstraints"  # Needs to correspond to the c++ name of the module
        self.import_name = "modules_constraints/guidance_constraints.h"

        self.type = "constraint"
        self.constraints = []
        self.constraints.append(inequality.LinearConstraints(params, n_discs, max_obstacles + static_obstacles))
        self.add_submodule(constraint_submodule(params, n_discs, max_obstacles))

        self.description = (
            "Solves the planning problem in parallel for constraints linearized with respect to a set of "
            + "guidance trajectories \n"
            + "\t\t- Underlying Constraint: "
            + self.submodules[0].description
        )

        # self.constraints.append(low_level_constraints(params, n_discs, max_obstacles)) # @todo: Add a module here instead!
        # self.cpp_constraint_name = cpp_constraint_name # Necessary for now to run the correct C++ module

    def write_to_solver_interface(self, header_file):
        assert len(self.submodules) == 1, "GuidanceConstraints expects one type of submodule for collision avoidance constraints"
        header_file.write("#define GUIDANCE_CONSTRAINTS_TYPE " + self.submodules[0].module_name + "\n")


class GaussianConstraintModule(Module):

    """
    Linear constraints for scenario-based motion planning
    """

    def __init__(self, params, n_discs, max_obstacles):
        self.module_name = "GaussianConstraints"  # Needs to correspond to the c++ name of the module
        self.import_name = "modules_constraints/gaussian_constraints.h"
        self.type = "constraint"
        self.description = "Avoid dynamic obstacles under Gaussian motion uncertainty using the 1D CDF."

        self.constraints = []
        self.constraints.append(inequality.GaussianConstraints(n_discs, max_obstacles, params))


class EllipsoidalConstraintModule(Module):

    """
    Ellipsoidal Constraints for collision avoidance
    Data: Obstacle position, obstacle and vehicle radius, Gaussian 2D mean and variance
    """

    def __init__(self, params, n_discs, max_obstacles):
        self.module_name = "EllipsoidalConstraints"  # Needs to correspond to the c++ name of the module
        self.import_name = "modules_constraints/ellipsoidal_constraints.h"
        self.type = "constraint"
        self.description = "Avoid dynamic obstacles described as ellipsoids."

        self.constraints = []
        self.constraints.append(inequality.EllipsoidConstraints(n_discs, max_obstacles, params))


class LinearizedEllipsoidalConstraintModule(Module):

    """
    Linearized Ellipsoidal Constraints for collision avoidance
    Data: Obstacle position, obstacle and vehicle radius
    """

    def __init__(self, params, n_discs, max_obstacles):
        self.module_name = "EllipsoidalConstraints"  # Needs to correspond to the c++ name of the module
        self.import_name = "modules_constraints/ellipsoidal_constraints.h"
        self.type = "constraint"
        self.description = "Avoid dynamic obstacles described as ellipsoids, but where the constraints are linearized."

        self.constraints = []
        self.constraints.append(inequality.EllipsoidLinearizedConstraints(n_discs, max_obstacles, params))


class LinearizedConstraintsModule(Module):
    """
    Linear constraints for scenario-based motion planning
    Data: A, b in A^Tx <= b
    """

    def __init__(self, params, n_discs, max_obstacles, static_obstacles):
        self.module_name = "LinearizedConstraints"  # Needs to correspond to the c++ name of the module
        self.import_name = "modules_constraints/linearized_constraints.h"
        self.type = "constraint"
        self.description = "Avoid dynamic obstacles by computing a linear constraint for each obstacle that divides them from the robot."

        self.constraints = []
        self.constraints.append(inequality.LinearConstraints(params, n_discs, max_obstacles + static_obstacles))


class BoundaryYModule(Module):
    """
    Linear constraints for scenario-based motion planning
    Data: A, b in A^Tx <= b
    """

    def __init__(self, params, n_discs, width):
        self.module_name = "BoundaryYModule"  # Needs to correspond to the c++ name of the module
        # self.import_name = "modules_constraints/linearized_constraints.h"
        self.type = "constraint"
        self.description = "Simple road boundaries in the y-direction"

        self.constraints = []
        self.constraints.append(inequality.LinearConstraintsY(n_discs, -width/2., width/2.))

class RoomBoundaryModule(Module):
    """
    Linear constraints for scenario-based motion planning
    Data: A, b in A^Tx <= b
    """

    def __init__(self, params, n_discs, x_size, y_size, x_offset=0., y_offset=0.):
        self.module_name = "RoomBoundary"  # Needs to correspond to the c++ name of the module
        # self.import_name = "modules_constraints/linearized_constraints.h"
        self.type = "constraint"
        self.description = "Simple road boundaries for a rectanglular room."

        self.constraints = []
        self.constraints.append(inequality.RoomConstraints(n_discs, -x_size/2., x_size/2., -y_size/2., y_size/2., x_offset, y_offset))