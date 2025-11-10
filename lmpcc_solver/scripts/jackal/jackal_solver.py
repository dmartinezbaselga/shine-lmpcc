# The forces client files should be under "python_forces_code/forces"!
# Or needs to be in the path already
import os
import shutil
import sys

sys.path.append("../")
sys.path.append("")

import helpers

# If your forces is in this directory add it
helpers.load_forces_path()

import copy

import forcespro.nlp
import numpy as np

import dynamics
import generate_cpp_files
import objective
import systems
from jackal import jackal_settings as settings

# Press the green button in the gutter to run the script.
if __name__ == "__main__":

    # Set to False to only generate C++ code
    # generate_solver = True
    if len(sys.argv) > 1:
        generate_solver = sys.argv[1].lower() == "true"
    else:
        generate_solver = False

    # Also activate if the second argument

    print("--- Starting Model creation ---")
    print(
        "Using Configuration: "
        + helpers.bcolors.OKGREEN
        + settings.configuration
        + helpers.bcolors.ENDC
    )

    """
        Important: Problem formulation
        - Forces considers the first input/state (k = 1 in docs, k = 0 in python) to be the INITIAL state, which
        should not be constrained. I.e., all inequalities are shifted one step (k = 1, ..., k = N in python)
         
        - Because we want to optimize the final stage, while forces optimizes until N-1 in docs (N-2 in Python), 
        we need to add an additional stage at the end for x_N in python (i.e., N+1 in docs). Note that this stage does 
        not do anything, it just exist so we can optimize the rest 
        
        -> For these two reasons, define N_bar (forces horizon) versus N our horizon:
        N_bar = N + 2 (i.e., one stage added for the initial stage and one for the final optimized stage)
    """
    settings.N_bar = settings.N + 2

    # Systems to control
    robot = settings.robot
    model = settings.model
    model.interfaces = settings.interfaces

    print(model)
    print(settings.modules)

    # Load model parameters from the settings
    solver = forcespro.nlp.SymbolicModel(settings.N_bar)
    solver.N = settings.N_bar  # prediction/planning horizon
    solver.nvar = model.nvar  # number of online variables
    solver.neq = model.nx  # number of equality constraints
    solver.npar = settings.npar

    # Bounds
    solver.lb = model.lower_bound()
    solver.ub = model.upper_bound()

    # Functions used in the optimization
    # Note that we use solver.N = N_bar here!
    for i in range(0, solver.N):
        # Although optimizing the first stage does not contribute to anything, it is fine.
        # We also do not really have to optimize the final input, but it only effects the runtimes
        if (
            i < settings.N_bar - 1
        ):  # should be ignored already, but to be sure, we can ignore the objective in the final stage

            # Python cannot handle lambdas without an additional function that manually creates the lambda with the correct value
            def objective_with_stage_index(stage_idx):
                return lambda z, p: objective.objective(
                    z, p, model, settings, stage_idx
                )

            solver.objective[i] = objective_with_stage_index(i)

        # For all stages after the initial stage (k = 0) and ignoring the final stage (k = N_bar-1), we specify inequalities
        if (i > 0) and (i < solver.N - 1):
            solver.ineq[i] = lambda z, p: settings.modules.inequality_constraints(
                z=z, param=p, model=model, settings=settings
            )
            solver.nh[i] = settings.nh

            solver.hu[i] = settings.modules.constraint_manager.upper_bound
            solver.hl[i] = settings.modules.constraint_manager.lower_bound
        else:
            solver.nh[i] = 0  # No constraints here

    # Equalities are specified on all stages
    solver.eq = lambda z: dynamics.discrete_dynamics(
        z, model, settings.integrator_stepsize
    )
    solver.E = np.concatenate(
        [np.zeros((model.nx, model.nu)), np.eye(model.nx)], axis=1
    )

    # Initial stage (k = 0) specifies the states
    solver.xinitidx = range(model.nu, model.nvar)

    # Set solver options
    options = forcespro.CodeOptions(robot.name + "FORCESNLPsolver")
    options.printlevel = 0  # 1 = timings, 2 = print progress
    options.optlevel = 3  # 0 no optimization, 1 optimize for size, 2 optimize for speed, 3 optimize for size & speed
    options.timing = 1
    options.overwrite = 1
    options.cleanup = 1

    # At infeasibility the optimization is not converging, going over the time limit, so give it a maximum time
    options.solver_timeout = 1

    # Internal parallelism
    # options.parallel = 8
    # options.nlp.max_num_threads = 8 # This one is not set?

    # External parallelism
    # https://forces.embotech.com/Documentation/parallel/index.html?highlight=max_num_threads
    if settings.multi_solver:
        options.threadSafeStorage = 1
        options.nlp.max_num_threads = 5  # Todo: Set this (needs to be the maximum number of threads to parallelize over)

    # options.solver_exit_external = 1 # Should exist but doesnt

    # Todo: Make solver options a lambda defined in the settings
    if not settings.use_sqp_solver:
        # -- PRIMAL DUAL INTERIOR POINT (Default Solver!) -- #
        options.maxit = 500  # Maximum number of iterations
        options.mu0 = 20  # IMPORTANT: CANNOT BE 20 FOR SQP!
        options.nlp.TolIneq = 1e-6
        
        options.init = 2 # Warmstart with specified primal variables!

    else:
        # -- SQP (Useful under strict computation time limits) -- #
        # https://forces.embotech.com/Documentation/high_level_interface/index.html#sequential-quadratic-programming-algorithm
        # https://forces.embotech.com/Documentation/examples/robot_arm_sqp/index.html#sec-robot-arm-sqp
        options.solvemethod = "SQP_NLP"

        # Number of QPs to solve, default is 1. More iterations is higher optimality but longer computation times
        options.sqp_nlp.maxqps = 1
        options.maxit = 100  # This should limit the QP iterations, but if it does I get -8: QP cannot proceed

        # Tolerances
        options.sqp_nlp.TolStat = 1e-3  # inf norm tol. on stationarity
        options.sqp_nlp.TolEq = 1e-3  # tol. on equality constraints
        options.nlp.TolIneq = 1e-6

        options.sqp_nlp.qpinit = 0  # 1 # 1 = centered start, 0 = cold start

        # Should be a faster robust linear system solver (default in the future they say)
        options.nlp.linear_solver = "symm_indefinite_fast"

        # options.parallel = 8

        # Increasing helps when exit code is -8
        options.sqp_nlp.reg_hessian = 5e-9  # = default
        options.exportBFGS = (
            1  # Todo: Make this "2" later for lower triangular instead of diagonal
        )

        options.nlp.parametricBFGSinit = (
            1  # Allows us to initialize the estimate at run time with the exported one
        )

        # Speeding up the solver
        # Makes a huge difference (obtained from the exported BFGS)
        # options.nlp.bfgs_init = np.diag(np.array([0.000408799, 0.398504, 1.22784, 0.697318, 1.27293, 0.0718911, 0.0718882, 0.980375, 0.19122]))
        # options.nlp.bfgs_init = np.diag(np.array([1.27245e-05, 0.508587, 1.62729, 0.697318, 1.27512, 0.0718914, 0.0718882, 0.981584, 0.400975]))
        # options.nlp.bfgs_init = np.diag(np.array([0.0373602, 0.646782, 0.612693, 0.697318, 0.9781, 0.0718882, 0.0718882, 0.772672, 0.534718]))
        # 0.0622294, 0.105641, 0.799462, 0.697318, 0.978434, 0.196775,
        # 0.166427, 0.605408, 0.424102, 0.616312, 0.285552, 0.202704,
        # options.nlp.bfgs_init = np.diag(np.array([0.130528, 0.275461, 0.287056, 0.0895459, 1.10396, 0.306627]))

        options.nlp.bfgs_init = np.eye(model.nx + model.nu)  # @note: WRONG!

        # np.diag(np.array([0.0657674, 0.277346, 0.373907, 0.649418, 1.30917, 0.650918]))
        settings.bfgs_init = options.nlp.bfgs_init

        # Disables checks for NaN and Inf (only use this if your optimization is working)
        options.nlp.checkFunctions = 0

    if generate_solver:
        print("--- Generating solver ---")

        # Remove the previous solver
        dir_path = os.path.dirname(os.path.realpath(__file__))
        solver_path = dir_path + "/" + robot.name + "FORCESNLPsolver"
        new_solver_path = dir_path + "/../" + robot.name + "FORCESNLPsolver"
        print("Path of the new solver: {}".format(new_solver_path))
        if os.path.exists(new_solver_path) and os.path.isdir(new_solver_path):
            shutil.rmtree(new_solver_path)

        # Creates code for symbolic model formulation given above, then contacts server to generate new solver
        generated_solver = solver.generate_solver(options)  # , outputs

        # Move the solver up a directory for convenience
        if os.path.isdir(solver_path):
            shutil.move(solver_path, new_solver_path)

    generate_cpp_files.write_model_header(settings, model)

    if not generate_solver:
        print(
            helpers.bcolors.WARNING
            + 'Warning: Not generating a new solver "generate_solver" is set to "False"'
            + helpers.bcolors.ENDC
        )
