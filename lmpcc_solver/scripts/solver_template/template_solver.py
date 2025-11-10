"""
TEMPLATE
Main system specific file to create an MPC optimization solver using Forces Pro
"""

# Add forces to the path here.
# The forces client files should be under "python_forces_code/forces"!
# Or needs to be in the path already
import sys, os, shutil
import helpers

sys.path.append("../")
sys.path.append("")

# If your forces is in this directory add it
helpers.load_forces_path()

import numpy as np
import forcespro.nlp

import dynamics, systems
import objective
import generate_cpp_files
from solver_template import template_settings as settings

if __name__ == '__main__':

    # Set to False to only generate C++ code
    generate_solver = True

    print("--- Starting Model creation ---")

    '''
        Important: Problem formulation
        - Forces considers the first input/state (k = 1 in docs, k = 0 in python) to be the INITIAL state, which
        should not be constrained. I.e., all inequalities are shifted one step (k = 1, ..., k = N in python)

        - Because we want to optimize the final stage, while forces optimizes until N-1 in docs (N-2 in Python), 
        we need to add an additional stage at the end for x_N in python (i.e., N+1 in docs). Note that this stage does 
        not do anything, it just exist so we can optimize the rest 

        -> For these two reasons, define N_bar (forces horizon) versus N our horizon:
        N_bar = N + 2 (i.e., one stage added for the initial stage and one for the final optimized stage)
    '''
    settings.N_bar = settings.N + 2

    # Systems to control
    robot = # @todo <SYSTEM_NAME> Pick system from "systems.py"
    model = # @todo <DYNAMIC_MODEL_NAME> Pick dynamics from "dynamics.py"
    model.interfaces = settings.interfaces

    print(model)    # Print the robot and model
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

    # Stage-wise definitions
    # Note that we use solver.N = N_bar here!
    for i in range(0, solver.N):
        # Although optimizing the first stage does not contribute to anything, it is fine.
        # We also do not really have to optimize the final input, but it only effects the runtimes
        if i < settings.N_bar - 1:

            # Python cannot handle lambdas without an additional function that manually creates the lambda with the correct value
            def objective_with_stage_index(stage_idx):
                return lambda z, p: # @todo Pick an objective from objective.py (e.g., objective.objective(z, p, model, settings, stage_idx))

            solver.objective[i] = objective_with_stage_index(i)

        # For all stages after the initial stage (k = 0) and ignoring the final stage (k = N_bar-1), specify inequalities
        if (i > 0) and (i < solver.N - 1):
            solver.ineq[i] = lambda z, p: settings.modules.inequality_constraints(z,
                                                                                  param=p,
                                                                                  model=model,
                                                                                  settings=settings)
            solver.nh[i] = settings.nh

            solver.hu[i] = settings.modules.constraint_manager.upper_bound
            solver.hl[i] = settings.modules.constraint_manager.lower_bound
        else:
            solver.nh[i] = 0  # No constraints here

    # Dynamical constraints
    solver.eq = lambda z: dynamics.discrete_dynamics(z, model, settings.integrator_stepsize)
    solver.E = np.concatenate([np.zeros((model.nx, model.nu)), np.eye(model.nx)], axis=1)

    # States that need to be initialized at runtime
    solver.xinitidx = range(model.nu, model.nvar)

    # ==== Solver options ==== #
    options = forcespro.CodeOptions(robot.name + 'FORCESNLPsolver')
    options.printlevel = 0  # 1 = timings, 2 = print progress
    options.optlevel = 3  # 0 no optimization, 1 optimize for size, 2 optimize for speed, 3 optimize for size & speed
    options.timing = 1
    options.overwrite = 1
    options.cleanup = 1
    # options.init = 0 # Warm start?

    if not settings.use_sqp_solver:
        # -- PRIMAL DUAL INTERIOR POINT (Default Solver!) -- #
        options.maxit = 300  # Maximum number of iterations
        options.mu0 = 20  # Do not set this for SQP!
    else:
        # -- SQP (Useful under strict computation time limits) -- #
        # https://forces.embotech.com/Documentation/high_level_interface/index.html#sequential-quadratic-programming-algorithm
        # https://forces.embotech.com/Documentation/examples/robot_arm_sqp/index.html#sec-robot-arm-sqp
        options.solvemethod = "SQP_NLP"

        # Number of QPs to solve, default is 1. More iterations is higher optimality but longer computation times
        options.sqp_nlp.maxqps = 1
        options.maxit = 100  # This should limit the QP iterations, but does not seem to work

        # Tolerances
        options.sqp_nlp.TolStat = 1e-3  # inf norm tol. on stationarity
        options.sqp_nlp.TolEq = 1e-3    # tol. on equality constraints
        options.nlp.TolIneq = 1e-6

        options.sqp_nlp.qpinit = 0 # 1 = centered start, 0 = cold start

        # Should be a faster robust linear system solver
        options.nlp.linear_solver = 'symm_indefinite_fast'

        # Increasing helps when exit code is -8
        options.sqp_nlp.reg_hessian = 5e-9  # = default

        # The BFGS guess speeds up the solver
        options.exportBFGS = 1
        options.nlp.parametricBFGSinit = 1  # Allows us to initialize the estimate at run time with the exported one

        # This initial BFGS comes from the lmpcc running when print_init_bfgs is true
        options.nlp.bfgs_init = # @todo Get this value from the C++ controller (initially you can use np.eye())  np.diag(np.array([0.698998, 3.79421, 0.34662, 0.0635451, 1.13039, 0.365216, 0.226923, 0.253899]))
        settings.bfgs_init = options.nlp.bfgs_init

        # Disables checks for NaN and Inf (only use this if your optimization is working)
        options.nlp.checkFunctions = 0

    if generate_solver:
        print("--- Generating solver ---")

        # Remove the previous solver
        dir_path = os.path.dirname(os.path.realpath(__file__))
        solver_path = dir_path + "/" + robot.name + 'FORCESNLPsolver'
        new_solver_path = dir_path + "/../" + robot.name + 'FORCESNLPsolver'
        print("Path of the new solver: {}".format(new_solver_path))
        if os.path.exists(new_solver_path) and os.path.isdir(new_solver_path):
            shutil.rmtree(new_solver_path)

        # Creates code for symbolic model formulation given above, then contacts server to generate new solver
        generated_solver = solver.generate_solver(options)

        # Move the solver up a directory for convenience
        if os.path.isdir(solver_path):
            shutil.move(solver_path, new_solver_path)

    # Generate C++ code
    generate_cpp_files.write_model_header(settings, model)
