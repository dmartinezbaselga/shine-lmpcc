"""
Main system specific file to create an MPC optimization solver using Forces Pro
"""

# Add forces to the path here.
# The forces client files should be under "python_forces_code/forces"!
# Or needs to be in the path already
import sys, os, shutil
sys.path.append("../")
sys.path.append("")

import helpers

# If your forces is in this directory add it
helpers.load_forces_path()

dir_path = os.path.dirname(os.path.realpath(__file__))

import numpy as np
import forcespro.nlp

import dynamics, systems
import objective
import generate_cpp_files
from prius import prius_settings as settings

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
    robot = systems.Roboat()   # Bounds and system name
    model = dynamics.RoboatModel(robot)

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

    # Stage-wise definitions
    # Note that we use solver.N = N_bar here!
    for i in range(0, solver.N):
        # Although optimizing the first stage does not contribute to anything, it is fine.
        # We also do not really have to optimize the final input, but it only effects the runtimes
        if i < settings.N_bar - 1:  # should be ignored already, but to be sure, we can ignore the objective in the final stage

            # Python cannot handle lambdas without an additional function that manually creates the lambda with the correct value
            def objective_with_stage_index(stage_idx):
                return lambda z, p: objective.objective(z, p, model, settings, stage_idx)


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

    # -- PRIMAL DUAL INTERIOR POINT (Default Solver!) -- #
    options.maxit = 200  # Maximum number of iterations
    options.mu0 = 20  # Do not set this for SQP!

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
