Solver Generation (Python)
======================================

.. figure:: ../images/implementation_diagram.png
   :width: 500px
   
Files and Architecture
--------------------------------------
The Python code is written as a framework that can be easily adapted for different systems, dynamics, objectives and constraints. Two type of files are present.

**System Specific Files** define the solver for a particular system. These need to be in a separate folder, e.g., ``/lmpcc/python_forces_code/jackal/``. The standard setup contains two files:

* ``<system>_solver.py``
	Main file for the solver. Settings of the Forces Pro solver can be changed here.

* ``<system>_settings.py`` 
	Defines the solved problem (objective and constraints). Contains problem parameters (e.g., integration step, horizon length, etc.).

These files need to be added for each particular system. It is possible to reuse the same solver code on multiple systems.

**Shared Files** function as a library of functions used to define a solver. These files are

* ``dynamics.py``
	Defines the model used in the MPC

* ``control_modules.py``
	Defines control modules. Each control module can add constraints and/or objectives and refers to a C++ class that will insert parameters when the controller is running. Modules can be stacked.

* ``objective.py``
	Defines the objective (currently still system specific)

* ``inequality.py``
	Inequality constraints such as ellipsoidal or linear constraints for collision avoidance

* ``systems.py``
	Defines for each system limits. The same system may be used by more than one solver, but the limits generally stay the same

Other helpers classes for managing parameters and weights are defined in ``helpers.py``.

Template Example (solver)
--------------------------------------
Snippets of the ``<systems>_solver.py`` file are highlighted below. All of the code below may be outdated. For an up-to-date example see ``jackal_solver.py``.

First the system and model are defined.

.. code-block:: python

    # Systems to control
    robot =         # Pick system from "systems.py"
    model =         # Pick dynamics from "dynamics.py"
    print(model)    # Print the robot and model

Then the objective and inequalities are assigned.

.. warning::
	The objective is posed on all stages. In contrast, inequalities are only posed on stage 1 to N, i.e., no constraints are present in the initial or the final stage of the optimization as defined in Forces Pro.

.. code-block:: python

    # Stage-wise definitions
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

Solver options can be customized (a default set is present).

.. code-block:: python

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
        options.nlp.bfgs_init = # np.diag(np.array([0.698998, 3.79421, 0.34662, 0.0635451, 1.13039, 0.365216, 0.226923, 0.253899]))
        settings.bfgs_init = options.nlp.bfgs_init

        # Disables checks for NaN and Inf (only use this if your optimization is working)
        options.nlp.checkFunctions = 0

Template Example (settings)
--------------------------------------

Example settings are given below. The first lines typically define high-level settings.

.. code-block:: python

	# --- Main MPC Parameters --- #
	N = 20                      # MPC Horizon
	integrator_stepsize = 0.2   # Timestep of the integrator
	n_discs = 1                 # Number of discs modeling the vehicle collision region

	use_sqp_solver = False      # Note: SQP for scenario has max_it = 1
	enable_repulsive = False    # Enable repulsive fields in the objective?

	# --- Constraint Selection --- #
	use_scenario_constraints = False      # Scenario approach
	use_ellipsoid_constraints = False    # Ellipsoid obstacle models
	use_gaussian_constraints = False    # Hai's Gaussian Constraints

	# --- Scenario Settings --- #
	if use_scenario_constraints:
	    max_scenarios = 24      # Maximum linear constraints supported in the solver

	# --- Ellipsoid/Gaussian Settings --- #
	if use_ellipsoid_constraints or use_gaussian_constraints:
	    modes = 1
	    max_obstacles = 6      # Maximum dynamic obstacles to evade with the planner
	    max_obstacles *= modes  # To account for the number of modes, we need an ellipsoid per mode!

	# --- SQP Settings --- #
	if use_sqp_solver:
	    print_init_bfgs = False

Parameters of the solver are defined through params (see helpers.ParameterStructure), which keeps track of the indices of the parameters. 

.. note::
	On the C++ side these parameters should be inserted in the same order to prevent a mismatch. 

.. code-block:: python

	# === Constraint Definition === #
	params = helpers.ParameterStructure()

	# Parameters for the spline (Cubic in x, y, 3 segments)
	params.add_multiple_parameters("spline1", 8)
	params.add_multiple_parameters("spline2", 8)
	params.add_multiple_parameters("spline3", 8)
	params.add_parameter("s1")
	params.add_parameter("s2")
	params.add_parameter("s3")
	params.add_parameter("d")

The "weights" are used to weight objectives. On the C++ side this can be done dynamically via ``rqt_reconfigure``. 

.. warning::

	We currently require the configuration of ``rqt_reconfigure`` to match the weights defined in the solver.

.. code-block:: python

	modules.add_module(control_modules.MPCBaseModule(params, weight_list))  # Adds weights to the overall weight list
	weights = modules.get_last_added_module().weights

	# Velocity tracking
	weights.add("v", weight_names=["velocity", "velocity_reference"],
				cost_function=lambda x, w: w[0] * (x - w[1]) ** 2)
	weights.add("a", weight_names="acceleration")
	weights.add("w", weight_names="angular_velocity")

The inequalities typically also define real-time parameters. By convention, we use order the parameters as objectives -> inequality constraints.

.. code-block:: python

	# Parameters for the vehicle collision region
	params.add_parameter("disc_r")
	params.add_multiple_parameters("disc_offset", n_discs)

	# --- Inequality Constraints --- #
	constraints = inequality.Constraints(params)

	if use_scenario_constraints:
	    constraints.add_constraint(inequality.LinearConstraints(n_discs, max_scenarios, params))

	if use_ellipsoid_constraints:
	    constraints.add_constraint(inequality.EllipsoidConstraints(n_discs, max_obstacles, params))

	if use_gaussian_constraints:
	    constraints.add_constraint(inequality.GaussianConstraints(n_discs, max_obstacles, params))

	# === Collect Constraint Data === #
	print(params)
	npar = params.n_par()
	nh = constraints.nh

