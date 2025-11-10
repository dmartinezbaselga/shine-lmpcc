.. _getting-started:

Getting Started
==============================

Installation
+++++++++++++++++++++++++++++++++
The Forces Pro solver is generated before deploying the C++/ROS controller. The software in ``python_forces_code`` uses the Python interface of Forces to generate the solver. When the solver is created, an additional class C++ class is generated to save solver parameters and setup functionality for the online controller.

FORCES Pro
--------------------------
See also the `Forces Pro Manual <https://forces.embotech.com/Documentation/installation/python.html>`_.
To set up your Forces Pro license, follow these steps:

* Obtain a license from `the Embotech site <https://www.embotech.com/products/forcespro/licensing/>`_ (this may take 2-3 workdays)

* When your license is approved, assign your license to your computer. Make sure to use the username of your account

* Download the fingerprinter and run it on your system. Paste the fingerprint into your forces license

* Download the forces client and extract the files to (e.g., ``/lmpcc/python_forces_code/forces``)

* If you place it somewhere else, you can set the specific path location in ``/lmpcc/python_forces_code/venv/bin/activate`` by adding the line ``export PYTHONPATH="${PYTHONPATH}:<your absolute path to forces>"`` at the end

Virtual Environment
-------------------------------
Make sure venv is installed: ``sudo apt install python3-venv``. To setup the virtual environment run::

	cd lmpcc/python_forces_code
	chmod +x setup_venv.sh
	source setup_venv.sh

This may take a while. You can ignore the warning regarding the *bdist wheel*. You should now have a virtual environment ``venv/``.

.. tip::
	If the virtual environment does not work, you can install the requirements in your main python distribution or use another package manager such as conda

Generating a Solver
+++++++++++++++++++++++++++++++
First set the system to the system where you want to build the controller for (for example `Carla` or `JackalSimulator`.::

	cd lmpcc
	chmod +x 01_set_system.sh
	./01_set_system.sh <system>

Then, generate the solver::

	cd python_forces_code
	chmod +x 02_generate_solver.sh
	./02_generate_solver.sh

.. Note:: The virtual environment is automatically sourced by `02_generate_solver.sh`. If you are editing a solver, you need to source the environment in the configuration of your IDE (e.g., https://www.jetbrains.com/help/pycharm/creating-virtual-environment.html#existing-environment).

A solver will be generated (you should see the states, inputs and model being printed). The generated C++ code is placed in ``/lmpcc/python_forces_code/generated_cpp/<system>_solver.h>``

Building the LMPCC Package
+++++++++++++++++++++++++++++++
When a solver has been generated, the ROS package can be build either using (recommended)::

	catkin build lmpcc

or defining in ``~/.bashrc``::

	alias cmrel='catkin_make -DCMAKE_BUILD_TYPE=Release'

and running ``cmrel``.




 
.. # Overall Architecture
.. The solver and controller code is written to allow control of different systems with different solvers. A brief overview of the most important components:
.. - Dynamical models are defined in `python_forces_code/dynamics` and set in each `<system>_solver.py` file.
.. - Physical systems and their limits are defined in `python_forces_code/systems.py`
.. - Each `<solver_name>_solver.py` and `<solver_name>_settings.py` defines a solver with objectives, constraints and a mathematical model. A set of C++ classes with control functionality is automatically generated in `python_forces_code/generated_cpp/`.
.. - The controller interface with the system or simulation is defined in `<system/simulation>_interface.cpp`. This allows to keep the controller abstracted from system specific details such as sensors and actuators.
.. - The value of the parameters is defined in `cfg/PredictiveConfiguration_<system>.cfg` which is automatically selected. The configuration must contain all weights in the weight_list as defined in the solver settings.

.. The system type is set in CMakeLists.txt in SYSTEM_TO_USE. For simplicity you can run `./set_system.sh <system>` to set the system to your system (or even add it in your bashrc). Only the solver and configuration of the selected system are compiled, i.e., you do not need to run all the solvers to be able to compile.