Overview
===================================

.. figure:: ../images/implementation_diagram.png
   :width: 500px

Architecture
+++++++++++++++++++++++++++++++++++
The structure of this package is divided in two sets of code:

* Python Side
	Generates an optimization solver using Forces Pro with desired dynamics, objective function and constraints.

* C++ Side
	Implements the real-time controller. The generated solver is wrapped with an interface to make it easy to use.


Control Modules
++++++++++++++++++++++++++++++++++
Since July-2022, we are working with 'Control Modules'. Each control module can define objectives and constraints and links to a C++ class where the user can implement the real-time behavior (e.g., inserting the positions of obstacles, the current reference path, etc.).

You can define the control modules to use in the Python solver code, which automatically prepares the C++ controller to run these modules in real-time. By stacking modules, functionality can be easily swapped or added to the MPC. For example, if we add a ContouringModule and a EllipsoidalConstraintModule, then the controller will follow a reference path and construct ellipsoidal constraints around obstacles.

To make a new control module, add a class in `control_modules.py` extending `Module`. In C++, create a class extending from `ControllerModule` (see the documentation in the base class in `types.h` for an overview of available functionality). 

Available modules are documented in :ref:`control_modules_obj` and :ref:`control_modules_ineq`.

System Interfaces
++++++++++++++++++++++++++++++++++
On the C++ side, all real-time information is provided through an interface that is specific to the application (e.g., Jackal, CARLA, Drones, etc.). This makes it possible to deploy the same controller for different systems. See :ref:`implemented-systems`.