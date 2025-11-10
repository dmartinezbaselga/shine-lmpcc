Controller (C++ & ROS)
===================================

.. figure:: ../images/implementation_diagram.png
   :width: 500px

Architecture
+++++++++++++++++++++++++++++++++++
The main file of the C++ controller is ``lmpcc_controller.cpp``. Any preset parameters are passed from ``yaml`` files (see the ``config`` folder) and are read by ``lmpcc_configuration.cpp``.

The interaction of the controller with the environment (real or simulated) is abstracted through the ``interface``. The ``interface`` receives data from the environment and communicates it to the controller. The output of the controller, based on this data, is again passed through the ``interface`` to actuate the system. An interface, extending the base class ``interface.h`` should be created for each environment. See ``include/interfaces`` for examples.

The controller itself solves an MPC problem based on the data provided by the interface and the solver as defined in the Python code. To customize the behavior of the controller, modules can be defined to update objective or inequality terms. Each of these modules

* Updates their data and computes updated values (for the objective or inequalities)

* Inserts parameters to the solver

* Visualizes the computations if necessary

Through this structure a large variety of MPC formulations can be implemented.


Main Controller
+++++++++++++++++++++++++++++++++++

The main MPC implementation is based on:

**B. Brito, B. Floor, L. Ferranti, J. Alonso-Mora** *Model Predictive Contouring Control for Collision Avoidance in Unstructured Dynamic Environments* IEEE RA-L June 2019. Available: https://www.autonomousrobots.nl/docs/19-brito-ral.pdf.

MPCC
-----------------------------------
.. doxygenclass:: MPCC
	:project: LMPCC
	:members:
	:private-members:

Real-Time Data Types
+++++++++++++++++++++++++++++++++++

Dynamic Obstacles
-----------------------------------
.. doxygenclass:: DynamicObstacle
	:project: LMPCC
	:members:

Collision Discs
-----------------------------------
.. doxygenstruct:: Disc
	:project: LMPCC
	:members:

.. doxygenclass:: VehicleRegion
	:project: LMPCC
	:members:

.. _control_modules_obj:

Control Modules (Objectives)
+++++++++++++++++++++++++++++++++++

Reference Path
-----------------------------------
The reference path is currently still part of this package. For 2D we will likely switch to using a separate ``roadmap`` package, currently available at https://github.com/oscardegroot/roadmap. For documentation, see https://lmpcc-roadmap.readthedocs.io/en/latest/.

The ``roadmap`` package will replace the functionality in this class.

.. doxygenclass:: ReferencePath
	:project: LMPCC
	:members:

Homotopy Guidance
-----------------------------------
Creates dynamic topology distinct guidance paths for the MPC to follow. This method was published in:

**O. de Groot, L. Ferranti, D. Gavrila, J. Alonso-Mora** *Globally Guided Trajectory Planning in Dynamic Environments* Submitted to ICRA`23.

.. doxygenclass:: HomotopyGuidance
	:project: LMPCC
	:members:


.. _control_modules_ineq:

Control Modules (Inequalities)
+++++++++++++++++++++++++++++++++++

EllipsoidalConstraints
-----------------------------------
Avoids dynamic obstacles modeled as ellipsoids. Can also be used to conservatively satisfy collision avoidance chance-constraints by configuring the level set of the Gaussian at the desired risk as keep-out ellipsoid. This method was featured in:

**B. Brito, B. Floor, L. Ferranti, J. Alonso-Mora** *Model Predictive Contouring Control for Collision Avoidance in Unstructured Dynamic Environments* IEEE RA-L June 2019. Available: https://www.autonomousrobots.nl/docs/19-brito-ral.pdf.

And was used with uncertainty in:

**L.Ferranti, B. Brito and E.Pool, et al.** *SafeVRU: A Research Platform for the Interaction of Self-Driving Vehicles with Vulnerable Road Users* IEEE IV Symposium 2019. Available: https://www.autonomousrobots.nl/docs/19-ferranti-iv.pdf

A dynamically linearized version of the constraints is available in `LinearizedEllipsoidalConstraints`.

.. doxygenclass:: EllipsoidalConstraints
	:project: LMPCC
	:members:
	:private-members:

GaussianConstraints
-----------------------------------
Avoids dynamic obstacles subject to Gaussian motion uncertainty by formulating a deterministic constraint on the 1-D Cummulative Density Function (CDF). This work was published in:

**H. Zhu and J. Alonso-Mora** *Chance-Constrained Collision Avoidance for MAVs in Dynamic Environments* IEEE RA-L June 2019. Available: https://www.autonomousrobots.nl/docs/19-zhu-RAL.pdf.

.. doxygenclass:: GaussianConstraints
	:project: LMPCC
	:members:
	:private-members:


Scenario-Based MPC (ScenarioConstraints)
------------------------------------------
The ``ScenarioConstraints`` class is used to track the support of scenario constraints over multiple discs that define the robot's collision region. Two scenario-based methods are supported, details below.

.. doxygenclass:: ScenarioConstraints
	:project: LMPCC
	:members:
	:private-members:

The ScenarioConstraints class relies on the following other classes for computing the constraints.

Scenario-MPCC (S-MPCC)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Scenario-based inequalities for dynamic obstacle avoidance under non-Gaussian uncertainty as published in:

**O. de Groot, B. Brito, L. Ferranti, D. Gavrila, J. Alonso-Mora** *Scenario-Based Trajectory Optimization in Uncertain Dynamic Environments*, IEEE RA-L April 2021. Available: https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=9410362.

Configured by setting ros parameter `scenarios/use_trajectory_sampling` to false.

.. doxygenclass:: SMPCC
	:project: LMPCC
	:members:
	:private-members:

Safe Horizon MPC (SH-MPC)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

WIP

Configured by setting ros parameter `scenarios/use_trajectory_sampling` to true.

.. doxygenclass:: SafeHorizon
	:project: LMPCC
	:members:
	:private-members:

Other scenario related classes are discussed below.

Polygon Construction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. doxygenclass:: PolygonSearch
	:project: LMPCC
	:members:

Sampling
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. doxygenclass:: GaussianSampler
	:project: LMPCC
	:members:

Safety Certifier
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. doxygenclass:: SafetyCertifier
	:project: LMPCC
	:members:

.. Interfaces
.. +++++++++++++++++++++++++++++++++++

.. Interface (Base Class)
.. -----------------------------------
.. .. doxygenclass:: Interface
.. 	:project: LMPCC
.. 	:members:
.. 	:protected-members:

.. Specific Interfaces
.. -----------------------------------
.. Carla
.. ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. .. doxygenclass:: CarlaInterface
.. 	:project: LMPCC
.. 	:members:
.. 	:private-members:

.. Jackal
.. ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. .. doxygenclass:: JackalSimulatorInterface
.. 	:project: LMPCC
.. 	:members:	
.. 	:private-members:

.. Prius
.. ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. .. doxygenclass:: PriusInterface
.. 	:project: LMPCC
.. 	:members:	
.. 	:private-members:

.. Hovergames (Gazebo)
.. ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. .. doxygenclass:: HovergamesGazeboInterface
.. 	:project: LMPCC
.. 	:members:	
.. 	:private-members: