/**
 * @file mpc_base.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief A base module for updating weights of an MPC controller. Weights are configured in the python solver
 * generation.
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __MPC_BASE_H__
#define __MPC_BASE_H__

// #include <ros_tools/helpers.h>
#include <lmpcc/types.h>
// #include <generated/weight_loader.h>

class predictive_configuration;
class VehicleRegion;
class SolverInterface;

class MPCBaseModule : public ControllerModule
{
public:
  /**
   * @brief Construct a new Controller Module object. Note that controller module initialization happens in the solver
   * class itself based on the python code.
   */
  MPCBaseModule(ros::NodeHandle &nh, predictive_configuration *config, VehicleRegion *vehicle);

public:
  /** ====== MANDATORY FUNCTIONS (purely virtual) ==========*/

  /**
   * @brief Update the module (any computations that need to happen before setting solver parameters)
   *
   * @param solver_interface
   */
  virtual void Update(SolverInterface *solver_interface, RealTimeData &data) override;

  /**
   * @brief Insert computed parameters for the solver
   *
   * @param solver_interface
   */
  virtual void SetParameters(SolverInterface *solver_interface, const RealTimeData &data, int N_iter,
                             int &param_idx) override;

  /**
   * @brief Visualize the computations in this module
   *
   */
  virtual void Visualize(){};
  /* ======================================================== */

  /** ====== OPTIONAL FUNCTIONS ==========*/

  /**
   * @brief Check if this module is ready for control
   *
   * @return true If this module can execute the necessary computations with the current data
   * @return false Otherwise
   */
  // virtual bool ReadyForControl(SolverInterface *solver_interface, const RealTimeData &data) { return true; }; //
  // Default: true

  /**
   * @brief Check if the objective of this module was reached
   *
   * @param solver_interface
   * @param data
   * @return true If the objective was reached
   */
  // virtual bool ObjectiveReached(SolverInterface *solver_interface, const RealTimeData &data) { return true; }; //
  // Default: true

  /**
   * @brief Function used to update any class members when new data is received
   *
   * @param solver_interface The solver data. Necessary because the state is defined in this class
   * @param data All real-time data
   * @param data_name The name of the data that was updated (to decide if anything needs to be updated)
   */
  // virtual void OnDataReceived(SolverInterface *solver_interface, RealTimeData &data, std::string &&data_name){};

  /**
   * @brief Reset any members if necessary
   *
   * @param solver_interface
   */
  // virtual void OnReset(SolverInterface *solver_interface){};

  /**
   * @brief Override to define a custom optimization loop. Note that there can only be ONE customized optimization.
   *
   * @return int exit_code of the solver, return any exit_code other than "EXIT_CODE_NOT_OPTIMIZED_YET" to define this
   * as a custom optimization
   */
  // virtual int Optimize(SolverInterface *solver_interface) { return EXIT_CODE_NOT_OPTIMIZED_YET; }; // Default: no
  // custom optimization

  /**
   * @brief Read parameters from the dynamic reconfiguration
   *
   * @param config The parameters from the configuration
   */
  virtual void ReconfigureCallback(SolverInterface *solver_interface, lmpcc::PredictiveControllerConfig &config,
                                   uint32_t level, bool first_callback) override;

  /**
   * @brief Export runtime data
   *
   * @param data_saver the data_saver object to add the data to
   */
  // virtual void ExportData(RosTools::DataSaver &data_saver){};

private:
};

#endif // __MPC_BASE_H__