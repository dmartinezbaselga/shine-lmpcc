/**
 * @file reference_velocity.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief
 * @version 0.1
 * @date 2022-09-23
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __REFERENCE_VELOCITY_H__
#define __REFERENCE_VELOCITY_H__

#include <lmpcc/types.h>

/**
 * @todo Possibly merge with homotopy guidance
 *
 */
class ReferenceVelocityModule : public ControllerModule
{
public:
    /**
     * @brief Construct a new Controller Module object. Note that controller module initialization happens in the solver class itself based on the python code.
     */
    ReferenceVelocityModule(ros::NodeHandle &nh, predictive_configuration *config, VehicleRegion *vehicle)
        : ControllerModule(nh, config, vehicle)
    {
        type_ = ModuleType::OBJECTIVE;
    };

public:
    /** ====== MANDATORY FUNCTIONS (purely virtual) ==========*/

    /**
     * @brief Update the module (any computations that need to happen before setting solver parameters)
     *
     * @param solver_interface
     */
    void Update(SolverInterface *solver_interface, RealTimeData &data) override
    {
    }

    /**
     * @brief Insert computed parameters for the solver
     *
     * @param solver_interface
     */
    void SetParameters(SolverInterface *solver_interface, const RealTimeData &data, int N_iter, int &param_idx) override
    {
        // Insert the reference velocity data from the real-time data
        // @note: Dynamic based on data
        // N_iter = std::max(0, std::min(N_iter - 1, (int)data.reference_velocity_.size() - 1)); // Because the range is 0 - N+2 with 0 and N+2 irrelevant
        // std::cout << "N_iter: " << N_iter << ": " << data.reference_velocity_[N_iter] << std::endl;
        // solver_interface->setParameter(N_iter, param_idx, data.reference_velocity_[N_iter]);

        solver_interface->setParameter(N_iter, param_idx, config_->reference_velocity_);
    }

    void ReconfigureCallback(SolverInterface *solver_interface, lmpcc::PredictiveControllerConfig &config, uint32_t level)
    {
        if (first_reconfigure_callback_)
        {
            config.velocity_reference = config_->reference_velocity_;
            first_reconfigure_callback_ = false;
        }
        else
            config_->reference_velocity_ = config.velocity_reference;
    }

    /**
     * @brief Visualize the computations in this module
     *
     */
    virtual void Visualize() override
    {
    }

private:
    bool first_reconfigure_callback_ = true;
    /* ======================================================== */

    // /** ====== OPTIONAL FUNCTIONS ==========*/

    // /**
    //  * @brief Check if this module is ready for control
    //  *
    //  * @return true If this module can execute the necessary computations with the current data
    //  * @return false Otherwise
    //  */
    // virtual bool ReadyForControl(SolverInterface *solver_interface, const RealTimeData &data) { return true; }; // Default: true

    // /**
    //  * @brief Check if the objective of this module was reached
    //  *
    //  * @param solver_interface
    //  * @param data
    //  * @return true If the objective was reached
    //  */
    // virtual bool ObjectiveReached(SolverInterface *solver_interface, const RealTimeData &data) { return true; }; // Default: true

    // /**
    //  * @brief Function used to update any class members when new data is received
    //  *
    //  * @param solver_interface The solver data. Necessary because the state is defined in this class
    //  * @param data All real-time data
    //  * @param data_name The name of the data that was updated (to decide if anything needs to be updated)
    //  */
    // virtual void OnDataReceived(SolverInterface *solver_interface, RealTimeData &data, std::string &&data_name){};

    // /**
    //  * @brief Reset any members if necessary
    //  *
    //  * @param solver_interface
    //  */
    // virtual void OnReset(SolverInterface *solver_interface){};

    // /**
    //  * @brief Override to define a custom optimization loop. Note that there can only be ONE customized optimization.
    //  *
    //  * @return int exit_code of the solver, return any exit_code other than "EXIT_CODE_NOT_OPTIMIZED_YET" to define this as a custom optimization
    //  */
    // virtual int Optimize(SolverInterface *solver_interface) { return EXIT_CODE_NOT_OPTIMIZED_YET; }; // Default: no custom optimization

    // /**
    //  * @brief Read parameters from the dynamic reconfiguration
    //  *
    //  * @param config The parameters from the configuration
    //  */
    // virtual void ReconfigureCallback(lmpcc::PredictiveControllerConfig &config, uint32_t level){};

    // /**
    //  * @brief Export runtime data
    //  *
    //  * @param data_saver the data_saver object to add the data to
    //  */
    // virtual void ExportData(RosTools::DataSaver &data_saver){};
};
#endif // __REFERENCE_VELOCITY_H__