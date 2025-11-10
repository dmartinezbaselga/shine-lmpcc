#include "modules_objectives/mpc_base.h"

#include <generated/weight_loader.h>
#include <lmpcc_solver/SolverInclude.h>

MPCBaseModule::MPCBaseModule(ros::NodeHandle &nh, predictive_configuration *config, VehicleRegion *vehicle)
    : ControllerModule(nh, config, vehicle)
{
  type_ = ModuleType::OBJECTIVE;
}

void MPCBaseModule::Update(SolverInterface *solver_interface, RealTimeData &data)
{
}

void MPCBaseModule::SetParameters(SolverInterface *solver_interface, const RealTimeData &data, int N_iter,
                                  int &param_idx)
{
  if (N_iter == 0)
    LMPCC_INFO("MPCBaseModule::SetParameters()");

  solver_interface->setWeightParameters(N_iter, param_idx);
}

void MPCBaseModule::ReconfigureCallback(SolverInterface *solver_interface, lmpcc::PredictiveControllerConfig &config,
                                        uint32_t level, bool first_callback)
{
  WeightLoader weight_loader;
  weight_loader.LoadWeightsFromConfig(solver_interface, config);

  if (first_callback)
  {
    weight_loader.LoadWeightsFromYaml(solver_interface, config);
  }
}