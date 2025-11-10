import datetime
import os

import control_modules

generated_h_path = os.path.dirname(os.path.abspath(__file__)) + "/../include/lmpcc_solver/"
generated_cpp_path = os.path.dirname(os.path.abspath(__file__)) + "/../src/"
generated_external_path = os.path.dirname(os.path.abspath(__file__)) + "/../../lmpcc/include/generated/"

if not os.path.exists(generated_cpp_path):
    os.makedirs(generated_cpp_path)
if not os.path.exists(generated_h_path):
    os.makedirs(generated_h_path)
if not os.path.exists(generated_external_path):
    os.makedirs(generated_external_path)


def tabs(tab_level):
    result = ""
    for i in range(tab_level):
        result += "\t"

    return result


def open_function(h_file, cpp_file, function_header, optional_header_with_defaults=None, class_name="SolverInterface", has_type=True):

    if optional_header_with_defaults is None:
        optional_header_with_defaults = function_header

    # In the cpp file write the function declaration with function opener
    if has_type:
        split_header = function_header.split()
        idx = (len(split_header) > 1)
        split_header[idx] = class_name + "::" + split_header[idx]
        function_header = ' '.join(split_header)
    else:
        function_header = class_name + "::" + function_header # For the constructor

    cpp_file.write(tabs(1) + function_header)
    cpp_file.write("\n" + tabs(1) + "{\n")

    # In the h file, add the declaration only
    h_file.write(tabs(1) + optional_header_with_defaults + ";\n")


def close_function(file):
    file.write(tabs(1) + "}\n\n")


def val_zero(val):
    if val < 10:
        return "0" + str(val)
    else:
        return str(val)


def write_bool(file, tab_level, var_name, var):
    file.write(tabs(tab_level))
    if var:
        file.write(var_name + " = true;\n")
    else:
        file.write(var_name + " = false;\n")

def write_interface_configuration(settings, model):
    file_path_config_header = generated_h_path + "InterfaceDefinition.h"
    config_header = open(file_path_config_header, "w")
    config_header.write("#ifndef INTERFACE_CONFIG_H\n"
                        "#define INTERFACE_CONFIG_H\n\n")

    # TO PREVENT ERRORS: we will assume that there is an interface with the same name if none is specified
    if (not hasattr(settings, 'interfaces') or len(settings.interfaces) == 0):
        print('No interfaces specified in settings.interfaces. Assuming an interface with name: {}!'.format(
            model.system.name))
        settings.interfaces = [model.system.name]

    for idx, interface in enumerate(settings.interfaces):
        # Keep if statement if there are more than one interface defined
        if len(settings.interfaces) > 1:
            config_header.write("#ifdef SYSTEM_" + interface + "\n")

        # Otherwise we only have one define, so we do not need the "if" statement
        config_header.write("#pragma message \"Configuration: " + interface + " (See SYSTEM_OF_USE in CMakeLists!)\"\n"
                                                                              "\t#include <interfaces/" + interface.lower() + "_interface.h>\n"
                                                                                                                              "\t#define INTERFACE_CLASS " + interface + "Interface\n")

        if len(settings.interfaces) > 1:
            config_header.write("#endif\n")

    config_header.write("#endif\n")

def write_solver_configuration(settings, model):
    file_path_config_header = generated_h_path + "SolverDefinition.h"
    config_header = open(file_path_config_header, "w")
    config_header.write("#ifndef SOLVER_CONFIG_H\n"
                         "#define SOLVER_CONFIG_H\n\n")

    config_header.write("#define SOLVER SolverInterface\n"
                        "#define SOLVER_CAST(x) ((SolverInterface*)(&(*x)))\n")

    config_header.write("#endif\n")

    config_header.close()


def write_solver_include(settings, model):
    file_path_config_header = generated_h_path + "SolverInclude.h"
    config_header = open(file_path_config_header, "w")
    config_header.write("#ifndef SOLVER_INCLUDE_H\n"
                         "#define SOLVER_INCLUDE_H\n\n")

    config_header.write("#include <" + model.system.name + "Solver.h>\n")

    config_header.write("#endif\n")

    config_header.close()

# Useful for triggering DVC updates after solver generation
def write_configuration_flag(settings):

    file_path_configuration = generated_external_path + "Configuration" + settings.configuration
    configuration_flag_file = open(file_path_configuration, "w")
    date_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    configuration_flag_file.write("Configuration: " + settings.configuration + "\n")
    configuration_flag_file.write("hash: " + str(hash(settings)) + "\n") # Hash the settings (same when the settings do not change!)
    # configuration_flag_file.write("Generated: " + str(date_time))
    configuration_flag_file.close()

def write_current_configuration(settings):
    """
    Writes the current configuration to a file so that it can be checked by other code
    """
    file_path_configuration = generated_external_path + "CurrentConfiguration"
    configuration_flag_file = open(file_path_configuration, "w")

    configuration_flag_file.write(settings.configuration + "\n")
    configuration_flag_file.close()

def write_module_file(settings, model):
    module_path = generated_external_path
    os.makedirs(module_path, exist_ok=True)
    module_file = open(module_path + "modules.h", "w")
    submodule_file = open(module_path + "submodules.h", "w")

    submodule_file.write("#ifndef __SUBMODULES__\n")
    submodule_file.write("#define __SUBMODULES__\n\n")

    # Import submodules
    for module in settings.modules.modules:
        if hasattr(module, 'submodules'):  # Add an import if defined
            for submodule in module.submodules:
                # Quickly, check if there are not other modules with the same name
                module_already_included = False
                for module in settings.modules.modules:
                    if type(module) == type(submodule):
                        module_already_included = True
                        break
                if module_already_included:
                    break

                submodule_file.write("#include <" + submodule.import_name + ">\n")
    for module in settings.modules.modules:
        module.write_to_solver_interface(submodule_file)
    if not settings.modules.contains_module(control_modules.HomotopyGuidanceConstraintModule):
        submodule_file.write("#include <modules_constraints/ellipsoidal_constraints.h>\n")
        submodule_file.write("#define GUIDANCE_CONSTRAINTS_TYPE EllipsoidalConstraints\n")

    submodule_file.write("#endif")

    module_file.write("#ifndef __MODULES__\n")
    module_file.write("#define __MODULES__\n\n")

    # Import modules
    for module in settings.modules.modules:
        if hasattr(module, 'import_name'):  # Add an import if defined
            module_file.write("#include <" + module.import_name + ">\n")

    # Initialization of the modules
    module_file.write("inline void InitializeModules(std::vector<std::unique_ptr<ControllerModule>>& controller_modules,\n\t ros::NodeHandle &nh, predictive_configuration *config, VehicleRegion *vehicle){\n")

    for module in settings.modules.modules:
        if hasattr(module, 'import_name'): # Some modules may not have C++ components (if they depend on other modules)
            module_file.write("\t\tcontroller_modules.emplace_back(nullptr);\n"
                                "\t\tcontroller_modules.back().reset(new " + module.module_name + "(nh, config, vehicle));\n")

    module_file.write("}\n")
    module_file.write("#endif")

    weight_file = open(generated_external_path + "weight_loader.h", "w")

    weight_file.write("#ifndef __WEIGHT_LOADER__\n")
    weight_file.write("#define __WEIGHT_LOADER__\n\n")
    weight_file.write("#include <lmpcc/lmpcc_configuration.h>\n")
    weight_file.write("#include <lmpcc/PredictiveControllerConfig.h>\n")
    weight_file.write("#include <lmpcc_solver/SolverInclude.h>\n")
    weight_file.write("#include <ros_tools/base_configuration.h>\n\n")

    weight_file.write("class WeightLoader : public RosTools::BaseConfiguration\n{\npublic:\n")
    weight_file.write("\tvoid LoadWeightsFromConfig(SolverInterface* solver, lmpcc::PredictiveControllerConfig &config){\n")
    for weight in settings.weight_list:
        weight_file.write("\t\tsolver->weights_." + weight + "_ = config." + weight + ";\n")
    weight_file.write("\t}\n")

    weight_file.write("\tvoid LoadWeightsFromYaml(SolverInterface* solver, lmpcc::PredictiveControllerConfig& config){\n")
    weight_file.write("\t\tros::NodeHandle nh;\n")
    for weight in settings.weight_list:
        weight_file.write("\t\tdouble " + weight + ";\n"
                        "\t\tretrieveParameter(nh, \"weights/" + weight + "\"," + weight + ", -1.);\n"
                        "\t\tif (" + weight + " != -1){\n"
                        "\t\t\tsolver->weights_." + weight + "_ = " + weight + ";\n"
                        "\t\t\tconfig." + weight + " = " + weight + ";\n\t\t}\n")
    weight_file.write("\t}\n")
    weight_file.write("};\n")
    weight_file.write("#endif")


def write_model_header(settings, model):

    if settings.follower_solver:
        # get the file path for rospy_tutorials
        generated_cpp_path = os.path.dirname(os.path.abspath(__file__)) + "/../../lmpcc_follower/include/lmpcc_follower/generated_cpp/"
    else:
        generated_cpp_path = os.path.dirname(os.path.abspath(__file__)) + "/../src/"
    
    if not os.path.exists(generated_cpp_path):
            os.makedirs(generated_cpp_path)

    write_interface_configuration(settings, model)
    write_solver_include(settings, model)
    write_configuration_flag(settings)
    write_current_configuration(settings)
    write_module_file(settings, model)

    file_path = generated_cpp_path + model.system.name +"Solver"
    print("Placing generated C++ code in: {}".format(file_path))
    header_file = open(generated_h_path + model.system.name + "Solver.h", "w")
    cpp_file = open(generated_cpp_path + model.system.name +"Solver.cpp", "w")

    cpp_file.write("#include \"lmpcc_solver/" + model.system.name + "Solver.h\"\n")

    header_file.write("/** This file was autogenerated by the lmpcc package at " + datetime.datetime.now().strftime("%I:%M%p on %B %d, %Y") + "*/\n")
    header_file.write(\
        "#ifndef __"+model.system.name+"MODEL_H__\n"
        "#define __"+model.system.name+"MODEL_H__\n\n"
        "#include <ros/ros.h>\n"
        "#include <vector>\n"
        "#include <memory>\n\n"
        "#include <lmpcc_solver/PropagateDynamics.h>\n"
        "#include <lmpcc_solver/PropagateDynamicsRequest.h>\n"
        "#include <lmpcc_solver/PropagateDynamicsResponse.h>\n"
        "#include <std_msgs/Float64MultiArray.h>\n"
        "#include <std_msgs/MultiArrayDimension.h>\n\n"
        "#include <" + model.system.name+"FORCESNLPsolver.h>\n"
        "#include <" + model.system.name+"FORCESNLPsolver_memory.h>\n\n"
        "#include \"lmpcc_tools/collision_region.h\"\n"
        "#include \"ros_tools/helpers.h\"\n\n")

    header_file.write(\
        "extern \"C\"\n"
        "{\n"
        "\textern solver_int32_default "+model.system.name+"FORCESNLPsolver_adtool2forces("+model.system.name+"FORCESNLPsolver_float *x,  /* primal vars                                         */\n"
                                                           "\t\t\t\t\t\t\t\t\t\t"+model.system.name+"FORCESNLPsolver_float *y,  /* eq. constraint multiplers                           */\n"
                                                           "\t\t\t\t\t\t\t\t\t\t"+model.system.name+"FORCESNLPsolver_float *l,  /* ineq. constraint multipliers                        */\n"
                                                           "\t\t\t\t\t\t\t\t\t\t"+model.system.name+"FORCESNLPsolver_float *p,  /* parameters                                          */\n"
                                                           "\t\t\t\t\t\t\t\t\t\t"+model.system.name+"FORCESNLPsolver_float *f,  /* objective function (scalar)                         */\n"
                                                           "\t\t\t\t\t\t\t\t\t\t"+model.system.name+"FORCESNLPsolver_float *nabla_f, /* gradient of objective function                      */\n"
                                                           "\t\t\t\t\t\t\t\t\t\t"+model.system.name+"FORCESNLPsolver_float *c,	   /* dynamics                                            */\n"
                                                           "\t\t\t\t\t\t\t\t\t\t"+model.system.name+"FORCESNLPsolver_float *nabla_c, /* Jacobian of the dynamics (column major)             */\n"
                                                           "\t\t\t\t\t\t\t\t\t\t"+model.system.name+"FORCESNLPsolver_float *h,	   /* inequality constraints                              */\n"
                                                           "\t\t\t\t\t\t\t\t\t\t"+model.system.name+"FORCESNLPsolver_float *nabla_h, /* Jacobian of inequality constraints (column major)   */\n"
                                                           "\t\t\t\t\t\t\t\t\t\t"+model.system.name+"FORCESNLPsolver_float *hess,	   /* Hessian (column major)                              */\n"
                                                           "\t\t\t\t\t\t\t\t\t\tsolver_int32_default stage,	   /* stage number (0 indexed)                            */\n"
                                                           "\t\t\t\t\t\t\t\t\t\tsolver_int32_default iteration, /* iteration number of solver                          */\n"
                                                           "\t\t\t\t\t\t\t\t\t\tsolver_int32_default threadID /* Id of caller thread 								   */);\n"
        "}\n\n"
    )
    cpp_file.write("extern \"C\"\n{\n"
                   "\t"+model.system.name+"FORCESNLPsolver_extfunc extfunc_eval_"+model.system.name.lower()+" = &"+model.system.name+"FORCESNLPsolver_adtool2forces;\n"
                    "}\n")


    header_file.write("class " + model.system.name + "Weights\n"
                    "{\n")

    header_file.write("public:\n")

    for weight in settings.weight_list:
        header_file.write("\tdouble " + weight + "_;")

        header_file.write("\n")

    # Setters / Getters
    for weight in settings.weight_list:
        header_file.write("\tvoid set_" + weight + "(double value){ "+ weight + "_ = value;};\n")

    for weight in settings.weight_list:
        header_file.write("\tdouble get_" + weight + "(){ return " + weight + "_;};\n")



    header_file.write("};\n\n")

    # The dynamic state structure
    header_file.write("\nclass "+model.system.name+"DynamicsState\n"
                      "{\n\n")

    header_file.write("public:\n")

    header_file.write("\t"+model.system.name+"DynamicsState(){};\n\n")

    # Set functions
    header_file.write("\t/** @brief Setter functions for variables in the model */\n")
    for idx, state in enumerate(model.states):
        if model.states_from_sensor[idx]:
            header_file.write("\tvoid set_"+state+"(double value) { "+state+"_ = value; };\n")

    header_file.write("\n\t// Getter functions for variables in the model\n")
    for idx, state in enumerate(model.states):
        if model.states_from_sensor[idx]:
            header_file.write("\tdouble& "+state+"() { return "+state+"_; };\n")

    header_file.write("\n\tvoid init()\n"
                      "\t{\n")

    for idx, state in enumerate(model.states):
        if model.states_from_sensor[idx]:
            header_file.write("\t\t" + state + "_ = 0.0;\n")

    header_file.write("\t}\n")

    open_function(header_file, cpp_file, "void print()", class_name=model.system.name+"DynamicsState")
    cpp_file.write("\t\tROS_WARN(\"========== State ==========\");\n")
    for idx, state in enumerate(model.states):
        if model.states_from_sensor[idx]:
            cpp_file.write("\t\tROS_INFO_STREAM(\"" + state + " \t= \" << " + state + "_);\n")
    cpp_file.write("\t\tROS_WARN(\"============================\");\n")
    close_function(cpp_file)

    header_file.write("\nprivate:\n")
    for idx, state in enumerate(model.states):
        if model.states_from_sensor[idx]:
            header_file.write("\tdouble "+state+"_;\n")

    header_file.write("};\n\n")

    # PLAN STRUCT
    header_file.write("\nstruct "+model.system.name+"Plan\n{\n"
                    "\tstd::vector<" + model.system.name + "DynamicsState> states_;\n"
                    "\t"+model.system.name+"Plan() {};\n"
                    "\t"+model.system.name+"Plan(int N)\n\t{\n"
                    "\t\tstates_.resize(N);\n"
                      "\t\tReset();\n"
                      "\t}\n\n")

    header_file.write("\tvoid Reset()\n\t{\n"
                    "\t\tfor(size_t k = 0; k < states_.size(); k++)\n\t\t{\n")

    for idx, state in enumerate(model.states):
        if model.states_from_sensor[idx]:
            header_file.write("\t\t\tstates_[k].set_"+state+"(0.);\n")

    header_file.write("\t\t}\n\t}\n\n")

    open_function(header_file, cpp_file, "void Print()", class_name=model.system.name+"Plan")
    cpp_file.write("\t\tstd::cout << \"=== Plan ===========================\\n\";\n"
                      "\t\tfor (size_t k = 0; k < states_.size(); k++)\n"
                      "\t\t\tstd::cout << \"[\" << k << \"]:\\t")
    for idx, state in enumerate(model.states):
        if model.states_from_sensor[idx]:
            cpp_file.write(state+ ": \" << states_[k]." + state + "() << \"\\t\\t\\t| ")

    cpp_file.write("\\n\";\n\n")
    cpp_file.write("\t\tstd::cout << \"====================================\\n\";\n")
    close_function(cpp_file)

    header_file.write("};\n\n")

    # THE MAIN CLASS
    current_class = "SolverInterface"
    header_file.write("class SolverInterface\n"
                      "{\n\n")

    header_file.write("protected:\n")
    header_file.write("\t"+model.system.name + "Plan plan_;\n")
    header_file.write("\t"+model.system.name + "Plan initial_plan_;\n\n")
    header_file.write("\t"+model.system.name +"DynamicsState state_;\n\n")
    header_file.write("\tchar *solver_memory_;\n")
    header_file.write("\t"+model.system.name + "FORCESNLPsolver_mem *solver_memory_handle_;\n\n")

    header_file.write("public:\n")
    header_file.write("\tint solver_id_;\n\n")

    header_file.write("\tstruct ControlInputs{\n")
    for name, var_name in model.control_inputs.items():
        header_file.write("\t\tdouble " + name + ";\n")
    header_file.write("\t};\n\n"
                      "\tControlInputs control_inputs_;\n")

    # header_file.write("\t"+model.system.name+"DynamicsInput control_;\n")
    header_file.write("\t"+model.system.name+"Weights weights_;\n\n")

    if not hasattr(settings, 'method_name_for_recording'):
        settings.method_name_for_recording = ''

    # @todo: Isolate module variables
    header_file.write(\
        "\t"+model.system.name+"FORCESNLPsolver_params forces_params_;\n" +
        "\t"+model.system.name+"FORCESNLPsolver_output forces_output_;\n" +
        "\t"+model.system.name+"FORCESNLPsolver_info forces_info_;\n\n" +
        "\t"+model.system.name+"FORCESNLPsolver_output last_solution_;\n"
        "\t"+model.system.name+"FORCESNLPsolver_info last_info_;\n"
        "\tbool has_safe_solution_;\n\n"
        "\tunsigned int FORCES_N;		 // Horizon length\n"
        "\tunsigned int FORCES_NBAR;	 // Horizon length necessary in Forces to get a length of FORCES_N in practice\n"
        "\tunsigned int FORCES_NU;		 // Number of control variables\n"
        "\tunsigned int FORCES_NX;		 // Differentiable variables\n"
        "\tunsigned int FORCES_TOTAL_V; // Total variable count\n"
        "\tunsigned int FORCES_NPAR;	 // Parameters per iteration\n"
        "\tdouble DT;\n\n"
        "\tstd::unique_ptr<VehicleRegionTemplate> area_;\n"
        "\tstd::vector<VehicleRegion> initial_vehicle_predictions_, optimized_vehicle_predictions_;\n\n"
        "\tint n_discs_, n_segments_;\n"
        "\tstd::string method_name_;\n"
        "\tbool use_sqp_solver;\n\n")

    open_function(header_file, cpp_file, "SolverInterface(int solver_id)",
                  optional_header_with_defaults="SolverInterface(int solver_id = 0)",
                  has_type=False)
    cpp_file.write("\t\tsolver_id_ = solver_id;\n"          
        "\t\tFORCES_N = " + str(settings.N) + "; // Horizon length\n"
                                "\t\tFORCES_NBAR = " + str(settings.N_bar) + "; // Horizon length necessary in forces for N\n"
                                "\t\tFORCES_NU = " + str(model.nu) + "; // Number of control variables\n"
                                "\t\tFORCES_NX = " + str(model.nx) + "; // Differentiable variables\n"
                                "\t\tFORCES_TOTAL_V = " + str(model.nvar) + "; // Total variable count\n"
                                "\t\tFORCES_NPAR = " + str(settings.npar) + "; // Parameters per iteration\n"
                                "\t\tDT = " + str(settings.integrator_stepsize) + "; // Step size of the intergrator\n"
                                "\n\t\t// Solver parameters\n"
                                "\t\tn_discs_ = " + str(settings.n_discs) + "; // Number of discs in robot collision region\n")
    cpp_file.write("\t\tarea_.reset(new VehicleRegionTemplate(" + str(settings.n_discs) + ", {")
    for i in range(settings.n_discs):
        cpp_file.write(str(float(model.system.area.offsets[i])))
        if i != settings.n_discs - 1:
            cpp_file.write(", ")
    cpp_file.write("}, " + str(model.system.area.radius) + "));\n")

    cpp_file.write("\t\tn_segments_ = " + str(settings.n_segments) + "; // Number of discs in robot collision region\n"
                                "\t\tmethod_name_ = \"" + settings.method_name_for_recording + "\";\n\n"
                                "\t\tsolver_memory_ = (char*)malloc(" + model.system.name + "FORCESNLPsolver_get_mem_size());\n"
	                            "\t\tsolver_memory_handle_ = " + model.system.name + "FORCESNLPsolver_external_mem(solver_memory_, solver_id, " + model.system.name + "FORCESNLPsolver_get_mem_size());\n\n"
                             )

    write_bool(cpp_file, 2, "use_sqp_solver", settings.use_sqp_solver)

    cpp_file.write("\t\tplan_ = "+model.system.name + "Plan(" + str(settings.N) + ");\n")
    cpp_file.write("\t\tinitial_plan_ = "+model.system.name + "Plan(" + str(settings.N) + ");\n")
    cpp_file.write("\t\tState().init();\n")
    cpp_file.write("\t\tresetSolver();\n")
    close_function(cpp_file)

    # header_file.write("\t// Module specific code\n")
    # for module in settings.modules.modules:
    #     module.write_to_solver_interface(header_file, cpp_file)

    open_function(header_file, cpp_file, "~SolverInterface()")
    cpp_file.write("\t\tfree(solver_memory_);\n")
    close_function(cpp_file)

    header_file.write("\n// Tools for copying solver data to other solvers that can run in parallel")
    header_file.write("\n\tchar *GetSolverMemory() { return solver_memory_; };\n")
    open_function(header_file, cpp_file, "void CopyInternalSolverData(SolverInterface *other)")
    cpp_file.write("memcpy(solver_memory_, other->GetSolverMemory(), " + model.system.name + "FORCESNLPsolver_get_mem_size());\n")
    close_function(cpp_file)

    open_function(header_file, cpp_file, "void CopySolverParameters(SolverInterface *other)")
    cpp_file.write("\t\tforces_params_ = other->forces_params_;\n"
                    "\t\tinitial_plan_ = other->initial_plan_;\n"
                    "\t\tplan_ = other->plan_;\n"
                    "\t\tinitial_vehicle_predictions_ = other->initial_vehicle_predictions_;\n"
                    "\t\toptimized_vehicle_predictions_ = other->optimized_vehicle_predictions_;\n")
    close_function(cpp_file)

    open_function(header_file, cpp_file, "void CopySolverOutput(SolverInterface *other)")
    cpp_file.write("forces_output_ = other->forces_output_;\n")
    cpp_file.write("forces_info_ = other->forces_info_;\n")
    close_function(cpp_file)

    # Inputs references
    header_file.write("\n\t/* Inputs */\n")
    for i in range(0, len(model.inputs)):
        header_file.write("\tdouble& " + model.inputs[i] + \
                          "(unsigned int k = 0) { return forces_params_.x0[k * FORCES_TOTAL_V + " + str(i) + "]; };\n")

    """
    header_file.write('\t/* States and Inputs for actuation */\n')
    for actuated_var in model.possible_inputs_to_vehicle:
        is_input = False
        is_state = False
        for idx, input in enumerate(model.inputs):
            if actuated_var == input:
                header_file.write("\tdouble " + input + \
                                  "() { return forces_output_.x01[" + str(idx) + "]; };\n")
                is_input = True
                break

        if is_input:
            continue

        for idx, state in enumerate(model.states):
            if actuated_var == state:
                header_file.write("\tdouble " + state + \
                                  "() { return forces_params_.x0[FORCES_TOTAL_V + " + str(model.nu + idx) + "]; };\n")
                is_state = True
                break

        if not (is_input or is_state):
            raise ValueError('Variable ' + actuated_var + ' is not an input or a state in the model, please check your model description')
    """

    header_file.write("\n\t/* States */ \n")
    for i in range(0, len(model.states)):
        header_file.write("\tdouble& " + model.states[i] + \
                          "(unsigned int k) { return forces_params_.x0[k * FORCES_TOTAL_V + " + str(model.nu + i) + "]; };\n")

    # Getters / Setters
    header_file.write("\n\t"+model.system.name+"DynamicsState& State(){return state_;};\n")
    header_file.write("\t"+model.system.name+"DynamicsState& Plan(int k){return plan_.states_[k];};\n")
    header_file.write("\t"+model.system.name+"DynamicsState& InitialPlan(int k){return initial_plan_.states_[k];};\n")
    header_file.write("\tvoid PrintPlan() { plan_.Print(); };\n\n")
    header_file.write("\tstd::vector<VehicleRegion> &InitialVehiclePrediction() { return initial_vehicle_predictions_; };\n\n")
    header_file.write("\tstd::vector<VehicleRegion> &OptimizedVehiclePredictions() { return optimized_vehicle_predictions_; };\n\n")

    # Load weights of the base MPC model
    if settings.modules.contains_module(control_modules.MPCBaseModule):
        open_function(header_file, cpp_file, "void setWeightParameters(int k, int& param_idx)")
        for idx, weight in enumerate(settings.weight_list):#settings.weights.weights):
            cpp_file.write("\t\tsetParameter(k, param_idx, weights_." + weight + "_);\n")
        close_function(cpp_file)

    # Reset solver function #
    open_function(header_file, cpp_file, "void resetSolver()")
    cpp_file.write("\t\tfor (long int i = 0; i < *(&forces_params_.all_parameters + 1) - forces_params_.all_parameters; i++)\n"
                        "\t\t\tforces_params_.all_parameters[i] = 0.0;\n\n"
                        "\t\tfor (long int i = 0; i < *(&forces_params_.xinit + 1) - forces_params_.xinit; i++)\n"
                        "\t\t\tforces_params_.xinit[i] = 0.0;\n\n"
                        "\t\tfor (size_t i = 0; i < FORCES_NBAR*FORCES_TOTAL_V; i++)\n"
                        "\t\t\tforces_params_.x0[i] = 0.0;\n")
    close_function(cpp_file)

    # Set Parameter function #
    header_file.write("\t/** @brief Set a solver parameter at index index of stage k to value */\n")
    open_function(header_file, cpp_file, "void setParameter(unsigned int k, int& index, double value)")
    cpp_file.write("\t\tforces_params_.all_parameters[k*FORCES_NPAR + index] = value;\n")
    cpp_file.write("\t\tindex++;\n") # Increase the index automatically
    close_function(cpp_file)

    # Set Parameter function (rvalue)#
    header_file.write("\t/** @brief Set a solver parameter at index index of stage k to value */\n")
    open_function(header_file, cpp_file, "void setParameterSpecific(unsigned int k, int index, double value)")
    cpp_file.write("\t\tforces_params_.all_parameters[k*FORCES_NPAR + index] = value;\n")
    close_function(cpp_file)

    header_file.write("\t/** @brief Set a solver parameter at index index of stage k to value */\n")
    open_function(header_file, cpp_file, "double getParameter(unsigned int k, unsigned int index)")
    cpp_file.write("\t\treturn forces_params_.all_parameters[k*FORCES_NPAR + index];\n")
    close_function(cpp_file)

    # Solve function #
    header_file.write("\t/** @brief Solve the optimization */\n")
    open_function(header_file, cpp_file, "int solve(int max_iterations, double tolerance)",
                                         optional_header_with_defaults="int solve(int max_iterations = 0, double tolerance = 1e-3)")
    # open_function(header_file, cpp_file, "int solve(int max_iterations = 0, double tolerance = 1e-3);")

    # header_file.write("\tint solve(int max_iterations = 0, double tolerance = 1e-3);\n")
    # open_function(header_file, cpp_file, "int solve(int max_iterations = 0, double tolerance = 1e-3)")

    cpp_file.write("\t\tif(EqualityConverged(tolerance)){ // If the eq. constraints are satisfied\n")
    cpp_file.write("\t\t\tlast_solution_ = forces_output_; // Save the last solution\n\n")
    cpp_file.write("\t\t\tlast_info_ = forces_info_; // Save the last information of the solver\n")
    cpp_file.write("\t\t\thas_safe_solution_ = true;\n\t\t}\n")
    # @Note: Use external memory always now
    # cpp_file.write("\t\t" + model.system.name + "FORCESNLPsolver_mem *mem;\n"
	# 	              "\t\tmem = " + model.system.name + "FORCESNLPsolver_internal_mem(0);\n\n")

    if settings.use_sqp_solver and (not settings.use_scenario_constraints):
        if settings.multi_solver:
            raise IOError("Multi solver is not supported with SQP yet in LMPCC")
        cpp_file.write("\t\tint exit_code = -1;\n"
                          "\t\tfor (int solve_iteration = 0; solve_iteration < max_iterations; solve_iteration++){\n"
                          "\t\t\t// We load the previous trajectory in the first iterations and reuse the previous iteration in later iterations\n"
                          "\t\t\tif (solve_iteration == 0)                     // | feasible\n"
                          "\t\t\t\tsetReinitialize(true); // Initialize with the shifted trajectory\n"
                          "\t\t\telse if (solve_iteration >= 1)\n"
                          "\t\t\t\tsetReinitialize(false);\n"
                          "\t\t\t// Solve an iteration of the SQP problem and load the output\n"
                          "\t\t\texit_code = " + model.system.name + "FORCESNLPsolver_solve(&forces_params_, &forces_output_, "
                                                              "&forces_info_, mem, stdout, extfunc_eval_" + model.system.name.lower()+");\n"
                          "\t\t}\n"
                          "\t\treturn exit_code;")
    else:
        cpp_file.write("\t\tint exit_code = "+model.system.name+"FORCESNLPsolver_solve(&forces_params_, &forces_output_, &forces_info_, solver_memory_handle_, stdout, extfunc_eval_"+model.system.name.lower()+");\n")
        cpp_file.write("\t\treturn exit_code;\n")
    close_function(cpp_file)

    header_file.write("\t/** @brief Print Solver Info for this Iteration */\n")
    open_function(header_file, cpp_file, "void printSolveInfo()")
    if settings.use_sqp_solver:
        if settings.print_init_bfgs:
            cpp_file.write("\t\tstd::cout << \"BFGS: \\n\";\n"
                            "\t\tfor (size_t i = 0; i < FORCES_TOTAL_V; i++)\n"
                            "\t\t\tstd::cout << forces_output_.BFGSdiagonal01[i] << \", \";\n"
                            "\t\tstd::cout << \"\\n\";\n\n"
                            "\t\tfor (size_t i = 0; i < FORCES_TOTAL_V; i++)\n"
                            "\t\t\tstd::cout << forces_output_.BFGSdiagonal09[i] << \", \";\n"
                            "\t\tstd::cout << \"\\n\";\n\n"
                            "\t\tfor (size_t i = 0; i < FORCES_TOTAL_V; i++)\n"
                            "\t\t\tstd::cout << forces_output_.BFGSdiagonal19[i] << \", \";\n"
                            "\t\tstd::cout << \"\\n\";\n\n")
    elif settings.multi_solver:
        cpp_file.write("\t\tstd::cout << forces_info_.it << std::endl;\n")
    close_function(cpp_file)

    # Function for safety under equality convergence
    open_function(header_file, cpp_file, "double GetPreviousEqualityTolerance()")
    cpp_file.write("\t\treturn last_info_.res_eq;\n")
    close_function(cpp_file)

    open_function(header_file, cpp_file, "bool EqualityConverged(double tolerance)")
    cpp_file.write("\t\treturn forces_info_.res_eq < tolerance;\n")
    close_function(cpp_file)

    open_function(header_file, cpp_file, "bool HasSafeSolution()")
    cpp_file.write("\t\treturn has_safe_solution_;\n")
    close_function(cpp_file)

    open_function(header_file, cpp_file, "void UseLastSolution()")
    cpp_file.write("\t\tforces_output_ = last_solution_;\n")
    close_function(cpp_file)

    open_function(header_file, cpp_file, "double Cost() const")
    cpp_file.write("\t\treturn forces_info_.pobj;\n")
    close_function(cpp_file)

    header_file.write("\t/** @brief Reinitialize the solver (SQP) */\n")
    open_function(header_file, cpp_file, "void setReinitialize(bool value)")
    if settings.use_sqp_solver:
        cpp_file.write("\t\tforces_params_.reinitialize = value;\n"
                          "\t\thas_safe_solution_ = false;\n"
                            "\t\tif(value)\n\t\t\tloadBFGS();\n")
    close_function(cpp_file)

    # Insert Predicted Trajectory function #
    header_file.write("\t/** @brief Important Note: output.x01 holds the initial state! */\n")
    open_function(header_file, cpp_file, "void insertPredictedTrajectory()")
    cpp_file.write("\t\tfor (unsigned int i = 0; i < FORCES_TOTAL_V; i++){\n")

    # This needs to be N_bar in order to save all computed values
    for k in range(0, settings.N_bar):
        if(k >= 9):
            cpp_file.write("\t\t\tforces_params_.x0[i + " + str(k) + " * FORCES_TOTAL_V] = forces_output_.x" + str(k + 1) + "[i];\n")
        else:
            cpp_file.write("\t\t\tforces_params_.x0[i + " + str(k) + " * FORCES_TOTAL_V] = forces_output_.x0" + str(k + 1) + "[i];\n")

    cpp_file.write("\t\t}\n")
    close_function(cpp_file)

    # Set initial spline function (value based), spline cannot be updated from the state usually #
    # Check what index the spline is
    # spline_idx = -1
    # for idx, state in enumerate(model.states):
    #     if state == "spline":
    #         spline_idx = idx

    spline_idx = model.get_state_idx('spline')
    if spline_idx != -1:
        header_file.write("\n\t/** @brief Set xinit at index to value */\n")
        open_function(header_file, cpp_file, "void setInitialSpline(double value)")
        cpp_file.write("\t\tforces_params_.xinit[" + str(spline_idx) + "] = value;\n")
        close_function(cpp_file)

    # Set initial plan to previously planned plan #
    header_file.write("\n\t// Set initial plan to previously computed plan\n")
    open_function(header_file, cpp_file, "void setInitialPlanToPreviousPlan()")
    cpp_file.write("\t\tinitial_plan_ = plan_; // at initialization we set the last plan as the plan of the "
                      "previous optimization\n")
    close_function(cpp_file)

    header_file.write("\n\t// Load predictions of the vehicle based on this initial plan\n")
    open_function(header_file, cpp_file, "void LoadInitialVehiclePredictions()")
    cpp_file.write("\t\tinitial_vehicle_predictions_.clear();\n"
                      "\t\tfor(size_t k = 0; k < FORCES_N; k++){\n"
                      "\t\t\tinitial_vehicle_predictions_.emplace_back(Eigen::Vector2d(InitialPlan(k).x(), InitialPlan(k).y()),\n"
                        "\t\t\tInitialPlan(k).psi(),\n"
                        "\t\t\t*area_);\n"
                      "\t\t}\n")
    close_function(cpp_file)

    header_file.write("\n\t// Load predictions of the vehicle based on the optimized plan\n")
    open_function(header_file, cpp_file, "void LoadOptimizedVehiclePredictions()")
    cpp_file.write("\t\toptimized_vehicle_predictions_.clear();\n"
                      "\t\tfor(size_t k = 0; k < FORCES_N; k++){\n"
                      "\t\t\toptimized_vehicle_predictions_.emplace_back(Eigen::Vector2d(Plan(k).x(), Plan(k).y()),\n"
                        "\t\t\tPlan(k).psi(),\n"
                        "\t\t\t*area_);\n"
                      "\t\t}\n")
    close_function(cpp_file)

    open_function(header_file, cpp_file, "void LoadVehiclePredictionsToInitialPlan(const std::vector<VehicleRegion> &vehicle_prediction)")
    cpp_file.write("\t\t	for (size_t k = 0; k < FORCES_N; k++){\n"
                    "\t\t\t		initial_plan_.states_[k].set_x(vehicle_prediction[k].pos_(0));\n"
                    "\t\t\t		initial_plan_.states_[k].set_y(vehicle_prediction[k].pos_(1));\n"
                    "\t\t\t		initial_plan_.states_[k].set_psi(vehicle_prediction[k].orientation_);\n"
                    "\t\t\t	}\n")
    close_function(cpp_file)

    header_file.write("\t/** @brief Use the FORCES Pro Dynamics to compute the integrated trajectory based on the dynamics. */\n")
    open_function(header_file, cpp_file, model.system.name+"DynamicsState PropagateInitialState("+model.system.name+"DynamicsState &initial_state, int n_steps)",
                  optional_header_with_defaults=model.system.name+"DynamicsState PropagateInitialState("+model.system.name+"DynamicsState &initial_state, int n_steps=1)")
    cpp_file.write(
        "\tlmpcc_solver::PropagateDynamicsRequest req;\n\n"
        "\tstd_msgs::Float64MultiArray input_msg;\n"
        "\tfor (size_t k = 0; k < FORCES_N; k++){\n")
    for input in model.inputs:
        cpp_file.write("\t\tinput_msg.data.push_back(" + input + "(k));\n")

    cpp_file.write(
        "\t}\n"
        "\tstd_msgs::MultiArrayDimension dims;\n"
        "\tdims.size = FORCES_NU;\n"
        "\tinput_msg.layout.dim.push_back(dims);\n"
        "\tdims.size = FORCES_N;\n"
        "\tinput_msg.layout.dim.push_back(dims);\n"
        "\treq.inputs = input_msg;\n\n"
        "\tstd_msgs::Float64MultiArray x_init_msg;\n")

    for idx, state in enumerate(model.states):
        if model.states_from_sensor[idx]:
            cpp_file.write("\tx_init_msg.data.push_back(initial_state." + state + "());\n")
        else:
            cpp_file.write("\tx_init_msg.data.push_back(0.);\n")

    cpp_file.write(
        "\treq.x_init = x_init_msg;\n"
        "\tdims.size = FORCES_NX;\n"
        "\treq.x_init.layout.dim.push_back(dims);\n\n"
        "\tlmpcc_solver::PropagateDynamicsResponse response;\n"
        "\tif (ros::service::call(\"propagate_dynamics\", req, response)){\n"
        "\t\t"+model.system.name+"DynamicsState output;\n")
    for idx, state in enumerate(model.states):
        if model.states_from_sensor[idx]:
            cpp_file.write("\t\toutput.set_" + state + "(response.outputs.data[" + str(idx) + "]);\n")

    cpp_file.write(
        "\t\tstate_ = output;\n"
        "\t\treturn output;\n"
            "\t}else{\n"
            "\t\tROS_ERROR(\"Failed to propagate the dynamics using the PropagateDynamics server (is it running?)\");\n"
            "\t\treturn initial_state;\n"
            "\t}\n")
    close_function(cpp_file)

    # Set initial system state to xinit and x0 in solver #
    header_file.write("\t/** @brief Set all initial solver values to the current state */\n")
    open_function(header_file, cpp_file, "void setSolverInitialState()")
    cpp_file.write("\t\t// Fill xinit\n")
    for idx, state in enumerate(model.states):
        if model.states_from_sensor[idx]:
            cpp_file.write("\t\tforces_params_.xinit[" + str(idx) + "] = state_." + state + "();\n")

    # Do not set the 0th initial guess values
    cpp_file.write("\n")
    cpp_file.write("\t\t// Fill first entries of x0\n")
    for idx, state in enumerate(model.states):
        if model.states_from_sensor[idx]:
            cpp_file.write("\t\t" + state + "(0) = state_." + state + "();\n")

    cpp_file.write("\n")
    cpp_file.write("\t\t// Set initial plan to previously computed plan\n")
    cpp_file.write("\t\tsetInitialPlanToPreviousPlan();\n")
    close_function(cpp_file)

    open_function(header_file, cpp_file, model.system.name+"DynamicsState& GetInitialState()")
    cpp_file.write("\t\tstatic " + model.system.name + "DynamicsState state;\n")
    for idx, state in enumerate(model.states):
        if model.states_from_sensor[idx]:
            cpp_file.write("\t\tstate." + state + "() = forces_params_.xinit[" + str(idx) + "];\n")
    cpp_file.write("\t\treturn state;\n")
    close_function(cpp_file)

    header_file.write("\n\t// Set solver values to sensor values\n")
    open_function(header_file, cpp_file, "void resetAtInfeasible()")
    cpp_file.write("\t\tfor (size_t k = 0; k < FORCES_NBAR; k++){\n")
    for idx, state in enumerate(model.states):
        if model.states_from_sensor[idx]:
            if model.states_from_sensor_at_infeasible[idx]:  # Set to the sensor
                cpp_file.write("\t\t\t" + state + "(k) = state_." + state + "();\n")
            else:   # Otherwise set to 0.0
                cpp_file.write("\t\t\t" + state + "(k) = 0.0;\n")
    cpp_file.write("\t\t}\n")
    close_function(cpp_file)

    header_file.write("\n\t// Load the solution of the solver into this class\n")
    open_function(header_file, cpp_file, "void LoadSolution(bool shift_plan_forward)", optional_header_with_defaults="void LoadSolution(bool shift_plan_forward=false)")
    cpp_file.write("\t\tinsertPredictedTrajectory(); // Loads output into x0\n\n"
            "\t\t// Save and Propagate the plan\n"
            "\t\tLoadPlan(shift_plan_forward); // Loads x0 into the plan\n"
            "\t\tLoadOptimizedVehiclePredictions(); // Load the plan into a set of vehicle predictions\n")
    close_function(cpp_file)

    header_file.write("\n\t// Load the computed plan into the plan object. Since the initial state is loaded in x01, we start at x02.\n")
    open_function(header_file, cpp_file, "void LoadPlan(bool shifted)")
    cpp_file.write("\t\tif(shifted){\n"
                      "\t\t\tLoadShiftedPlan();\n"
                      "\t\t\treturn;\n\t\t}\n\n")

    for idx, state in enumerate(model.states):
        if not model.states_from_sensor[idx]:
            continue

        cpp_file.write("\t\t/* Load " + state + " */\n")
        for k in range(0, settings.N):
            cpp_file.write("\t\tplan_.states_[" + str(k) + "].set_" + state + "(forces_output_.x" + val_zero(k+2) + "[" + str(idx+model.nu) + "]);\n")

    # Populate the control inputs
    for name, var_name in model.control_inputs.items():
        if var_name in model.inputs:
            cpp_file.write("\tcontrol_inputs_." + name + " = " + var_name + "(0);\n")
        elif var_name in model.states:
            cpp_file.write("\tcontrol_inputs_." + name + " = " + var_name + "(1);\n")
        else:
            raise IOError("Given control input variable name " + var_name + " is not part of the model")

    close_function(cpp_file)

    # The shifted plan shifts the entire plan forward
    open_function(header_file, cpp_file, "void LoadShiftedPlan()")
    for idx, state in enumerate(model.states):
        if not model.states_from_sensor[idx]:
            continue

        cpp_file.write("\t\t/* Load " + state + " */\n")
        for k in range(0, settings.N - 1):
            cpp_file.write(
                "\t\tplan_.states_[" + str(k) + "].set_" + state + "(forces_output_.x" + val_zero(k + 3) + "[" + str(
                    idx + model.nu) + "]);\n")

        # Make the terminal state the same as the panterminal state
        cpp_file.write(
                "\t\tplan_.states_[" + str(settings.N-1) + "].set_" + state + "(forces_output_.x" + val_zero(settings.N + 1) + "[" + str(
                    idx + model.nu) + "]);\n")

    # Populate the control inputs
    for name, var_name in model.control_inputs.items():
        if var_name in model.inputs:
            cpp_file.write("\tcontrol_inputs_." + name + " = " + var_name + "(0);\n")
        elif var_name in model.states:
            cpp_file.write("\tcontrol_inputs_." + name + " = " + var_name + "(1);\n")
        else:
            raise IOError("Given control input variable name " + var_name + " is not part of the model")
    close_function(cpp_file)

    # Load initial plan into the correct entries of x0
    header_file.write("\n\t// Load the initial guess into the solver (solver setting x0). "
                      "Note that since x(0) is the initial state, we load x(1), x(2), ...\n")
    open_function(header_file, cpp_file, "void loadInitialPlanAsWarmStart()")
    cpp_file.write("\t\tfor (size_t k = 0; k < FORCES_N; k++)\n\t\t{\n")

    for idx, state in enumerate(model.states):
        if not model.states_from_sensor[idx]:
            continue

        cpp_file.write("\t\t\t"+state+"(k+1) = initial_plan_.states_[k]." + state +"();\n")

    cpp_file.write("\t\t}\n\n")
    cpp_file.write("\t\t// Also set the final Forces state\n")
    for idx, state in enumerate(model.states):
        if not model.states_from_sensor[idx]:
            continue
        cpp_file.write("\t\t"+state+"(FORCES_N + 1) = " + state + "(FORCES_N);\n")
    close_function(cpp_file)

    # Access to the output data
    open_function(header_file, cpp_file, "double output(int k, int variable)")
    cpp_file.write("\t\tswitch(k){\n")
    for k in range(settings.N_bar):
        cpp_file.write("\t\t\tcase " + str(k) + ": return forces_output_.x" + val_zero(k+1) + "[variable];\n")
    cpp_file.write("\t\t\tdefault: throw std::runtime_error(\"Solver: incorrect output requested\");\n"
                      "\t\t}\n")
    close_function(cpp_file)

    open_function(header_file, cpp_file, "void setTimeout(double seconds)")
    cpp_file.write("\t\tforces_params_.solver_timeout = seconds;\n")
    close_function(cpp_file)

    # BFGS Initialization
    open_function(header_file, cpp_file,"void resetBFGS()")
    if settings.use_sqp_solver:
        cpp_file.write("\t\tdouble value;\n")
        cpp_file.write("\t\tfor (long int i = 0; i < " + str((int)(model.nvar*model.nvar/2 + model.nvar/2)) + "; i++){\n")

        idx = 0
        didx = 2
        for i in range(model.nvar):
            cpp_file.write("\t\t\t")
            if i != 0:
                cpp_file.write("else ")

            cpp_file.write("if(i == " + str(idx) + ") {value = " + str(settings.bfgs_init[i, i]) + ";}\n")
            idx += didx
            didx += 1

        cpp_file.write("\t\t\telse { value = 0.0;};\n")

        for k in range(settings.N_bar):
            if k< 10:
                cpp_file.write("\t\t\tforces_params_.BFGSinitLower0" + str(k) + "[i] = value;\n")
            else:
                cpp_file.write("\t\t\tforces_params_.BFGSinitLower" + str(k) + "[i] = value;\n")
        cpp_file.write("\t\t}\n")
    close_function(cpp_file)

    open_function(header_file, cpp_file,"void loadBFGS()")
    if settings.use_sqp_solver:
        cpp_file.write("\t\tint idx = 0;\n"
                            "\t\tint didx = 2;\n"
                            "\t\tfor(u_int i = 0; i < FORCES_TOTAL_V; i++){\n")
        for k in range(settings.N_bar):
            if k < 10:
                cpp_file.write("\t\t\tforces_params_.BFGSinitLower0" + str(k) + "[idx] = forces_output_.")
            else:
                cpp_file.write("\t\t\tforces_params_.BFGSinitLower" + str(k) + "[idx] = forces_output_.")

            if k + 1 < 10:
                cpp_file.write("BFGSdiagonal0" + str(k+1) + "[i];\n")
            else:
                cpp_file.write("BFGSdiagonal" + str(k + 1) + "[i];\n")

        cpp_file.write("\t\t\tidx += didx;\n"
                          "\t\t\tdidx++;\n"
                          "\t\t}\n")
    close_function(cpp_file)

    open_function(header_file, cpp_file, "void debugPrintSolverInfo(const unsigned int &w_solver_val)")
    if not settings.use_sqp_solver:
        cpp_file.write("\t\tunsigned int w_solver_info_table = w_solver_val + 17;\n\n"
                          "\t\tstd::cout << \"Printing all available solver information:\" << std::endl;\n"
                          "\t\tstd::cout << std::string(w_solver_info_table,'_') << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(9,' ') << \"it | \" << std::setw(w_solver_val)"
                          " << forces_info_.it << \" |\" << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(5,' ') << \"it2opt | \" << std::setw(w_solver_val)"
                          " << forces_info_.it2opt << \" |\" << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(5,' ') << \"res_eq | \" << std::setw(w_solver_val)"
                          " << forces_info_.res_eq << \" |\" << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(3,' ') << \"res_ineq | \" << std::setw(w_solver_val)"
                          " << forces_info_.res_ineq << \" |\" << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(5,' ') << \"rsnorm | \" << std::setw(w_solver_val)"
                          " << forces_info_.rsnorm << \" |\" << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(2,' ') << \"rcompnorm | \" << std::setw(w_solver_val)"
                          " << forces_info_.rcompnorm << \" |\" << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(7,' ') << \"pobj | \" << std::setw(w_solver_val)"
                          " << forces_info_.pobj << \" |\" << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(7,' ') << \"dobj | \" << std::setw(w_solver_val)"
                          " << forces_info_.dobj << \" |\" << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(7,' ') << \"dgap | \" << std::setw(w_solver_val)"
                          " << forces_info_.dgap << \" |\" << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(6,' ') << \"rdgap | \" << std::setw(w_solver_val)"
                          " << forces_info_.rdgap << \" |\" << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(9,' ') << \"mu | \" << std::setw(w_solver_val)"
                          " << forces_info_.mu << \" |\" << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(5,' ') << \"mu_aff | \" << std::setw(w_solver_val)"
                          " << forces_info_.mu_aff << \" |\" << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(6,' ') << \"sigma | \" << std::setw(w_solver_val)"
                          " << forces_info_.sigma << \" |\" << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(3,' ') << \"lsit_aff | \" << std::setw(w_solver_val)"
                          " << forces_info_.lsit_aff << \" |\" << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(4,' ') << \"lsit_cc | \" << std::setw(w_solver_val)"
                          " << forces_info_.lsit_cc << \" |\" << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(3,' ') << \"step_aff | \" << std::setw(w_solver_val)"
                          " << forces_info_.step_aff << \" |\" << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(4,' ') << \"step_cc | \" << std::setw(w_solver_val)"
                          " << forces_info_.step_cc << \" |\" << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(2,' ') << \"solvetime | \" << std::setw(w_solver_val)"
                          " << forces_info_.solvetime << \" |\" << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(1,' ') << \"fevalstime | \""
                          " << std::setw(w_solver_val) << forces_info_.fevalstime << \" |\" << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(w_solver_info_table-2,'_') << \"|\" << std::endl;\n"
                          "\t\tstd::cout << std::endl;\n"
                          "\t\tstd::cout << \"Done printing all available solver information\" << std::endl;\n"
                          "\t\tstd::cout << std::endl;\n")
    else:
        cpp_file.write("\t\tunsigned int w_solver_info_table = w_solver_val + 17;\n\n"
                          "\t\tstd::cout << \"Printing all available solver information:\" << std::endl;\n"
                          "\t\tstd::cout << std::string(w_solver_info_table,'_') << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(9,' ') << \"it | \" << std::setw(w_solver_val)"
                          " << forces_info_.it << \" |\" << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(5,' ') << \"res_eq | \" << std::setw(w_solver_val)"
                          " << forces_info_.res_eq << \" |\" << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(5,' ') << \"rsnorm | \" << std::setw(w_solver_val)"
                          " << forces_info_.rsnorm << \" |\" << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(7,' ') << \"pobj | \" << std::setw(w_solver_val)"
                          " << forces_info_.pobj << \" |\" << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(2,' ') << \"solvetime | \" << std::setw(w_solver_val)"
                          " << forces_info_.solvetime << \" |\" << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(1,' ') << \"fevalstime | \""
                          " << std::setw(w_solver_val) << forces_info_.fevalstime << \" |\" << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(5,' ') << \"QPtime | \" << std::setw(w_solver_val)"
                          " << forces_info_.it << \" |\" << std::endl;\n"
                          "\t\tstd::cout << \"|\" << std::string(w_solver_info_table-2,'_') << \"|\" << std::endl;\n"
                          "\t\tstd::cout << std::endl;\n"
                          "\t\tstd::cout << \"Done printing all available solver information\" << std::endl;\n"
                          "\t\tstd::cout << std::endl;\n")
    close_function(cpp_file)

    open_function(header_file, cpp_file, "void debugPrintAll()")
    cpp_file.write("\t\t/* Pre-determined size and precision settings */\n"
                      "\t\tconst unsigned int prec = 4;\n\n"
                      "\t\tconst unsigned int w_idx = 3;\n"
                      "\t\tconst unsigned int w_solver_val = prec + 7;\n\n"
                      "\t\tconst unsigned int w_col1 = 13;\n"
                      "\t\tconst unsigned int w_coln = w_solver_val + 2;\n\n"
                      "\t\tconst unsigned int max_stage_col = 11; // w_col1=14, w_coln=12, FORCES_NBAR=15 is using full"
                      " terminal width on full HD screen, just to be on the safe side\n\n"
                      "\t\t/* Determine amount and sizes of tables to print and their corresponding amount of columns"
                      "*/\n"
                      "\t\tconst unsigned int n_tables = std::ceil(double(FORCES_NBAR)/max_stage_col);\n"
                      "\t\tconst unsigned int n_stage_col_not_last = std::min(FORCES_NBAR,max_stage_col);\n"
                      "\t\tconst unsigned int n_stage_col_last = FORCES_NBAR % max_stage_col;\n\n"
                      "\t\t/* Start debug print statement */\n"
                      "\t\tstd::cout << std::setprecision(prec);\n"
                      "\t\tstd::cout << std::endl;\n"
                      "\t\tstd::cout << \"Printing all available information per optimization stage:\" << std::endl;"
                      "\n"
                      "\t\tstd::cout << \"IMPORTANT NOTE:\" << std::endl;\n"
                      "\t\tstd::cout << \"k = 0 => table shows calculated inputs and initial states\" << std::endl;"
                      "\n"
                      "\t\tstd::cout << \"k = 1 => table shows calculated inputs and 1st state update\" << "
                      "std::endl;\n"
                      "\t\tstd::cout << \"...\" << std::endl;\n"
                      "\t\tstd::cout << \"k = N => table shows calculated inputs and Nth state update\" << "
                      "std::endl;\n"
                      "\t\tstd::cout << \"The plan consists of the inputs for k in [0, N_bar-3] ([0, " +
                      str(settings.N_bar-3) + "]) and the states for k in [1, N_bar-2] ([1, " + str(settings.N_bar-2) +
                      "]), both with length N = " + str(settings.N) + "\" << std::endl;\n"
                      "\t\tstd::cout << \"The tables below display the plan for the states\" << std::endl;\n\n"
                      "\t\t/* Changing variables over the different tables */\n"
                      "\t\t// k_start_table and n_stage_col determine together the starting stage and ending stage of"
                      " the table (thereby implicitly defining the table size)\n"
                      "\t\tunsigned int k_start_table = 0;\n"
                      "\t\tunsigned int n_stage_col;\n"
                      "\t\tunsigned int w_table;\n"
                      "\t\tstd::string array_name;\n"
                      "\t\tbool is_last_table;\n\n"
                      "\t\t/* Print all stage tables */\n"
                      "\t\tfor (unsigned int table_idx = 0; table_idx < n_tables; table_idx++) {\n"
                      "\t\t\t/* Determine table size (especially important in case of last table, which might deviate"
                      " from the rest) */\n"
                      "\t\t\tif (table_idx == n_tables-1 && n_stage_col_last != 0)\n"
                      "\t\t\t\tn_stage_col = n_stage_col_last;\n"
                      "\t\t\telse\n"
                      "\t\t\t\tn_stage_col = n_stage_col_not_last;\n"
                      "\t\t\tw_table = w_col1 + 1 + n_stage_col*(w_coln + 1) + 1;\n\n"
                      "\t\t\tif (table_idx == n_tables - 1)\n"
                      "\t\t\t\tis_last_table = true;\n\n"
                      "\t\t\t/* Call printing functionality for each table */\n"
                      "\t\t\tdebugPrintTableBegin(w_table, k_start_table, n_stage_col, w_col1, w_coln, is_last_table);"
                      "\n\n"
                      "\t\t\t// Print all solver settings\n"
                      "\t\t\tdebugPrintTableVariablesString(w_table, \"Solver settings\");\n"
                      "\t\t\tdebugPrintTableEmptyColLine(n_stage_col, w_col1, w_coln, true);\n"
                      "\t\t\tdebugPrintTableEmptyColLine(n_stage_col, w_col1, w_coln, false);\n"
                      "\t\t\tif (table_idx == 0) {\n"
                      "\t\t\t\tdebugPrintTableArrayEntry(k_start_table, n_stage_col, w_col1, w_coln, w_idx,"
                      " w_solver_val, \"xinit\"); // xinit (only in first table)\n"
                      "\t\t\t\tdebugPrintTableEmptyColLine(n_stage_col, w_col1, w_coln, false);\n"
                      "\t\t\t}\n"
                      "\t\t\tdebugPrintTableArrayEntry(k_start_table, n_stage_col, w_col1, w_coln, w_idx,"
                      " w_solver_val, \"x0\"); // x0\n"
                      "\t\t\tdebugPrintTableEmptyColLine(n_stage_col, w_col1, w_coln, false);\n"
                      "\t\t\tdebugPrintTableArrayEntry(k_start_table, n_stage_col, w_col1, w_coln, w_idx,"
                      " w_solver_val, \"params\"); // params\n"
                      "\t\t\tdebugPrintTableEmptyLine(w_table);\n\n"
                      "\t\t\t// Print all solver outputs\n"
                      "\t\t\tdebugPrintTableVariablesString(w_table, \"Solver outputs\");\n"
                      "\t\t\tdebugPrintTableEmptyColLine(n_stage_col, w_col1, w_coln, true);\n"
                      "\t\t\tdebugPrintTableEmptyColLine(n_stage_col, w_col1, w_coln, false);\n"
                      "\t\t\tdebugPrintTableArrayEntry(k_start_table, n_stage_col, w_col1, w_coln, w_idx,"
                      " w_solver_val, \"inputs\"); // inputs\n"
                      "\t\t\tdebugPrintTableEmptyColLine(n_stage_col, w_col1, w_coln, false);\n"
                      "\t\t\tdebugPrintTableArrayEntry(k_start_table, n_stage_col, w_col1, w_coln, w_idx,"
                      " w_solver_val, \"states\"); // states\n"
                      "\t\t\tdebugPrintTableEnd(w_table);\n\n\n"
                      "\t\t\t/* Update starting stage index for next table */\n"
                      "\t\t\tk_start_table += n_stage_col;\n"
                      "\t\t}\n\n"
                      "\t\t/* End debug print statement */\n"
                      "\t\tstd::cout << std::endl;\n"
                      "\t\tstd::cout << \"Done printing all available information per optimization stage\" <<"
                      " std::endl;\n"
                      "\t\tstd::cout << std::endl;\n\n"
                      "\t\t/* Print all solver information */\n"
                      "\t\tdebugPrintSolverInfo(w_solver_val);\n")
    close_function(cpp_file)

    open_function(header_file, cpp_file, "void debugPrintTableBegin(const unsigned int &w_table,"
                               " const unsigned int &k_start_table, const unsigned int &n_stage_col,"
                               " const unsigned int &w_col1, const unsigned int &w_coln, bool &is_last_table)")
    cpp_file.write("\t\tstd::cout << std::string(w_table,'_') << std::endl;\n"
                      "\t\tstd::cout << \"|\" << std::string(w_col1,' ');\n"
                      "\t\tfor (unsigned int k = k_start_table; k < k_start_table+n_stage_col; k++) {\n"
                      "\t\t\tstd::cout << \"|\" << std::string(w_coln-8,' ');\n"
                      "\t\t\tif (k < 10)\n"
                      "\t\t\t\tstd::cout << \"  \";\n"
                      "\t\t\telse if (k < 100)\n"
                      "\t\t\t\tstd::cout << \" \";\n"
                      "\t\t\tstd::cout << \"k = \" << k << \" \";\n"
                      "\t\t}\n"
                      "\t\tstd::cout << \"|\" << std::endl;\n"
                      "\t\tstd::cout << \"|\" << std::string(w_table-2,'=') << \"|\" << std::endl;\n\n"
                      "\t\t// Indicate start and end of plan\n"
                      "\t\tif (k_start_table == 0) {\n"
                      "\t\t\tstd::cout << \"|\" << std::string(w_col1+1+w_coln,'=') << \"| Start of Plan \";\n"
                      "\t\t\tif (!is_last_table) {\n"
                      "\t\t\t\tstd::cout << std::string(w_table-1-w_col1-1-w_coln-16-2,'.') << \" |\" << std::endl;\n"
                      "\t\t\t} else {\n"
                      "\t\t\t\tstd::cout << std::string(w_table-1-w_col1-1-w_coln-16-14-w_coln-1,'.')"
                      " << \" end of Plan |\" << std::string(w_coln,'=') << \"|\" << std::endl;\n"
                      "\t\t\t\treturn;\n"
                      "\t\t\t}\n"
                      "\t\t} else if (!is_last_table) {\n"
                      "\t\t\tstd::cout << \"| \" << std::string(w_table-4,'.') << \" |\" << std::endl;\n"
                      "\t\t}\n"
                      "\t\tif (is_last_table) {\n"
                      "\t\t\tstd::cout << \"| \" << std::string(w_table-2-14-w_coln-1,'.') << \" end of Plan |\""
                      " << std::string(w_coln,'=') << \"|\" << std::endl;\n"
                      "\t\t}\n")
    close_function(cpp_file)

    open_function(header_file, cpp_file, "void debugPrintTableEnd(const unsigned int &w_table)")
    cpp_file.write("\t\tstd::cout << \"|\" << std::string(w_table-2,'_') << \"|\" << std::endl;\n")
    close_function(cpp_file)

    open_function(header_file, cpp_file, "void debugPrintTableEmptyLine(const unsigned int &w_table)")
    cpp_file.write("\t\tstd::cout << \"|\" << std::string(w_table-2,' ') << \"|\" << std::endl;\n")
    close_function(cpp_file)

    open_function(header_file, cpp_file, "void debugPrintTableEmptyColLine(const unsigned int &n_stage_col,"
                               " const unsigned int &w_col1, const unsigned int &w_coln, const bool &is_idx_line)")
    cpp_file.write("\t\tfor (unsigned int k = 0; k < n_stage_col+1; k++) {\n"
                      "\t\t\tif (k == 0) {\n"
                      "\t\t\t\tif (is_idx_line)\n"
                      "\t\t\t\t\tstd::cout << \"|\" << std::string(w_col1-4,' ') << \"idx \";\n"
                      "\t\t\t\telse\n"
                      "\t\t\t\t\tstd::cout << \"|\" << std::string(w_col1,' ');\n"
                      "\t\t\t}\n"
                      "\t\t\telse if (k < n_stage_col)\n"
                      "\t\t\t\tstd::cout << \"|\" << std::string(w_coln,' ');\n"
                      "\t\t\telse\n"
                      "\t\t\t\tstd::cout << \"|\" << std::string(w_coln,' ') << \"|\" << std::endl;\n"
                      "\t\t}\n")
    close_function(cpp_file)

    open_function(header_file, cpp_file, "void debugPrintTableVariablesString(const unsigned int &w_table, "
                               "const std::string &name)")
    cpp_file.write("\t\tstd::cout << \"|\" << std::string(w_table-2,'-') << \"|\" << std::endl;\n"
                      "\t\tunsigned int str_len = name.length();\n"
                      "\t\tstd::cout << \"| \" << name << std::string(w_table-str_len-3,' ') << \"|\" << std::endl;\n"
                      "\t\tstd::cout << \"|\" << std::string(w_table-2,'-') << \"|\" << std::endl;\n")
    close_function(cpp_file)

    open_function(header_file, cpp_file, "void debugPrintTableArrayEntry(const unsigned int &k_start_table,"
                               " const unsigned int &n_stage_col, const unsigned int &w_col1,"
                               " const unsigned int &w_coln, const unsigned int &w_idx,"
                               "const unsigned int &w_solver_val, const std::string &name)")
    cpp_file.write("\t\tunsigned int idx_max = 0;\n\n"
                      "\t\tbool is_xinit = false;\n"
                      "\t\tbool is_x0 = false;\n"
                      "\t\tbool is_params = false;\n"
                      "\t\tbool is_inputs = false;\n"
                      "\t\tbool is_states = false;\n\n"
                      "\t\tif (!name.compare(\"xinit\")) {\n"
                      "\t\t\tis_xinit = true;\n"
                      "\t\t\tidx_max = FORCES_NX;\n"
                      "\t\t} else if (!name.compare(\"x0\")) {\n"
                      "\t\t\tis_x0 = true;\n"
                      "\t\t\tidx_max = FORCES_TOTAL_V;\n"
                      "\t\t} else if (!name.compare(\"params\")) {\n"
                      "\t\t\tis_params = true;\n"
                      "\t\t\tidx_max = FORCES_NPAR;\n"
                      "\t\t} else if (!name.compare(\"inputs\")) {\n"
                      "\t\t\tis_inputs = true;\n"
                      "\t\t\tidx_max = FORCES_NU;\n"
                      "\t\t} else if (!name.compare(\"states\")) {\n"
                      "\t\t\tis_states = true;\n"
                      "\t\t\tidx_max = FORCES_NX;\n"
                      "\t\t}\n\n"
                      "\t\tfor (unsigned int idx = 0; idx < idx_max; idx++) {\n"
                      "\t\t\tif (idx == 0) {\n"
                      "\t\t\t\tunsigned int str_len = name.length();\n"
                      "\t\t\t\tstd::cout << \"|\" << std::string(w_col1-str_len-w_idx-3,' ') << name << \": \" <<"
                      " std::setw(w_idx) << idx << \" \";\n"
                      "\t\t\t} else\n"
                      "\t\t\t\tstd::cout << \"|\" << std::string(w_col1-w_idx-1,' ') << std::setw(w_idx) << idx <<"
                      " \" \";\n"
                      "\t\t\tfor (unsigned int k = k_start_table; k < k_start_table+n_stage_col; k++) {\n"
                      "\t\t\t\tif (is_xinit) {\n"
                      "\t\t\t\t\tif (k == 0) // xinit only exists in first stage (0)\n"
                      "\t\t\t\t\t\tstd::cout << \"| \" << std::setw(w_solver_val) << forces_params_.xinit[idx] <<"
                      " \" \";\n"
                      "\t\t\t\t\telse\n"
                      "\t\t\t\t\t\tstd::cout << \"|\" << std::string(w_coln,' ');\n"
                      "\t\t\t\t} else if (is_x0) {\n"
                      "\t\t\t\t\tstd::cout << \"| \" << std::setw(w_solver_val) <<"
                      " forces_params_.x0[k*FORCES_TOTAL_V + idx] << \" \";\n"
                      "\t\t\t\t} else if (is_params) {\n"
                      "\t\t\t\t\tstd::cout << \"| \" << std::setw(w_solver_val) <<"
                      " forces_params_.all_parameters[k*FORCES_NPAR + idx] << \" \";\n"
                      "\t\t\t\t} else if (is_inputs) {\n"
                      "\t\t\t\t\tstd::cout << \"| \" << std::setw(w_solver_val) << output(k,idx) << \" \";\n"
                      "\t\t\t\t} else if (is_states) {\n"
                      "\t\t\t\t\tstd::cout << \"| \" << std::setw(w_solver_val) << output(k,FORCES_NU + idx)"
                      " << \" \";\n"
                      "\t\t\t\t}\n"
                      "\t\t\t}\n"
                      "\t\t\tstd::cout << \"|\" << std::endl;\n"
                      "\t\t}\n")
    close_function(cpp_file)

    # Add set functions for all parameters defined by name
    # for key, param in settings.params.parameters.items():
    #     open_function(header_file, cpp_file, "void set_" + param + "(unsigned int k, double value)")
    #     idx = getattr(settings.params, param + "_index")
    #     cpp_file.write("\t\tsetParameterSpecific(k, " + str(idx) + ", value);\n")
    #     close_function(cpp_file)

    header_file.write("};\n")
    header_file.write("#endif\n")
    header_file.close()
