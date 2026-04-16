#ifndef CONTROL_HPP
#define CONTROL_HPP

#include "C++Utilities/CppImports.hpp"
#include "LCU_SLAVE_Types.hpp"

extern "C" {
#ifdef USE_1_DOF
#include "control.h"
#endif
}

#ifdef USE_5_DOF
#include "C_5DOF_EL.h"
#endif

namespace Control {

#ifdef USE_5_DOF
static C_5DOF_EL g_c5dof_el;
static C_5DOF_EL::ExtU_C_5DOF_EL_T g_5dof_inputs{};  // Maintain inputs across calls
#endif

// Control output structure containing voltage(s)
struct ControlOutput {
#ifdef USE_1_DOF
    float voltage;  // 1DOF: single output voltage
    float z1, z2, z3;  // State variables
#elif defined(USE_5_DOF)
    std::array<float, 10> voltages;  // 5DOF: voltage per LPU (10 total)
#endif
};

void init() {
#ifdef USE_1_DOF
    control_initialize();
#elif defined(USE_5_DOF)
    g_c5dof_el.initialize();
#endif
}

ControlOutput current_update(float desired_current = 0.0f, bool use_direct_current_reference = false) {
#ifdef USE_1_DOF
    control_U.corriente_real = LCU_Slave::g_lpu_array->get_lpu<0>().shunt_v;

    if (use_direct_current_reference) {
        // CURRENT_CONTROL mode: bypass outer loop and inject current setpoint directly.
        control_DW.RateTransition_Buffer0 = desired_current;
    }

    control_step0();

    return ControlOutput{control_Y.Voltage, control_Y.z1, control_Y.z2, control_Y.z3};

#elif defined(USE_5_DOF)
    // 5DOF: Step the fast control loop (0.5ms rate)
    g_c5dof_el.setExternalInputs(&g_5dof_inputs);
    g_c5dof_el.step0();

    ControlOutput output;
    
    if (use_direct_current_reference) {
        // CURRENT_CONTROL mode: Apply desired current directly to all 10 LPUs
        output.voltages[0] = desired_current;
        output.voltages[1] = desired_current;
        output.voltages[2] = desired_current;
        output.voltages[3] = desired_current;
        output.voltages[4] = desired_current;
        output.voltages[5] = desired_current;
        output.voltages[6] = desired_current;
        output.voltages[7] = desired_current;
        output.voltages[8] = desired_current;
        output.voltages[9] = desired_current;
    } else {
        // Return the computed voltages for all 10 LPUs
        const auto& outputs = g_c5dof_el.getExternalOutputs();
        output.voltages[0] = static_cast<float>(outputs.V_h.V_HEMS1);
        output.voltages[1] = static_cast<float>(outputs.V_h.V_HEMS2);
        output.voltages[2] = static_cast<float>(outputs.V_h.V_HEMS3);
        output.voltages[3] = static_cast<float>(outputs.V_h.V_HEMS4);
        output.voltages[4] = static_cast<float>(outputs.V_h.V_EMS5);
        output.voltages[5] = static_cast<float>(outputs.V_h.V_EMS6);
        output.voltages[6] = static_cast<float>(outputs.V_h.V_EMS7);
        output.voltages[7] = static_cast<float>(outputs.V_h.V_EMS8);
        output.voltages[8] = static_cast<float>(outputs.V_h.V_EMS9);
        output.voltages[9] = static_cast<float>(outputs.V_h.V_EMS10);
    }

    return output;
#endif
}

void levitation_update(float reference) {
#ifdef USE_1_DOF
    control_U.Gap = LCU_Slave::g_airgap_array->get_airgap<0>().airgap_v;
    control_U.Referencia = reference;

    control_step1();

#elif defined(USE_5_DOF)
    // 5DOF: Collect all 8 airgap sensor values and update levitation reference
    g_5dof_inputs.SensoresPos.AG_Z1 = LCU_Slave::g_airgap_array->get_airgap<0>().airgap_v;
    g_5dof_inputs.SensoresPos.AG_Z2 = LCU_Slave::g_airgap_array->get_airgap<1>().airgap_v;
    g_5dof_inputs.SensoresPos.AG_Z3 = LCU_Slave::g_airgap_array->get_airgap<2>().airgap_v;
    g_5dof_inputs.SensoresPos.AG_Z4 = LCU_Slave::g_airgap_array->get_airgap<3>().airgap_v;
    g_5dof_inputs.SensoresPos.AG_Y5 = LCU_Slave::g_airgap_array->get_airgap<4>().airgap_v;
    g_5dof_inputs.SensoresPos.AG_Y6 = LCU_Slave::g_airgap_array->get_airgap<5>().airgap_v;
    g_5dof_inputs.SensoresPos.AG_Y7 = LCU_Slave::g_airgap_array->get_airgap<6>().airgap_v;
    g_5dof_inputs.SensoresPos.AG_Y8 = LCU_Slave::g_airgap_array->get_airgap<7>().airgap_v;
    g_5dof_inputs.RefZ = reference;
    
    g_c5dof_el.setExternalInputs(&g_5dof_inputs);

    // Step the outer control loop (1ms rate)
    g_c5dof_el.step1();
#endif
}

void deinit() {
#ifdef USE_1_DOF
    control_terminate();
#elif defined(USE_5_DOF)
    C_5DOF_EL::terminate();
#endif
}
}; // namespace Control

#endif // CONTROL_HPP
