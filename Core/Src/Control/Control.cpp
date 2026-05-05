#include "Control/Control.hpp"
#include "LCU_SLAVE.hpp"

namespace Control {

// 5DOF control instance (CONTROLH10_1 class-based interface)
#ifdef USE_5_DOF
static CONTROLH10_1 control_5dof;
CONTROLH10_1::ExtU_CONTROLH10_1_T inputs{};
#endif

void init() {
}

ControlOutput current_update(std::optional<float> desired_current) {
#ifdef USE_1_DOF
    control_U.corriente_real = LCU_Slave::lpu_array.get_lpu<0>().shunt_v;

    if (desired_current.has_value()) {
        // CURRENT_CONTROL mode: bypass outer loop and inject current setpoint directly.
        control_DW.RateTransition_Buffer0 = desired_current.value();
    }

    control_step0();

    return ControlOutput{control_Y.Voltage, control_Y.z1, control_Y.z2, control_Y.z3};

#elif defined(USE_5_DOF)
    // 5DOF: Step the fast control loop with new class-based interface
    
    // Map current sensors (I_HEMS indices)
    inputs.I_HEMS[0] = LCU_Slave::lpu_array.get_lpu<3>().shunt_v;
    inputs.I_HEMS[1] = LCU_Slave::lpu_array.get_lpu<9>().shunt_v;
    inputs.I_HEMS[2] = LCU_Slave::lpu_array.get_lpu<5>().shunt_v;
    inputs.I_HEMS[3] = LCU_Slave::lpu_array.get_lpu<7>().shunt_v;

    if (desired_current.has_value()) {
        // Manual current control - set all hems to desired value (if supported)
        inputs.CorrienteManual = desired_current.value();
    }
    control_5dof.setExternalInputs(&inputs);
 
    control_5dof.step0();
    
    const auto& outputs = control_5dof.getExternalOutputs();
    return ControlOutput{static_cast<float>(outputs.V[0]), static_cast<float>(outputs.V[1]), 
                         static_cast<float>(outputs.V[2]), static_cast<float>(outputs.V[3])};
#endif
}

void levitation_update(float reference) {
#ifdef USE_1_DOF
    control_U.Gap = LCU_Slave::airgap_array.get_airgap<0>().airgap_v;
    control_U.Referencia = reference;

    control_step1();

#elif defined(USE_5_DOF)
    // 5DOF: Collect airgap sensor values and update levitation reference
    
    inputs.RefZ = reference;
    
    // Map airgap sensors (Sensores array, 0-indexed)
    // Physical airgaps 1, 4, 3, 2 map to Sensores indices as needed
    inputs.Sensores[0] = LCU_Slave::airgap_array.get_airgap<4>().airgap_v;
    inputs.Sensores[1] = LCU_Slave::airgap_array.get_airgap<1>().airgap_v;
    inputs.Sensores[2] = LCU_Slave::airgap_array.get_airgap<2>().airgap_v;
    inputs.Sensores[3] = LCU_Slave::airgap_array.get_airgap<3>().airgap_v;
    // Additional sensors if used
    inputs.Sensores[4] = 0.0; // unused
    inputs.Sensores[5] = 0.0; // unused
    inputs.Sensores[6] = 0.0; // unused
    inputs.Sensores[7] = 0.0; // unused
    
    inputs.ManualLevitacin = 1.0; // Enable levitation control
    control_5dof.setExternalInputs(&inputs);
    
    control_5dof.step1();
#endif
}

} // Namespace Control
