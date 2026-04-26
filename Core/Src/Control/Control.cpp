#include "Control/Control.hpp"
#include "LCU_SLAVE.hpp"

namespace Control {
ControlOutput current_update(std::optional<float> desired_current) {
#ifdef USE_1_DOF
    control_U.corriente_real = LCU_Slave::lpu_array.get_lpu<0>().shunt_v;

    if (desired_current.has_value()) {
        if (use_direct_current_reference) {
            // CURRENT_CONTROL mode: bypass outer loop and inject current setpoint directly.
            control_DW.RateTransition_Buffer0 = desired_current.value();
        }
    }

    control_step0();

    return ControlOutput{control_Y.Voltage, control_Y.z1, control_Y.z2, control_Y.z3};

#elif defined(USE_5_DOF)
    // 5DOF: Step the fast control loop
    inputs.I_HEMS.I_HEMS1 = LCU_Slave::lpu_array.get_lpu<0>().shunt_v;
    inputs.I_HEMS.I_HEMS2 = LCU_Slave::lpu_array.get_lpu<1>().shunt_v;
    inputs.I_HEMS.I_HEMS3 = LCU_Slave::lpu_array.get_lpu<2>().shunt_v;
    inputs.I_HEMS.I_HEMS4 = LCU_Slave::lpu_array.get_lpu<3>().shunt_v;
    inputs.I_EMS.I_EMS5 = LCU_Slave::lpu_array.get_lpu<4>().shunt_v;
    inputs.I_EMS.I_EMS6 = LCU_Slave::lpu_array.get_lpu<5>().shunt_v;
    inputs.I_EMS.I_EMS7 = LCU_Slave::lpu_array.get_lpu<6>().shunt_v;
    inputs.I_EMS.I_EMS8 = LCU_Slave::lpu_array.get_lpu<7>().shunt_v;
    inputs.I_EMS.I_EMS9 = LCU_Slave::lpu_array.get_lpu<8>().shunt_v;
    inputs.I_EMS.I_EMS10 = LCU_Slave::lpu_array.get_lpu<9>().shunt_v;
    
    control.step0();


    ControlOutput output;
    
    if (desired_current.has_value()) {
        // CURRENT_CONTROL mode: Apply desired current directly to all 10 LPUs
        output.voltages[0] = desired_current.value();
        output.voltages[1] = desired_current.value();
        output.voltages[2] = desired_current.value();
        output.voltages[3] = desired_current.value();
        output.voltages[4] = desired_current.value();
        output.voltages[5] = desired_current.value();
        output.voltages[6] = desired_current.value();
        output.voltages[7] = desired_current.value();
        output.voltages[8] = desired_current.value();
        output.voltages[9] = desired_current.value();
    } else {
        // Return the computed voltages for all 10 LPUs
        const auto& outputs = control.getExternalOutputs();
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
    control_U.Gap = LCU_Slave::airgap_array.get_airgap<0>().airgap_v;
    control_U.Referencia = reference;

    control_step1();

#elif defined(USE_5_DOF)
    // 5DOF: Collect all 8 airgap sensor values and update levitation reference
    inputs.SensoresPos.AG_Z1 = LCU_Slave::airgap_array.get_airgap<0>().airgap_v;
    inputs.SensoresPos.AG_Z2 = LCU_Slave::airgap_array.get_airgap<1>().airgap_v;
    inputs.SensoresPos.AG_Z3 = LCU_Slave::airgap_array.get_airgap<2>().airgap_v;
    inputs.SensoresPos.AG_Z4 = LCU_Slave::airgap_array.get_airgap<3>().airgap_v;
    inputs.SensoresPos.AG_Y5 = LCU_Slave::airgap_array.get_airgap<4>().airgap_v;
    inputs.SensoresPos.AG_Y6 = LCU_Slave::airgap_array.get_airgap<5>().airgap_v;
    inputs.SensoresPos.AG_Y7 = LCU_Slave::airgap_array.get_airgap<6>().airgap_v;
    inputs.SensoresPos.AG_Y8 = LCU_Slave::airgap_array.get_airgap<7>().airgap_v;
    inputs.RefZ = reference;

    control.step1();
#endif
}

} // Namespace Control
