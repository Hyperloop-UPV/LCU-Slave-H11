#include "Control/Control.hpp"
#include "LCU_SLAVE.hpp"

namespace Control {
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
    // 5DOF: Step the fast control loop
    control_U.I_real_hems[0] = LCU_Slave::lpu_array.get_lpu<9>().shunt_v;
    control_U.I_real_hems[1] = LCU_Slave::lpu_array.get_lpu<3>().shunt_v;
    control_U.I_real_hems[2] = LCU_Slave::lpu_array.get_lpu<7>().shunt_v;
    control_U.I_real_hems[3] = LCU_Slave::lpu_array.get_lpu<5>().shunt_v;

    if (desired_current.has_value()) {
        control_DW.Corriente_Buffer0[0] = desired_current.value();
        control_DW.Corriente_Buffer0[1] = desired_current.value();
        control_DW.Corriente_Buffer0[2] = desired_current.value();
        control_DW.Corriente_Buffer0[3] = desired_current.value();
    }
    
    return ControlOutput{control_Y.I[0], control_Y.I[1], control_Y.I[2], control_Y.I[3]};
#endif
}

void levitation_update(float reference) {
#ifdef USE_1_DOF
    control_U.Gap = LCU_Slave::airgap_array.get_airgap<0>().airgap_v;
    control_U.Referencia = reference;

    control_step1();

#elif defined(USE_5_DOF)
    // 5DOF: Collect all 8 airgap sensor values and update levitation reference
    control_U.Refz = reference;
    control_U.Airgaps_Model[0] = LCU_Slave::airgap_array.get_airgap<1>().airgap_v;
    control_U.Airgaps_Model[1] = LCU_Slave::airgap_array.get_airgap<4>().airgap_v;
    control_U.Airgaps_Model[2] = LCU_Slave::airgap_array.get_airgap<3>().airgap_v;
    control_U.Airgaps_Model[3] = LCU_Slave::airgap_array.get_airgap<2>().airgap_v;

    control_step1();
#endif
}

} // Namespace Control
