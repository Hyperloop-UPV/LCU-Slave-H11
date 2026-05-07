#include "Control/Control.hpp"
#include "LCU_SLAVE.hpp"

namespace Control {

void init() {
    model.initialize();
}

void deinit() {
    model.terminate();
}

std::array<float, CONTROL_LPU_COUNT> current_update(std::array<float, CONTROL_LPU_COUNT> input_currents, std::optional<float> desired_current) {
    
    for (size_t i = 0; i < CONTROL_LPU_COUNT; i++) {
        inputs.I_HEMS[i] = input_currents[i];
    }

    if (desired_current.has_value()) {
        inputs.CorrienteManual = desired_current.value();
        inputs.ManualLevitacin = 0.0; // Disable levitation control, enable manual current control
    }

    model.setExternalInputs(&inputs);
 
    model.step0();

    std::array<float, CONTROL_LPU_COUNT> output_currents{};
    for (size_t i = 0; i < CONTROL_LPU_COUNT; i++) {
        output_currents[i] = static_cast<float>(output.Voltages[i]);
    }
    
    return output_currents;
}

void levitation_update(std::array<float, CONTROL_AIRGAP_COUNT> input_airgaps, float reference) {

    for (size_t i = 0; i < CONTROL_AIRGAP_COUNT; i++) {
        inputs.Sensores[i] = input_airgaps[i];
    }

    // inputs.Sensores[0] = LCU_Slave::airgap_array.get_airgap<4>().airgap_v;
    // inputs.Sensores[1] = LCU_Slave::airgap_array.get_airgap<1>().airgap_v;
    // inputs.Sensores[2] = LCU_Slave::airgap_array.get_airgap<2>().airgap_v;
    // inputs.Sensores[3] = LCU_Slave::airgap_array.get_airgap<3>().airgap_v;

    inputs.RefZ = reference;
    inputs.ManualLevitacin = 1.0; // Enable levitation control
    inputs.CorrienteManual = 0.0; // Disable manual current control
    
    model.setExternalInputs(&inputs);
    
    model.step1();
}

} // Namespace Control
