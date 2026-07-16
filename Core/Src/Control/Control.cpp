#include "Control/Control.hpp"
#include "LCU_SLAVE.hpp"

namespace Control {

void init() { model.initialize(); }

void deinit() {
    model.terminate();
    control.output.clear();
}

void update_control() {
    auto& ctrl = control.output;
    for (int i = 0; i < 10; i++)
        ctrl.Voltages[i] = static_cast<float>(output.Voltages[i]);
    for (int i = 0; i < 4; i++)
        ctrl.GapsLocales[i] = static_cast<float>(output.GL[i]);
    for (int i = 0; i < 5; i++)
        ctrl.Estados[i] = static_cast<float>(output.Estados[i]);
    for (int i = 0; i < 4; i++)
        ctrl.CorrienteReferencia[i] = static_cast<float>(output.CorrienteReferencia[i]);
    ctrl.Referencia = static_cast<float>(output.Referencia);
    for (int i = 0; i < 8; i++)
        ctrl.A[i] = static_cast<float>(output.A[i]);
}

std::array<float, LCUConfig::ACTIVE_LPU_COUNT> current_update(
    const std::array<float, LCUConfig::ACTIVE_LPU_COUNT>& input_currents,
    std::optional<float> desired_current
) {
    if (control.input.cinema) {
        inputs.ABSOLUTECINEMA = 1.0;
        inputs.amp_A = control.input.cinema_current;
    } else {
        inputs.ABSOLUTECINEMA = 0.0;
        inputs.amp_A = 0.0;
    }

    for (size_t i = 0; i < LCUConfig::ACTIVE_LPU_COUNT; i++) {
        inputs.I[i] = input_currents[i];
    }

    if (desired_current.has_value()) {
        for (int i = 0; i < 10; i++)
            inputs.CorrienteManual[i] = desired_current.value();
        inputs.ManualLevitacin = 0.0;
    }

    model.setExternalInputs(&inputs);
    model.step0();

    std::array<float, LCUConfig::ACTIVE_LPU_COUNT> output_currents{};
    for (size_t i = 0; i < LCUConfig::ACTIVE_LPU_COUNT; i++) {
        output_currents[i] = static_cast<float>(output.Voltages[i]);
    }

    update_control();

    return output_currents;
}

void levitation_update(
    const std::array<float, LCUConfig::ACTIVE_AIRGAP_COUNT>& input_airgaps,
    float reference,
    bool ramping
) {
    for (size_t i = 0; i < LCUConfig::ACTIVE_AIRGAP_COUNT; i++) {
        inputs.Sensores[i] = input_airgaps[i];
    }

    inputs.RefZ = reference;
    inputs.ManualLevitacin = 1.0;
    for (int i = 0; i < 10; i++)
        inputs.CorrienteManual[i] = 0.0;
    inputs.RampaStep = ramping ? 1.0 : 0.0;
    inputs.enable = ramping ? 1.0 : 0.0;

    model.setExternalInputs(&inputs);
    model.step1();
}

} // Namespace Control
