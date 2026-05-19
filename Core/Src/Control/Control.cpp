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
    for (int i = 0; i < 4; i++)
        ctrl.CorrienteReferencia[i] = output.CorrienteReferencia[i];
    for (int i = 0; i < 5; i++)
        ctrl.Estados[i] = output.Estados[i];
    for (int i = 0; i < 4; i++)
        ctrl.GapsLocales[i] = output.GapsLocales[i];
    for (int i = 0; i < 4; i++)
        ctrl.Voltages[i] = output.Voltages[i];
    ctrl.Referencia = output.Referencia;
    for (int i = 0; i < 3; i++)
        ctrl.Fe[i] = output.Fe[i];
    for (int i = 0; i < 4; i++)
        ctrl.Fa[i] = output.Fa;
    for (int i = 0; i < 3; i++)
        ctrl.Ef[i] = output.Ef[i];
    for (int i = 0; i < 3; i++)
        ctrl.P[i] = output.P[i];
    for (int i = 0; i < 3; i++)
        ctrl.R[i] = output.R[i];
    for (int i = 0; i < 3; i++)
        ctrl.Zz[i] = output.Zz[i];
    for (int i = 0; i < 3; i++)
        ctrl.Fe_L[i] = output.Fe_L;
    for (int i = 0; i < 8; i++)
        ctrl.A[i] = output.A[i];
    for (int i = 0; i < 4; i++)
        ctrl.Ak[i] = output.Ak[i];
    for (int i = 0; i < 3; i++)
        ctrl.Bk[i] = output.Bk[i];
}

std::array<float, LCUConfig::ACTIVE_LPU_COUNT> current_update(
    const std::array<float, LCUConfig::ACTIVE_LPU_COUNT>& input_currents,
    std::optional<float> desired_current
) {

    for (size_t i = 0; i < LCUConfig::ACTIVE_LPU_COUNT; i++) {
        inputs.I_HEMS[i] = input_currents[i];
    }

    if (desired_current.has_value()) {
        inputs.CorrienteManual = desired_current.value();
        inputs.ManualLevitacin = 0.0; // Disable levitation control, enable manual current control
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
    inputs.ManualLevitacin = 1.0;           // Enable levitation control
    inputs.CorrienteManual = 0.0;           // Disable manual current control
    inputs.RampaStep = ramping ? 1.0 : 0.0; // Set ramping flag
    inputs.enable = ramping ? 1.0 : 0.0;    // Enable ramping if requested

    model.setExternalInputs(&inputs);

    model.step1();
}

} // Namespace Control
