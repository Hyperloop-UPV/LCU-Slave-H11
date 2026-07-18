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
    for (int i = 0; i < 10; i++)
        ctrl.CorrienteReferencia[i] = static_cast<float>(output.CorrienteReferencia[i]);
    ctrl.Referencia = static_cast<float>(output.Referencia);
    for (int i = 0; i < 8; i++)
        ctrl.A[i] = static_cast<float>(output.A[i]);

    // auto& dw = model.get_corriente().get_dw();
    // ctrl.PI_Integrator_HEMS[0] = static_cast<float>(dw.Integrator_DSTATE);
    // ctrl.PI_Integrator_HEMS[1] = static_cast<float>(dw.Integrator_DSTATE_e);
    // ctrl.PI_Integrator_HEMS[2] = static_cast<float>(dw.Integrator_DSTATE_o);
    // ctrl.PI_Integrator_HEMS[3] = static_cast<float>(dw.Integrator_DSTATE_m);
    // ctrl.PI_Integrator_EMS[0] = static_cast<float>(dw.Integrator_DSTATE_a);
    // ctrl.PI_Integrator_EMS[1] = static_cast<float>(dw.Integrator_DSTATE_f);
    // ctrl.PI_Integrator_EMS[2] = static_cast<float>(dw.Integrator_DSTATE_j);
    // ctrl.PI_Integrator_EMS[3] = static_cast<float>(dw.Integrator_DSTATE_k);
    // ctrl.PI_Integrator_EMS[4] = static_cast<float>(dw.Integrator_DSTATE_b);
    // ctrl.PI_Integrator_EMS[5] = static_cast<float>(dw.Integrator_DSTATE_j2);
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
    inputs.Ganancia_HEMS = 2;
    inputs.Ganancia_EMS = 1;
    inputs.Vibra = false;
    for (int i = 0; i < 10; i++)
        inputs.CorrienteManual[i] = 0.0;
    inputs.RampaStep = ramping ? 1.0 : 0.0;
    inputs.enable = ramping ? 1.0 : 0.0;

    model.setExternalInputs(&inputs);
    model.step1();
}

} // Namespace Control
