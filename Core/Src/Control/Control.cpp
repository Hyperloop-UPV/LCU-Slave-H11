#include "Control/Control.hpp"
#include "LCU_SLAVE.hpp"

namespace Control {

void init() { control_initialize(); }

void deinit() {
    control_terminate();
    control.output.clear();
}

static float ramp_current = 0.0f;

static void update_control() {
    auto& ctrl = control.output;
    ctrl.Voltages[0] = control_Y.Voltage;
    ctrl.Estados[0] = control_Y.z1;
    ctrl.Estados[1] = control_Y.z2;
    ctrl.Estados[2] = control_Y.z3;
    ctrl.Referencia = control_U.Referencia;
}

std::array<float, LCUConfig::ACTIVE_LPU_COUNT> current_update(
    const std::array<float, LCUConfig::ACTIVE_LPU_COUNT>& input_currents,
    std::optional<float> desired_current
) {
    if (desired_current.has_value()) {
        control_DW.RateTransition_Buffer0 = desired_current.value();
    }

    control_U.corriente_real = input_currents[0];

    control_step0();

    std::array<float, LCUConfig::ACTIVE_LPU_COUNT> output_currents{};
    output_currents[0] = static_cast<float>(control_Y.Voltage);

    update_control();

    return output_currents;
}

void levitation_update(
    const std::array<float, LCUConfig::ACTIVE_AIRGAP_COUNT>& input_airgaps,
    float reference,
    bool ramping
) {
    control_U.Gap = input_airgaps[0];

    if (ramping) {
        float error = reference - ramp_current;
        float step = RAMP_RATE;
        if (std::abs(error) <= step) {
            ramp_current = reference;
        } else {
            ramp_current += (error > 0.0f ? step : -step);
        }
    } else {
        ramp_current = reference;
    }
    control_U.Referencia = ramp_current;

    control_step1();
}

} // Namespace Control
