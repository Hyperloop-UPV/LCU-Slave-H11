#include "Control/Control.hpp"
#include "LCU_SLAVE.hpp"

namespace Control {

void init() { model.initialize(); }

void deinit() {
    model.terminate();
    control.output.clear();
}

void update_control() {
    const auto& v = output.V_h;
    auto& ctrl = control.output;
    ctrl.Voltages[0] = static_cast<float>(v.V_HEMS1);
    ctrl.Voltages[1] = static_cast<float>(v.V_HEMS2);
    ctrl.Voltages[2] = static_cast<float>(v.V_HEMS3);
    ctrl.Voltages[3] = static_cast<float>(v.V_HEMS4);
    ctrl.Voltages[4] = static_cast<float>(v.V_EMS5);
    ctrl.Voltages[5] = static_cast<float>(v.V_EMS6);
    ctrl.Voltages[6] = static_cast<float>(v.V_EMS7);
    ctrl.Voltages[7] = static_cast<float>(v.V_EMS8);
    ctrl.Voltages[8] = static_cast<float>(v.V_EMS9);
    ctrl.Voltages[9] = static_cast<float>(v.V_EMS10);
}

std::array<float, LCUConfig::ACTIVE_LPU_COUNT> current_update(
    const std::array<float, LCUConfig::ACTIVE_LPU_COUNT>& input_currents,
    std::optional<float> desired_current
) {
    auto& hems = inputs.I_HEMS;
    hems.I_HEMS1 = input_currents[0];
    hems.I_HEMS2 = input_currents[4];
    hems.I_HEMS3 = input_currents[5];
    hems.I_HEMS4 = input_currents[6];

    auto& ems = inputs.I_EMS;
    ems.I_EMS5  = input_currents[1];
    ems.I_EMS6  = input_currents[2];
    ems.I_EMS7  = input_currents[3];
    ems.I_EMS8  = input_currents[7];
    ems.I_EMS9  = input_currents[8];
    ems.I_EMS10 = input_currents[9];

    if (desired_current.has_value()) {
        model.manual_current = desired_current.value();
        model.manual_current_active = true;
    } else {
        model.manual_current_active = false;
    }

    model.setExternalInputs(&inputs);
    model.step0();

    std::array<float, LCUConfig::ACTIVE_LPU_COUNT> output_currents{};
    const auto& v = output.V_h;
    output_currents[0] = static_cast<float>(v.V_HEMS1);
    output_currents[4] = static_cast<float>(v.V_HEMS2);
    output_currents[5] = static_cast<float>(v.V_HEMS3);
    output_currents[6] = static_cast<float>(v.V_HEMS4);
    output_currents[1] = static_cast<float>(v.V_EMS5);
    output_currents[2] = static_cast<float>(v.V_EMS6);
    output_currents[3] = static_cast<float>(v.V_EMS7);
    output_currents[7] = static_cast<float>(v.V_EMS8);
    output_currents[8] = static_cast<float>(v.V_EMS9);
    output_currents[9] = static_cast<float>(v.V_EMS10);

    update_control();

    return output_currents;
}

void levitation_update(
    const std::array<float, LCUConfig::ACTIVE_AIRGAP_COUNT>& input_airgaps,
    float reference,
    bool ramping
) {
    auto& s = inputs.SensoresPos;
    s.AG_Z1 = input_airgaps[0];
    s.AG_Z2 = input_airgaps[1];
    s.AG_Z3 = input_airgaps[2];
    s.AG_Z4 = input_airgaps[3];
    s.AG_Y5 = input_airgaps[4];
    s.AG_Y6 = input_airgaps[5];
    s.AG_Y7 = input_airgaps[6];
    s.AG_Y8 = input_airgaps[7];

    inputs.RefZ = reference;
    (void)ramping;

    model.manual_current_active = false;

    model.setExternalInputs(&inputs);
    model.step1();
}

} // Namespace Control
