#ifndef CONTROL_HPP
#define CONTROL_HPP

#include "C++Utilities/CppImports.hpp"
#include "LCU_SLAVE_Types.hpp"

extern "C" {
#include "control.h"
}

namespace Control {
void init() { control_initialize(); }

float current_update() {
#ifdef USE_1_DOF
    control_U.corriente_real = LCU_Slave::g_lpu_array->get_lpu<0>().shunt_v;

    control_step0();

    return control_Y.Voltage;

#elif defined(USE_5_DOF)
    // (TODO)
    return 0.0f;
#endif
}

void levitation_update(float reference) {
#ifdef USE_1_DOF
    control_U.Gap = LCU_Slave::g_airgap_array->get_airgap<0>().airgap_v;
    control_U.Referencia = reference;

    control_step1();

#elif defined(USE_5_DOF)
    // (TODO)
#endif
}

void deinit() { control_terminate(); }
}; // namespace Control

#endif // CONTROL_HPP
