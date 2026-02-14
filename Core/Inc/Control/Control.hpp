#ifndef CONTROL_HPP
#define CONTROL_HPP

#include "C++Utilities/CppImports.hpp"

extern "C" {
#include "control.h"
}

namespace Control {
void init() { control_initialize(); }

float current_update(float measured_current) { // 2 kHz for now at least
    control_U.corriente_real = measured_current;

    control_step0();

    return control_Y.Voltage; // Use output
}

void levitation_update(float gap, float reference) { // 1 kHz for now at least
    control_U.Gap = gap;
    control_U.Referencia = reference;

    control_step1();
}

void deinit() { control_terminate(); }
}; // namespace Control

#endif // CONTROL_HPP
