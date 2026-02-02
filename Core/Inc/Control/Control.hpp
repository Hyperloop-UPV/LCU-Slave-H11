#ifndef CONTROL_HPP
#define CONTROL_HPP

#include "control.h"

namespace Control {
   public:
    void init() {
        control_initialize();
    }

    void current_update() { // 2 kHz for now at least
        control_U.corriente_real = /* set current value */;

        control_step0();

        control_Y.Voltage; // Use output
    }

    void levitation_update() { // 1 kHz for now at least
        control_U.Gap = /* set gap value */;
        control_U.Referencia = /* set reference value */;

        control_step1();
    }

    void deinit() {
        control_terminate();
    }
};

#endif // CONTROL_HPP