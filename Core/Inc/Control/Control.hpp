#ifndef CONTROL_HPP
#define CONTROL_HPP

#include "C++Utilities/CppImports.hpp"

#ifdef USE_5_DOF
#include "CONTROLH10_1.h"
#endif

namespace Control {

struct ControlOutput {
#ifdef USE_1_DOF
    float voltage;
    float z1, z2, z3;
#elif defined(USE_5_DOF)
    std::array<float, 4> voltages;
#endif
};

void init();

ControlOutput current_update(std::optional<float> desired_current = std::nullopt);

void levitation_update(float reference);

inline void deinit() {
#ifdef USE_1_DOF
    control_terminate();
#elif defined(USE_5_DOF)
    // 5DOF control terminates via static instance destructor
#endif
}

}; // namespace Control

#endif // CONTROL_HPP
