#ifndef CONTROL_HPP
#define CONTROL_HPP

#include "C++Utilities/CppImports.hpp"

#ifdef USE_1_DOF
extern "C" {
#include "control.h"
}
#endif

#ifdef USE_5_DOF
#include "C_5DOF_EL.h"
#endif

namespace Control {

#ifdef USE_5_DOF
inline C_5DOF_EL control{};
inline C_5DOF_EL::ExtU_C_5DOF_EL_T inputs{};
#endif

struct ControlOutput {
#ifdef USE_1_DOF
    float voltage;
    float z1, z2, z3;
#elif defined(USE_5_DOF)
    std::array<float, 10> voltages;
#endif
};

inline void init() {
#ifdef USE_1_DOF
    control_initialize();
#elif defined(USE_5_DOF)
    control.initialize();
    control.setExternalInputs(&inputs);
#endif
}

ControlOutput current_update(std::optional<float> desired_current = std::nullopt);

void levitation_update(float reference);

inline void deinit() {
#ifdef USE_1_DOF
    control_terminate();
#elif defined(USE_5_DOF)
    control.terminate();
#endif
}

}; // namespace Control

#endif // CONTROL_HPP
