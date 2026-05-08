#ifndef CONTROL_HPP
#define CONTROL_HPP

#include "C++Utilities/CppImports.hpp"
#include "ControlTop/ControlTop.h"

#ifdef USE_1_DOF
#define CONTROL_LPU_COUNT 1
#define CONTROL_AIRGAP_COUNT 1
#elif defined(USE_5_DOF) // Manually overriden as 3dof right now
#define CONTROL_LPU_COUNT 4
#define CONTROL_AIRGAP_COUNT 4
#endif

namespace Control {

inline ControlTop model{};
inline auto& output = model.getExternalOutputs();
inline ControlTop::ExtU_ControlTop_T inputs{};

void init();
void deinit();

std::array<float, CONTROL_LPU_COUNT> current_update(
    std::array<float, CONTROL_LPU_COUNT> input_currents,
    std::optional<float> desired_current = std::nullopt
);

void levitation_update(
    std::array<float, CONTROL_AIRGAP_COUNT> input_airgaps,
    float reference,
    bool ramping = false
);

}; // namespace Control

#endif // CONTROL_HPP
