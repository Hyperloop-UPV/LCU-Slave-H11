#ifndef CONTROL_HPP
#define CONTROL_HPP

#include "C++Utilities/CppImports.hpp"
#include "C_5DOF_EL.h"
#include "Config/LCUHardwareConfig.hpp"
#include "ControlShared.hpp"

namespace Control {

inline C_5DOF_EL model{};
inline auto& output = model.getExternalOutputs();
inline C_5DOF_EL::ExtU_C_5DOF_EL_T inputs{};
inline ControlBase control{};

void init();
void deinit();

std::array<float, LCUConfig::ACTIVE_LPU_COUNT> current_update(
    const std::array<float, LCUConfig::ACTIVE_LPU_COUNT>& input_currents,
    std::optional<float> desired_current = std::nullopt
);

void levitation_update(
    const std::array<float, LCUConfig::ACTIVE_AIRGAP_COUNT>& input_airgaps,
    float reference,
    bool ramping = false
);

}; // namespace Control

#endif // CONTROL_HPP
