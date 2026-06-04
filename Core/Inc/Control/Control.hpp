#ifndef CONTROL_HPP
#define CONTROL_HPP

#include "C++Utilities/CppImports.hpp"
extern "C" {
#include "control.h"
}
#include "Config/LCUHardwareConfig.hpp"
#include "ControlShared.hpp"

namespace Control {

constexpr float RAMP_RATE = 0.0002f;

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
