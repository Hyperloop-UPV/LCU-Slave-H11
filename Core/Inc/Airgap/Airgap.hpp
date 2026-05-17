#ifndef AIRGAP_HPP
#define AIRGAP_HPP

#include "C++Utilities/CppImports.hpp"
#include "AirgapShared.hpp"
#include "ST-LIB_LOW/Sensors/LinearSensor/LinearSensor.hpp"
#include "ST-LIB_LOW/Sensors/LinearSensor/FilteredLinearSensor.hpp"
#include "HALAL/Services/ADC/ADC.hpp"
#include "Control/Blocks/MovingAverage.hpp"

template <uint32_t MovingAverageSize> class Airgap : public AirgapBase {
    MovingAverage<MovingAverageSize> airgap_moving_avg;
    FilteredLinearSensor<volatile float, MovingAverageSize> airgap_sensor;

public:
    Airgap(ST_LIB::ADCDomain::Instance& airgap_instance, float airgap_offset, float airgap_slope)
        : airgap_moving_avg(), airgap_sensor(
                                   airgap_instance,
                                   airgap_slope,
                                   airgap_offset,
                                   &airgap_v,
                                   airgap_moving_avg
                               ) {}

    void update() { airgap_sensor.read(); }
};

template <typename AirgapTuple> class AirgapArray;

template <typename... AirgapInstances>
class AirgapArray<std::tuple<AirgapInstances...>>
    : public AirgapArrayBase<std::tuple<AirgapInstances...>> {
public:
    explicit AirgapArray(std::tuple<AirgapInstances...>& instance_refs)
        : AirgapArrayBase<std::tuple<AirgapInstances...>>(instance_refs) {}

    void update() {
        std::apply([](auto&... instance) { (instance.update(), ...); }, this->airgaps);
    }

    auto get_readings() const {
        return std::apply(
            [this](auto&... instance) {
                return std::array<float, this->count>{instance.airgap_v...};
            },
            this->airgaps
        );
    }
};

template <typename... AirgapInstances>
AirgapArray(std::tuple<AirgapInstances...>&) -> AirgapArray<std::tuple<AirgapInstances...>>;

#endif // AIRGAP_HPP
