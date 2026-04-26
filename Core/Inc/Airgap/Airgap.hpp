#ifndef AIRGAP_HPP
#define AIRGAP_HPP

#include "C++Utilities/CppImports.hpp"
#include "AirgapShared.hpp"
#include "ST-LIB_LOW/Sensors/LinearSensor/LinearSensor.hpp"
#include "ST-LIB_LOW/Sensors/LinearSensor/FilteredLinearSensor.hpp"
#include "HALAL/Services/ADC/ADC.hpp"
#include "Control/Blocks/MovingAverage.hpp"

template <uint32_t MovingAverageSize>
class Airgap : public AirgapBase {
    MovingAverage<MovingAverageSize> airgap_moving_avg;
    FilteredLinearSensor<volatile float, MovingAverageSize> airgap_sensor;

public:
    Airgap(ST_LIB::ADCDomain::Instance& airgap_instance, float airgap_offset, float airgap_slope)
        : airgap_moving_avg(),
          airgap_sensor(airgap_instance, airgap_slope, airgap_offset, &airgap_v, airgap_moving_avg) {}

    void update() { airgap_sensor.read(); }

    void zeroing() {
        double airgap_sum = 0.0;
        constexpr size_t sample_count = 1000;

        for (std::size_t i = 0; i < sample_count; i++) {
            airgap_sensor.read();
            airgap_sum += airgap_v;
        }

        airgap_sensor.set_offset(airgap_sum / sample_count);
    }
};

template <typename... AirgapInstances>
class AirgapArray {
    static constexpr size_t AirgapCount = sizeof...(AirgapInstances);

    using AirgapPtrTuple = std::tuple<std::remove_reference_t<AirgapInstances>*...>;

    AirgapPtrTuple airgap_instances;

public:
    explicit AirgapArray(AirgapInstances&... instances)
        : airgap_instances(std::make_tuple(&instances...)) {}

    void update() {
        std::apply([](auto*... instance) { (instance->update(), ...); }, airgap_instances);
    }

    void zeroing() {
        std::apply([](auto*... instance) { (instance->zeroing(), ...); }, airgap_instances);
    }

    template <size_t Index> auto& get_airgap() {
        return *std::get<Index>(airgap_instances);
    }
};

// Deduction guide for direct construction from references.
template <typename... AirgapInstances>
AirgapArray(AirgapInstances&...) -> AirgapArray<AirgapInstances...>;

#endif // AIRGAP_HPP
