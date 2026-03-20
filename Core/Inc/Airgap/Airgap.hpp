#ifndef AIRGAP_HPP
#define AIRGAP_HPP

#include "C++Utilities/CppImports.hpp"
#include "AirgapShared.hpp"
#include "ST-LIB_LOW/Sensors/LinearSensor/LinearSensor.hpp"
#include "HALAL/Services/ADC/NewADC.hpp"

class Airgap : public AirgapBase {
    // MovingAverage<10> airgap_moving_avg;
    LinearSensor<volatile float> airgap_sensor;

public:
    Airgap(ST_LIB::ADCDomain::Instance& airgap_instance, float airgap_offset, float airgap_slope)
        : airgap_sensor(
              airgap_instance,
              airgap_slope,
              airgap_offset,
              &airgap_v /*,
              airgap_moving_avg*/
          ) {}

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

template <typename AirgapTuple> class AirgapArray;

template <typename... AirgapInstances>
class AirgapArray<std::tuple<AirgapInstances...>> {
    static constexpr size_t AirgapCount = sizeof...(AirgapInstances);

    using AirgapPtrTuple = std::tuple<std::remove_reference_t<AirgapInstances>*...>;

    AirgapPtrTuple airgap_instances;

public:
    AirgapArray(std::tuple<AirgapInstances&...> _instances) {
        airgap_instances =
            std::apply([](auto&... instance) { return std::make_tuple(&instance...); }, _instances);
    }

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

#endif // AIRGAP_HPP
