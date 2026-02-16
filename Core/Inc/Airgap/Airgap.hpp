#ifndef AIRGAP_HPP
#define AIRGAP_HPP

#include "C++Utilities/CppImports.hpp"
#include "AirgapShared.hpp"
#include "ST-LIB_LOW/Sensors/LinearSensor/LinearSensor.hpp"
#include "HALAL/Services/ADC/NewADC.hpp"

class Airgap : public AirgapBase {
    MovingAverage<10> airgap_moving_avg;
    FilteredLinearSensor<float, 10> airgap_sensor;

public:
    Airgap(ST_LIB::ADCDomain::Instance& airgap_instance, float airgap_offset, float airgap_slope) :
        airgap_sensor(airgap_instance, airgap_slope, airgap_offset, &airgap_v, airgap_moving_avg) {
    }

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

#endif // AIRGAP_HPP
