#ifndef LPU_HPP
#define LPU_HPP

#include "C++Utilities/CppImports.hpp"
#include "LPUShared.hpp"
#include "HALAL/Services/PWM/PWM.hpp"
#include "ST-LIB_LOW/Sensors/LinearSensor/FilteredLinearSensor.hpp"
#include "HALAL/Services/ADC/ADC.hpp"
#include "Control/Blocks/MovingAverage.hpp"

template <
    uint32_t ShuntMovingAverageSize,
    uint32_t VbatMovingAverageSize,
    typename PWMPositive,
    typename PWMNegative>
class LPU : public LPUBase {
public:
    LPU(PWMPositive& pwm_positive,
        PWMNegative& pwm_negative,
        ST_LIB::ADCDomain::Instance& adc_vbat_instance,
        ST_LIB::ADCDomain::Instance& adc_shunt_instance,
        float vbat_offset,
        float vbat_slope,
        float shunt_offset,
        float shunt_slope)
        : pwm_positive(pwm_positive), pwm_negative(pwm_negative), shunt_moving_avg(),
          vbat_moving_avg(),
          vbat_sensor(adc_vbat_instance, vbat_slope, vbat_offset, &vbat_v, vbat_moving_avg),
          shunt_sensor(adc_shunt_instance, shunt_slope, shunt_offset, &shunt_v, shunt_moving_avg) {}

    void init() {
        pwm_positive.set_timer_frequency(20'000);
        pwm_negative.set_timer_frequency(20'000);
    }

    void update() {
        if (is_fixed_vbat) {
            vbat_v = fixed_vbat;
        } else {
            vbat_sensor.read();
        }
        shunt_sensor.read();
    }

    /**
     * @brief Set the duty cycle based on the desired output voltage and the current battery voltage
     */
    void set_out_voltage(float voltage) {
        // Avoid division by zero
        if (vbat_v < 5.0f) {
            set_duty(0.0f);
            WARNING(
                "Battery voltage too low (%.2f V) for reliable operation. Output disabled.",
                vbat_v
            );
        }

        float duty = voltage / vbat_v * 100.0f;
        duty = std::clamp(duty, -100.0f, 100.0f);
        set_duty(duty);
    }

    void set_duty(float duty) {
        if (duty >= 0.0f) {
            pwm_negative.set_duty_cycle(0.0f);
            pwm_positive.set_duty_cycle(duty);
        } else {
            pwm_positive.set_duty_cycle(0.0f);
            pwm_negative.set_duty_cycle(-duty);
        }
        duty_cycle = duty;
    }

    void set_fixed_duty_cycle() {
        set_duty(fixed_duty_cycle);
    }

    void disable() {
        set_duty(0.0f);
        pwm_negative.turn_off();
        pwm_positive.turn_off();
    }

    void enable() {
        pwm_positive.turn_on();
        pwm_negative.turn_on();
    }

private:
    PWMPositive& pwm_positive;
    PWMNegative& pwm_negative;

    MovingAverage<ShuntMovingAverageSize> shunt_moving_avg;
    MovingAverage<VbatMovingAverageSize> vbat_moving_avg;
    FilteredLinearSensor<volatile float, VbatMovingAverageSize> vbat_sensor;
    FilteredLinearSensor<volatile float, ShuntMovingAverageSize> shunt_sensor;
};

template <typename LPUTuple, typename EnablePinTuple> class LpuArray;

template <typename... LPUs, typename... EnablePins>
class LpuArray<std::tuple<LPUs...>, std::tuple<EnablePins...>>
    : public LpuArrayBase<std::tuple<LPUs...>> {
    std::tuple<EnablePins...>& enable_pins;
    static constexpr auto LpuCount = sizeof...(LPUs);

public:
    LpuArray(std::tuple<LPUs...>& lpu_refs, std::tuple<EnablePins...>& pin_refs)
        : LpuArrayBase<std::tuple<LPUs...>>(lpu_refs), enable_pins(pin_refs) {}

    void init() {
        std::apply([](auto&... lpu) { (lpu.init(), ...); }, this->lpus);
        disable_all();
    }

    void enable_all() {
        std::apply([](auto&... pin) { (pin.turn_off(), ...); }, this->enable_pins);
        std::apply([](auto&... lpu) { (lpu.enable(), ...); }, this->lpus);
    }

    void disable_all() {
        std::apply([](auto&... pin) { (pin.turn_on(), ...); }, this->enable_pins);
        std::apply([](auto&... lpu) { (lpu.disable(), ...); }, this->lpus);
    }

    void update_all() {
        std::apply([&](auto&... lpu) { ((lpu.update()), ...); }, this->lpus);
    }

    auto get_shunt_readings() const {
        return std::apply(
            [](auto&... lpu) { return std::array<float, LpuCount>{lpu.shunt_v...}; },
            this->lpus
        );
    }

    void set_out_voltages(const std::array<float, LpuCount>& voltages) {
        std::apply(
            [&](auto&... lpu) {
                size_t idx = 0;
                ((lpu.set_out_voltage(voltages[idx++])), ...);
            },
            this->lpus
        );
    }

    void set_fixed_duty_cycle_all() {
        std::apply([&](auto&... lpu) { (lpu.set_fixed_duty_cycle(), ...); }, this->lpus);
    }
};

template <typename... LPUs, typename... EnablePins>
LpuArray(std::tuple<LPUs...>&, std::tuple<EnablePins...>&)
    -> LpuArray<std::tuple<LPUs...>, std::tuple<EnablePins...>>;

template <
    uint32_t ShuntMovingAverageSize,
    uint32_t VbatMovingAverageSize,
    typename PWMPositive,
    typename PWMNegative>
inline auto make_lpu(
    PWMPositive& pwm_positive,
    PWMNegative& pwm_negative,
    ST_LIB::ADCDomain::Instance& adc_vbat_instance,
    ST_LIB::ADCDomain::Instance& adc_shunt_instance,
    float vbat_offset,
    float vbat_slope,
    float shunt_offset,
    float shunt_slope
) {
    using PWMPositiveType = std::remove_reference_t<PWMPositive>;
    using PWMNegativeType = std::remove_reference_t<PWMNegative>;

    return LPU<ShuntMovingAverageSize, VbatMovingAverageSize, PWMPositiveType, PWMNegativeType>(
        pwm_positive,
        pwm_negative,
        adc_vbat_instance,
        adc_shunt_instance,
        vbat_offset,
        vbat_slope,
        shunt_offset,
        shunt_slope
    );
}

#endif // LPU_HPP
