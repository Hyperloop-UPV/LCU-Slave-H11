#ifndef LPU_HPP
#define LPU_HPP

#include "C++Utilities/CppImports.hpp"
#include "LPUShared.hpp"
#include "HALAL/Services/PWM/PWM.hpp"
#include "ST-LIB_LOW/Sensors/LinearSensor/FilteredLinearSensor.hpp"
#include "HALAL/Services/ADC/ADC.hpp"
#include "Control/Blocks/MovingAverage.hpp"

template <typename PWMPositive, typename PWMNegative, uint32_t ShuntMovingAverageSize, uint32_t VbatMovingAverageSize> class LPU : public LPUBase {
public:
    LPU(PWMPositive& pwm_positive,
        PWMNegative& pwm_negative,
        ST_LIB::ADCDomain::Instance& adc_vbat_instance,
        ST_LIB::ADCDomain::Instance& adc_shunt_instance,
        float vbat_offset,
        float vbat_slope,
        float shunt_offset,
        float shunt_slope)
        : pwm_positive(pwm_positive), pwm_negative(pwm_negative),
          shunt_moving_avg(), vbat_moving_avg(),
          vbat_sensor(adc_vbat_instance, vbat_slope, vbat_offset, &vbat_v, vbat_moving_avg),
          shunt_sensor(adc_shunt_instance, shunt_slope, shunt_offset, &shunt_v, shunt_moving_avg) {
        pwm_positive.turn_on();
        pwm_negative.turn_on();
    }

    bool update() {
        if (is_fixed_vbat) {
            vbat_v = fixed_vbat;
        } else {
            vbat_sensor.read();
        }
        shunt_sensor.read();

        if (is_fixed_duty_cycle) {
            set_duty(fixed_duty_cycle);
        }

        return true;
    }

    /**
     * @brief Set the duty cycle based on the desired output voltage and the current battery voltage
     */
    bool set_out_voltage(float voltage) {
        if (is_fixed_duty_cycle) {
            return true;
        }
        // Avoid division by zero, but this shouldn't happen I think?
        if (vbat_v < 0.1f) {
            set_duty(0.0f);
            return false;
        }

        float duty = voltage / vbat_v * 100.0f;
        duty = std::clamp(duty, -100.0f, 100.0f);
        set_duty(duty);
        return true;
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

    bool enable() {
        #ifdef USE_LPU_READY
        if (!ready)
            return false;
        #endif
        #ifdef USE_LPU_FAULT
        if (fault)
            return false;
        #endif
        is_enabled = true;
        return true;
    }

    bool disable() {
        if (is_fixed_duty_cycle) {
            return false;
        }
        is_enabled = false;
        return true;
    }

    void zeroing() {
        double vbat_sum = 0.0;
        double shunt_sum = 0.0;
        constexpr size_t sample_count = 1000;

        for (std::size_t i = 0; i < sample_count; i++) {
            vbat_sensor.read();
            shunt_sensor.read();
            vbat_sum += vbat_v;
            shunt_sum += shunt_v;
        }

        vbat_sensor.set_offset(vbat_sum / sample_count);
        shunt_sensor.set_offset(shunt_sum / sample_count);
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
class LpuArray<std::tuple<LPUs...>, std::tuple<EnablePins...>> {
    static constexpr size_t LpuCount = sizeof...(LPUs);
    static constexpr size_t PinCount = sizeof...(EnablePins);

    static_assert(
        LpuCount == PinCount * 2 || (LpuCount == 1 && PinCount == 1),
        "Configuration Error: Must have exactly 2 LPUs per Enable Pin or have only 1 LPU and 1 "
        "Enable Pin (1DOF)."
    );

    using LPUPtrTuple = std::tuple<std::remove_reference_t<LPUs>*...>;
    using PinPtrTuple = std::tuple<std::remove_reference_t<EnablePins>*...>;

    LPUPtrTuple lpus;
    PinPtrTuple enable_pins;

    bool all_ok = true;

public:
    LpuArray(std::tuple<LPUs&...> _lpus, std::tuple<EnablePins&...> _pins) {
        lpus = std::apply([](auto&... lpu) { return std::make_tuple(&lpu...); }, _lpus);
        enable_pins = std::apply([](auto&... pin) { return std::make_tuple(&pin...); }, _pins);
        disable_all();
    }

    void enable_all() {
        std::apply([](auto&... pin) { (pin->turn_off(), ...); }, enable_pins);
        std::apply([](auto*... lpu) { (lpu->enable(), ...); }, lpus);
    }

    void disable_all() {
        std::apply([](auto&... pin) { (pin->turn_on(), ...); }, enable_pins);
        std::apply([](auto*... lpu) { (lpu->disable(), ...); }, lpus);
    }

    void zeroing_all() {
        std::apply([](auto*... lpu) { (lpu->zeroing(), ...); }, lpus);
    }

    template <size_t PairIndex> void enable_pair() {
        if constexpr (LpuCount == 1) {
            std::get<0>(enable_pins)->turn_off();
            std::get<0>(lpus)->enable();
            return;
        }
        std::get<PairIndex>(enable_pins)->turn_off();

        std::get<PairIndex * 2>(lpus)->enable();
        std::get<PairIndex * 2 + 1>(lpus)->enable();
    }

    template <size_t PairIndex> void disable_pair() {
        if constexpr (LpuCount == 1) {
            std::get<0>(enable_pins)->turn_on();
            std::get<0>(lpus)->disable();
            return;
        }
        std::get<PairIndex>(enable_pins)->turn_on();

        std::get<PairIndex * 2>(lpus)->disable();
        std::get<PairIndex * 2 + 1>(lpus)->disable();
    }

    bool update_all() {
        all_ok = true;
        std::apply([&](auto&... lpu) { ((all_ok &= lpu->update()), ...); }, lpus);
        return all_ok;
    }

    bool is_all_ok() { return all_ok; }

    template <size_t Index> auto& get_lpu() { return *std::get<Index>(lpus); }
};

// Deduction guide for LpuArray
template <typename... LPUs, typename... EnablePins>
LpuArray(std::tuple<LPUs&...>, std::tuple<EnablePins&...>)
    -> LpuArray<std::tuple<LPUs...>, std::tuple<EnablePins...>>;

#endif // LPU_HPP
