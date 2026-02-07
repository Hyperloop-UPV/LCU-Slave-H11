#ifndef LPU_HPP
#define LPU_HPP

#include "C++Utilities/CppImports.hpp"
#include "LPUShared.hpp"
#include "HALAL/Services/PWM/PWM.hpp"
#include "ST-LIB_LOW/Sensors/LinearSensor/LinearSensor.hpp"

template <typename PWMPositive, typename PWMNegative>
class LPU : public LPUBase {
   public:
    LPU(PWMPositive& pwm_positive, PWMNegative& pwm_negative,
        Pin &shunt_pin, Pin &vbat_pin, float vbat_offset, float vbat_slope, float shunt_offset, float shunt_slope) :
            pwm_positive(pwm_positive),
            pwm_negative(pwm_negative) {
        vbat_sensor = new LinearSensor<float>(vbat_pin, vbat_slope, vbat_offset, &vbat_v);
        shunt_sensor = new LinearSensor<float>(shunt_pin, shunt_slope, shunt_offset, &shunt_v);
    }

    void update() {
        if (state == State::Fault) return;

        if (fault) {
            disable();
            state = State::Fault;
            return;
        }

        if (state == State::Idle || state == State::Ready) {
            if (ready) {
                state = State::Ready;
                return;
            } else {
                state = State::Idle;
                return;
            }
        }
        vbat_sensor->read();
        shunt_sensor->read();
    }

    /**
     * @brief Set the duty cycle based on the desired output voltage and the current battery voltage (vbat_v).
     */
    bool set_out_voltage(float voltage) {
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
    }

    bool enable() {
        if (!ready) return false;
        if (fault) return false;
        pwm_positive.turn_on();
        pwm_negative.turn_on();
        state = State::Running;
        return true;
    }

    bool disable() {
        pwm_positive.turn_off();
        pwm_negative.turn_off();
        if (state == State::Running) {
            if (fault) state = State::Fault;
            else if (ready) state = State::Ready;
            else state = State::Idle;
        } else {
            return false;
        }
        return true;
    }

    void zeroing() {
        double vbat_sum = 0.0;
        double shunt_sum = 0.0;
        constexpr size_t sample_count = 1000;

        for (std::size_t i = 0; i < sample_count; i++) {
            vbat_sensor->read();
            shunt_sensor->read();
            vbat_sum += vbat_v;
            shunt_sum += shunt_v;
        }

        vbat_sensor->set_offset(vbat_sum / sample_count);
        shunt_sensor->set_offset(shunt_sum / sample_count);
    }

   private:
    PWMPositive& pwm_positive;
    PWMNegative& pwm_negative;
    LinearSensor <float> *vbat_sensor;
    LinearSensor <float> *shunt_sensor;
};

template <typename LPUTuple, typename EnablePinTuple>
class LpuArray;

template <typename... LPUs, typename... EnablePins>
class LpuArray<std::tuple<LPUs...>, std::tuple<EnablePins...>> {    
    static constexpr size_t LpuCount = sizeof...(LPUs);
    static constexpr size_t PinCount = sizeof...(EnablePins);

    static_assert(LpuCount == PinCount * 2 || (LpuCount == 1 && PinCount == 1), "Configuration Error: Must have exactly 2 LPUs per Enable Pin or have only 1 LPU and 1 Enable Pin (1DOF).");

    using LPUPtrTuple = std::tuple<std::remove_reference_t<LPUs>*...>;
    using PinPtrTuple = std::tuple<std::remove_reference_t<EnablePins>*...>;

    LPUPtrTuple lpus;
    PinPtrTuple enable_pins;

   public:
    LpuArray(std::tuple<LPUs&...> _lpus, std::tuple<EnablePins&...> _pins) {
        lpus = std::apply([](auto&... lpu) { return std::make_tuple(&lpu...); }, _lpus);
        enable_pins = std::apply([](auto&... pin) { return std::make_tuple(&pin...); }, _pins);
    }

    void enable_all_impl() {
        std::apply([](auto&... pin) { (pin->turn_on(), ...); }, enable_pins);
        std::apply([](auto*... lpu) { (lpu->enable(), ...); }, lpus);
    }

    void disable_all_impl() {
        std::apply([](auto&... pin) { (pin->turn_off(), ...); }, enable_pins);
        std::apply([](auto*... lpu) { (lpu->disable(), ...); }, lpus);
    }

    void zeroing_all_impl() {
        std::apply([](auto*... lpu) { (lpu->zeroing(), ...); }, lpus);
    }

    template <size_t LpuIndex>
    void enable_pair() {
        if constexpr (LpuCount == 1) {
            std::get<0>(enable_pins)->turn_on();
            std::get<0>(lpus)->enable();
            return;
        }
        constexpr size_t PinIndex = LpuIndex / 2;
        std::get<PinIndex>(enable_pins)->turn_on();

        std::get<PinIndex * 2>(lpus)->enable();
        std::get<PinIndex * 2 + 1>(lpus)->enable();
    }

    template <size_t Index>
    auto& get_lpu() {
        return *std::get<Index>(lpus);
    }
};

// Deduction guide for LpuArray
template <typename... LPUs, typename... EnablePins>
LpuArray(std::tuple<LPUs&...>, std::tuple<EnablePins&...>) -> LpuArray<std::tuple<LPUs...>, std::tuple<EnablePins...>>;

#endif // LPU_HPP