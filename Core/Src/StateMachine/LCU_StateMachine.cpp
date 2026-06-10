#include "StateMachine/LCU_StateMachine.hpp"
#include "LCU_SLAVE.hpp"
#include "Control/dpc_ai_controller.h"
#include "Timing/AppTiming.h"

namespace LCU_SM {

namespace {
constexpr float METERS_TO_MM = 1000.0f;
}

bool transition_connecting_to_idle() { return Communications::is_connected(); }

bool transition_idle_to_levitation() {
    return state_machine.desired_state == SlaveState::LEVITATION;
}

bool transition_idle_to_current_control() {
    return state_machine.desired_state == SlaveState::CURRENT_CONTROL;
}

bool transition_idle_to_debug() { return state_machine.desired_state == SlaveState::DEBUG; }

bool transition_levitation_to_idle() {
    return state_machine.desired_state != SlaveState::LEVITATION;
}

bool transition_current_control_to_idle() {
    return state_machine.desired_state != SlaveState::CURRENT_CONTROL;
}

bool transition_debug_to_idle() { return state_machine.desired_state != SlaveState::DEBUG; }

void on_fault_enter() {
    LCU_Slave::lpu_array.disable_all();
    LCU_Slave::slave_fault.turn_off();
    LCU_Slave::led_fault.turn_on();
    LCU_Slave::led_connected.turn_off();
    LCU_Slave::led_debug.turn_off();
    LCU_Slave::led_current_control.turn_off();
    LCU_Slave::led_levitation.turn_off();
    Control::deinit();
}

void on_idle_enter() {
    LCU_Slave::led_connected.turn_on();
}

void on_levitation_enter() {
    LCU_Slave::led_levitation.turn_on();
#if defined(USE_DPC_AI)
    update_sensors();
    const auto airgap_readings = LCU_Slave::airgap_array.get_readings();
    const auto shunt_readings = LCU_Slave::lpu_array.get_shunt_readings();
    dpc_ai_reset(airgap_readings[0] * METERS_TO_MM, shunt_readings[0]);
    Control::control.output.clear();
#else
    Control::init();
#endif
    LCU_Slave::lpu_array.enable_all();
}

void on_levitation_exit() {
    LCU_Slave::led_levitation.turn_off();
#if defined(USE_DPC_AI)
    Control::control.output.clear();
#else
    Control::deinit();
#endif
    LCU_Slave::lpu_array.disable_all();
}

void on_current_control_enter() {
    LCU_Slave::led_current_control.turn_on();
    Control::init();
    LCU_Slave::lpu_array.enable_all();
}

void on_current_control_exit() {
    LCU_Slave::led_current_control.turn_off();
    Control::deinit();
    LCU_Slave::lpu_array.disable_all();
}

void on_debug_enter() {
    LCU_Slave::led_debug.turn_on();
    LCU_Slave::lpu_array.enable_all();
}

void on_debug_exit() {
    LCU_Slave::led_debug.turn_off();
    LCU_Slave::lpu_array.disable_all();
}

void update_sensors() {
    const uint32_t start = app_timing_cycles();
    LCU_Slave::airgap_array.update();
    LCU_Slave::lpu_array.update_all();
    Control::control.output.Timing.sensor_update_us = app_timing_elapsed_cycles(start);
}

void cyclic_connecting() {
    LCU_Slave::led_connected.toggle();
    if (LCU_Slave::master_fault.read() == GPIO_PinState::GPIO_PIN_RESET) {
        FAULT("Master fault detected via GPIO during SPI connecting");
    }
}

void cyclic_idle_check_master_fault() {
    if (LCU_Slave::master_fault.read() == GPIO_PinState::GPIO_PIN_RESET) {
        FAULT("Master fault detected via GPIO");
        return;
    }
    if (state_machine.desired_state == SlaveState::FAULT) {
        FAULT("Master Fault detected via SPI");
        return;
    }
}

void cyclic_levitation_control_current() {
    auto shunt_readings = LCU_Slave::lpu_array.get_shunt_readings();
    auto control_output = Control::current_update(shunt_readings);
    LCU_Slave::lpu_array.set_out_voltages(control_output);
}

void cyclic_levitation_control_distance() {
    auto input_airgaps = LCU_Slave::airgap_array.get_readings();

    Control::levitation_update(
        input_airgaps,
        Control::control.input.RefZ,
        Control::control.input.ramping
    );
}

void cyclic_levitation_control_dpc() {
#if defined(USE_DPC_AI)
    static uint32_t last_start_cycles = 0U;
    const uint32_t start = app_timing_cycles();
    if (last_start_cycles != 0U) {
        const uint32_t period_cycles = start - last_start_cycles;
        if (period_cycles < (SystemCoreClock / 100U)) {
            app_timing_record_u32(
                &Control::control.output.Timing.dpc_period_us,
                &Control::control.output.Timing.dpc_period_max_us,
                period_cycles
            );
        } else {
            Control::control.output.Timing.dpc_period_us = 0U;
        }
    }
    last_start_cycles = start;

    const auto input_airgaps = LCU_Slave::airgap_array.get_readings();
    const auto shunt_readings = LCU_Slave::lpu_array.get_shunt_readings();
    const auto vbat_readings = LCU_Slave::lpu_array.get_vbat_readings();

    const float airgap_mm = input_airgaps[0] * METERS_TO_MM;
    const float target_mm = Control::control.input.RefZ * METERS_TO_MM;
    const dpc_ai_output_t dpc_output =
        dpc_ai_step(airgap_mm, target_mm, shunt_readings[0], vbat_readings[0]);
    Control::control.output.Timing.dpc_inference_us = dpc_ai_last_inference_us();
    Control::control.output.Timing.dpc_inference_max_us = dpc_ai_max_inference_us();

    std::array<float, LCUConfig::ACTIVE_LPU_COUNT> duties{};
    if (!dpc_output.ok) {
        LCU_Slave::lpu_array.set_duties(duties);
        app_timing_record_u32(
            &Control::control.output.Timing.dpc_total_us,
            &Control::control.output.Timing.dpc_total_max_us,
            app_timing_elapsed_cycles(start)
        );
        FAULT("DPC AI controller failed");
        return;
    }

    duties[0] = dpc_output.duty_percent;
    LCU_Slave::lpu_array.set_duties(duties);

    Control::control.output.Voltages[0] = dpc_output.voltage_v;
    Control::control.output.GapsLocales[0] = input_airgaps[0];
    Control::control.output.Estados[0] = dpc_output.duty_percent;
    Control::control.output.Referencia = Control::control.input.RefZ;
    app_timing_record_u32(
        &Control::control.output.Timing.dpc_total_us,
        &Control::control.output.Timing.dpc_total_max_us,
        app_timing_elapsed_cycles(start)
    );
#endif
}

void cyclic_current_control_current() {
    auto shunt_readings = LCU_Slave::lpu_array.get_shunt_readings();
    auto control_output =
        Control::current_update(shunt_readings, Control::control.input.RefCurrent);
    for (size_t i = 0; i < LCUConfig::ACTIVE_LPU_COUNT; i++) {
        if ((LCU_SM::state_machine.lpu_bitmask & (1 << i)) == 0) {
            control_output[i] = 0.0f; // Force zero output for LPUs not in current control
        }
    }
    LCU_Slave::lpu_array.set_out_voltages(control_output);
}

void cyclic_debug_fixed_pwm() {
    LCU_Slave::lpu_array.set_fixed_duty_cycle_all();
}

void start() { Scheduler::register_task(100, update_sensors); }

void update() {
    if (!Communications::is_connected() &&
        state_machine.current_state != SlaveState::SPI_CONNECTING) {
        FAULT("SPI connection lost");
    }
    if (FaultController::is_faulted()) {
        state_machine.current_state = SlaveState::FAULT;
        return;
    } else {
        state_machine.current_state = sm_operational.get_current_state();
    }
}

} // Namespace LCU_SM
