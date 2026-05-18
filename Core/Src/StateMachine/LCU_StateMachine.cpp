#include "StateMachine/LCU_StateMachine.hpp"
#include "LCU_SLAVE.hpp"

namespace LCU_SM {

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
    LCU_Slave::led_operational.turn_off();
    Control::deinit();
}

void on_idle_enter() {
}

void on_levitation_enter() {
    LCU_Slave::led_operational.turn_on();
    Control::init();
    LCU_Slave::lpu_array.enable_all();
}

void on_levitation_exit() {
    LCU_Slave::led_operational.turn_off();
    Control::deinit();
    LCU_Slave::lpu_array.disable_all();
}

void on_current_control_enter() {
    LCU_Slave::led_operational.turn_on();
    Control::init();
    LCU_Slave::lpu_array.enable_all();
}

void on_current_control_exit() {
    LCU_Slave::led_operational.turn_off();
    Control::deinit();
    LCU_Slave::lpu_array.disable_all();
}

void on_debug_enter() {
    LCU_Slave::led_operational.turn_on();
    LCU_Slave::lpu_array.enable_all();
}

void on_debug_exit() {
    LCU_Slave::led_operational.turn_off();
    LCU_Slave::lpu_array.disable_all();
}

void update_sensors() {
    LCU_Slave::airgap_array.update();
    LCU_Slave::lpu_array.update_all();
}

void cyclic_connecting() {
    LCU_Slave::led_operational.toggle();
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
