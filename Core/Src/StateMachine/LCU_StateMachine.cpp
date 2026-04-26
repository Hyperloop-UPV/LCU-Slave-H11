#include "StateMachine/LCU_StateMachine.hpp"
#include "LCU_SLAVE.hpp"

namespace LCU_SM {

bool request_global_fault_if_needed() {
    static bool reported = false;

    bool has_fault = false;
    const char* reason = nullptr;

    if (!LCU_Slave::lpu_array.is_all_ok()) {
        has_fault = true;
        reason = "LPU array fault detected";
    }

#ifdef USE_SPI_ERROR
    if (!has_fault && spi_error_counter && (*spi_error_counter >= LCU_Slave::MAX_SPI_ERRORS)) {
        has_fault = true;
        reason = "SPI error threshold exceeded";
    }
#endif

    if (!has_fault && LCU_Slave::master_fault_triggered) {
        has_fault = true;
        reason = "Master fault line asserted";
    }

    if (has_fault) {
        if (!reported) {
            FAULT(reason);
            reported = true;
        }
        return true;
    }

    reported = false;
    return false;
}

bool transition_connecting_to_idle() {
#ifdef USE_SPI_ERROR
    // Transition to IDLE if connection
    // is stable (counter is 0)
    return *spi_error_counter == 0;
#else
    return true;
#endif
}

bool transition_idle_to_levitating() {
    auto cmds = command_packet->flags;
    return  bool(cmds & CommandFlags::LEVITATE) ||
            bool(cmds & CommandFlags::CURRENT_CONTROL);
}

bool transition_levitating_to_idle() {
    auto cmds = command_packet->flags;
    bool stop_requested =   !bool(cmds & CommandFlags::LEVITATE) &&
                            !bool(cmds & CommandFlags::CURRENT_CONTROL);
    return stop_requested;
}

void on_fault_enter() {
    LCU_Slave::slave_fault.turn_off();
    LCU_Slave::led_fault.turn_on();
    Control::deinit();
    LCU_Slave::lpu_array.disable_all();
}

void on_levitate_enter() {
    LCU_Slave::led_operational.turn_on();
    Control::init();
    LCU_Slave::lpu_array.enable_all();
}

void on_levitate_exit() {
    LCU_Slave::led_operational.turn_off();
    Control::deinit();
    LCU_Slave::lpu_array.disable_all();
}

void cyclic_levitate_sensors() {
    LCU_Slave::airgap_array.update();
    LCU_Slave::lpu_array.update_all();
}

void cyclic_idle_sensors() {
    LCU_Slave::airgap_array.update();
    LCU_Slave::lpu_array.update_all();
}

void cyclic_levitate_control_current() {
    bool levitate_active = bool(command_packet->flags & CommandFlags::LEVITATE);
    bool direct_current_control = bool(command_packet->flags & CommandFlags::CURRENT_CONTROL);
    auto control_output = Control::current_update(
        direct_current_control ? std::optional<float>(command_packet->current_control.desired_current) : std::nullopt
    );
    uint16_t current_mask = command_packet->current_control.lpu_id_bitmask;
    bool apply_to_all_for_levitation = levitate_active;
    
#ifdef USE_1_DOF
    // 1-DOF: Single LPU
    if (apply_to_all_for_levitation || (current_mask & (1 << 0))) {
        LCU_Slave::lpu_array.get_lpu<0>().set_out_voltage(control_output.voltage);
    }
    
#elif defined(USE_5_DOF)
    // 5-DOF: Apply to all 10 LPUs as specified in bitmask
    if (apply_to_all_for_levitation || (current_mask & (1 << 0))) { LCU_Slave::lpu_array.get_lpu<0>().set_out_voltage(control_output.voltages[0]); }
    if (apply_to_all_for_levitation || (current_mask & (1 << 1))) { LCU_Slave::lpu_array.get_lpu<1>().set_out_voltage(control_output.voltages[1]); }
    if (apply_to_all_for_levitation || (current_mask & (1 << 2))) { LCU_Slave::lpu_array.get_lpu<2>().set_out_voltage(control_output.voltages[2]); }
    if (apply_to_all_for_levitation || (current_mask & (1 << 3))) { LCU_Slave::lpu_array.get_lpu<3>().set_out_voltage(control_output.voltages[3]); }
    if (apply_to_all_for_levitation || (current_mask & (1 << 4))) { LCU_Slave::lpu_array.get_lpu<4>().set_out_voltage(control_output.voltages[4]); }
    if (apply_to_all_for_levitation || (current_mask & (1 << 5))) { LCU_Slave::lpu_array.get_lpu<5>().set_out_voltage(control_output.voltages[5]); }
    if (apply_to_all_for_levitation || (current_mask & (1 << 6))) { LCU_Slave::lpu_array.get_lpu<6>().set_out_voltage(control_output.voltages[6]); }
    if (apply_to_all_for_levitation || (current_mask & (1 << 7))) { LCU_Slave::lpu_array.get_lpu<7>().set_out_voltage(control_output.voltages[7]); }
    if (apply_to_all_for_levitation || (current_mask & (1 << 8))) { LCU_Slave::lpu_array.get_lpu<8>().set_out_voltage(control_output.voltages[8]); }
    if (apply_to_all_for_levitation || (current_mask & (1 << 9))) { LCU_Slave::lpu_array.get_lpu<9>().set_out_voltage(control_output.voltages[9]); }
#endif
}

void cyclic_levitate_control_distance() {
    bool levitate_active = bool(command_packet->flags & CommandFlags::LEVITATE);
    bool direct_current_control = bool(command_packet->flags & CommandFlags::CURRENT_CONTROL);
    if (levitate_active && !direct_current_control) {
        Control::levitation_update(
            command_packet->levitate.desired_distance
        );
    }
}

void update() {
    if (FaultController::is_faulted()) {
        return;
    }

    if (request_global_fault_if_needed()) {
        return;
    }

    // General commands
    auto cmds = command_packet->flags;
    static bool was_enabled = false;
    if (bool(cmds & CommandFlags::ENABLE_LPU_BUFFER)) {
        uint16_t buffer_mask = command_packet->force_enable_lpu_buffer.lpu_buffer_id_bitmask;
        
#ifdef USE_1_DOF
        // 1-DOF: Single LPU pair
        if (buffer_mask & 0x03) { LCU_Slave::lpu_array.enable_pair<0>(); }
        else { LCU_Slave::lpu_array.disable_pair<0>(); }
        
#elif defined(USE_5_DOF)
        // 5-DOF: Enable LPU pairs based on bitmask
        // Each pair corresponds to 1 bit in the mask
        if (buffer_mask & 0x01) { LCU_Slave::lpu_array.enable_pair<0>(); }  // Pair 0 (LPU 0-1)
        else { LCU_Slave::lpu_array.disable_pair<0>(); }
         if (buffer_mask & 0x02) { LCU_Slave::lpu_array.enable_pair<1>(); }  // Pair 1 (LPU 2-3)
        else { LCU_Slave::lpu_array.disable_pair<1>(); }
        if (buffer_mask & 0x04) { LCU_Slave::lpu_array.enable_pair<2>(); }  // Pair 2 (LPU 4-5)
        else { LCU_Slave::lpu_array.disable_pair<2>(); }
        if (buffer_mask & 0x08) { LCU_Slave::lpu_array.enable_pair<3>(); }  // Pair 3 (LPU 6-7)
        else { LCU_Slave::lpu_array.disable_pair<3>(); }
        if (buffer_mask & 0x10) { LCU_Slave::lpu_array.enable_pair<4>(); }  // Pair 4 (LPU 8-9)
        else { LCU_Slave::lpu_array.disable_pair<4>(); }
#endif
        
        was_enabled = true;
    } else if (was_enabled) {
        if (sm_operational.get_current_state() != SlaveState::LEVITATING) {
            LCU_Slave::lpu_array.disable_all();
        }
        was_enabled = false;
    }
}

} // Namespace LCU_SM
