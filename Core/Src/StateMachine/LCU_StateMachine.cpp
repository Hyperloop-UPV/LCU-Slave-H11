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
    return *spi_error_counter == 0;
#else
    return true;
#endif
}

bool transition_idle_to_levitating() {
    auto cmds = command_packet->flags;
    return bool(cmds & CommandFlags::LEVITATE) || bool(cmds & CommandFlags::CURRENT_CONTROL);
}

bool transition_levitating_to_idle() {
    auto cmds = command_packet->flags;
    bool stop_requested =
        !bool(cmds & CommandFlags::LEVITATE) && !bool(cmds & CommandFlags::CURRENT_CONTROL);
    return stop_requested;
}

void on_fault_enter() {
    LCU_Slave::slave_fault.turn_off();
    LCU_Slave::led_fault.turn_on();
    LCU_Slave::led_operational.turn_off();
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

        status_packet->desired_current1 = 0.0f;
        status_packet->desired_current2 = 0.0f;
        status_packet->desired_current3 = 0.0f;
        status_packet->desired_current4 = 0.0f;

    status_packet->state0 = 0.0f;
    status_packet->state1 = 0.0f;
    status_packet->state2 = 0.0f;
    status_packet->state3 = 0.0f;
    status_packet->state4 = 0.0f;

    status_packet->airgap_local_1 = 0.0f;
    status_packet->airgap_local_2 = 0.0f;
    status_packet->airgap_local_3 = 0.0f;
    status_packet->airgap_local_4 = 0.0f;

    status_packet->desired_voltage_1 = 0.0f;
    status_packet->desired_voltage_2 = 0.0f;
    status_packet->desired_voltage_3 = 0.0f;
    status_packet->desired_voltage_4 = 0.0f;

    status_packet->target_distance = 0.0f;
    status_packet->Fe[0] = 0.0f;
    status_packet->Fe[1] = 0.0f;
    status_packet->Fe[2] = 0.0f;
    status_packet->Fa[0] = 0.0f;
    status_packet->Fa[1] = 0.0f;
    status_packet->Fa[2] = 0.0f;
    status_packet->Fa[3] = 0.0f;
    status_packet->Ef[0] = 0.0f;
    status_packet->Ef[1] = 0.0f;
    status_packet->Ef[2] = 0.0f;
    status_packet->P[0] = 0.0f;
    status_packet->P[1] = 0.0f;
    status_packet->P[2] = 0.0f;
    status_packet->R[0] = 0.0f;
    status_packet->R[1] = 0.0f;
    status_packet->R[2] = 0.0f;
    status_packet->Zz[0] = 0.0f;
    status_packet->Zz[1] = 0.0f;
    status_packet->Zz[2] = 0.0f;
    status_packet->Fe_L[0] = 0.0f;
    status_packet->Fe_L[1] = 0.0f;
    status_packet->Fe_L[2] = 0.0f;
    status_packet->A[0] = 0.0f;
    status_packet->A[1] = 0.0f;
    status_packet->A[2] = 0.0f;
    status_packet->A[3] = 0.0f;
    status_packet->A[4] = 0.0f;
    status_packet->A[5] = 0.0f;
    status_packet->A[6] = 0.0f;
    status_packet->A[7] = 0.0f;
    status_packet->Ak[0] = 0.0f;
    status_packet->Ak[1] = 0.0f;
    status_packet->Ak[2] = 0.0f;
    status_packet->Ak[3] = 0.0f;
    status_packet->Bk[0] = 0.0f;
    status_packet->Bk[1] = 0.0f;
    status_packet->Bk[2] = 0.0f;
}

void update_sensors() {
    LCU_Slave::airgap_array.update();
    LCU_Slave::lpu_array.update_all();
}

void check_master_fault() {
    if (LCU_Slave::master_fault.read() == GPIO_PinState::GPIO_PIN_RESET) {
        FAULT("Master fault detected via GPIO");
    }
}

void cyclic_levitate_control_current() {
    bool levitate_active = bool(command_packet->flags & CommandFlags::LEVITATE);
    bool direct_current_control = bool(command_packet->flags & CommandFlags::CURRENT_CONTROL);
    auto control_output = Control::current_update(
        {LCU_Slave::lpu_array.get_lpu<0>().shunt_v,
         LCU_Slave::lpu_array.get_lpu<1>().shunt_v,
         LCU_Slave::lpu_array.get_lpu<2>().shunt_v,
         LCU_Slave::lpu_array.get_lpu<3>().shunt_v},
        direct_current_control
            ? std::optional<float>(command_packet->current_control.desired_current)
            : std::nullopt
    );
    uint16_t current_mask = command_packet->current_control.lpu_id_bitmask;
    bool apply_to_all_for_levitation = levitate_active;

    // 5-DOF: Apply to all 10 LPUs as specified in bitmask
    if (apply_to_all_for_levitation || (current_mask & (1 << 0))) {
        LCU_Slave::lpu_array.get_lpu<0>().set_out_voltage(control_output[0]);
    }
    if (apply_to_all_for_levitation || (current_mask & (1 << 1))) {
        LCU_Slave::lpu_array.get_lpu<1>().set_out_voltage(control_output[1]);
    }
    if (apply_to_all_for_levitation || (current_mask & (1 << 2))) {
        LCU_Slave::lpu_array.get_lpu<2>().set_out_voltage(control_output[2]);
    }
    if (apply_to_all_for_levitation || (current_mask & (1 << 3))) {
        LCU_Slave::lpu_array.get_lpu<3>().set_out_voltage(control_output[3]);
    }

    status_packet->desired_current1 = Control::output.CorrienteReferencia[0];
    status_packet->desired_current2 = Control::output.CorrienteReferencia[1];
    status_packet->desired_current3 = Control::output.CorrienteReferencia[2];
    status_packet->desired_current4 = Control::output.CorrienteReferencia[3];

    status_packet->state0 = Control::output.Estados[0];
    status_packet->state1 = Control::output.Estados[1];
    status_packet->state2 = Control::output.Estados[2];
    status_packet->state3 = Control::output.Estados[3];
    status_packet->state4 = Control::output.Estados[4];

    status_packet->airgap_local_1 = Control::output.GapsLocales[0];
    status_packet->airgap_local_2 = Control::output.GapsLocales[1];
    status_packet->airgap_local_3 = Control::output.GapsLocales[2];
    status_packet->airgap_local_4 = Control::output.GapsLocales[3];

    status_packet->desired_voltage_1 = Control::output.Voltages[0];
    status_packet->desired_voltage_2 = Control::output.Voltages[1];
    status_packet->desired_voltage_3 = Control::output.Voltages[2];
    status_packet->desired_voltage_4 = Control::output.Voltages[3];

    status_packet->target_distance = Control::output.Referencia;

    status_packet->Fe[0] = Control::output.Fe[0];
    status_packet->Fe[1] = Control::output.Fe[1];
    status_packet->Fe[2] = Control::output.Fe[2];
    status_packet->Fa[0] = Control::output.Fa[0];
    status_packet->Fa[1] = Control::output.Fa[1];
    status_packet->Fa[2] = Control::output.Fa[2];
    status_packet->Fa[3] = Control::output.Fa[3];
    status_packet->Ef[0] = Control::output.Ef[0];
    status_packet->Ef[1] = Control::output.Ef[1];
    status_packet->Ef[2] = Control::output.Ef[2];
    status_packet->P[0] = Control::output.P[0];
    status_packet->P[1] = Control::output.P[1];
    status_packet->P[2] = Control::output.P[2];
    status_packet->R[0] = Control::output.R[0];
    status_packet->R[1] = Control::output.R[1];
    status_packet->R[2] = Control::output.R[2];
    status_packet->Zz[0] = Control::output.Zz[0];
    status_packet->Zz[1] = Control::output.Zz[1];
    status_packet->Zz[2] = Control::output.Zz[2];
    status_packet->Fe_L[0] = Control::output.Fe_L[0];
    status_packet->Fe_L[1] = Control::output.Fe_L[1];
    status_packet->Fe_L[2] = Control::output.Fe_L[2];
    status_packet->A[0] = Control::output.A[0];
    status_packet->A[1] = Control::output.A[1];
    status_packet->A[2] = Control::output.A[2];
    status_packet->A[3] = Control::output.A[3];
    status_packet->A[4] = Control::output.A[4];
    status_packet->A[5] = Control::output.A[5];
    status_packet->A[6] = Control::output.A[6];
    status_packet->A[7] = Control::output.A[7];
    status_packet->Ak[0] = Control::output.Ak[0];
    status_packet->Ak[1] = Control::output.Ak[1];
    status_packet->Ak[2] = Control::output.Ak[2];
    status_packet->Ak[3] = Control::output.Ak[3];
    status_packet->Bk[0] = Control::output.Bk[0];
    status_packet->Bk[1] = Control::output.Bk[1];
    status_packet->Bk[2] = Control::output.Bk[2];
}

void cyclic_levitate_control_distance() {
    bool levitate_active = bool(command_packet->flags & CommandFlags::LEVITATE);
    bool direct_current_control = bool(command_packet->flags & CommandFlags::CURRENT_CONTROL);
    if (levitate_active && !direct_current_control) {
        auto input_airgaps = LCU_Slave::airgap_array.get_readings();

        Control::levitation_update(
            input_airgaps,
            command_packet->levitate.desired_distance,
            command_packet->levitate.ramping
        );
    }
}

void start() {
    Scheduler::register_task(100, update_sensors);
    check_master_fault_id = Scheduler::register_task(10000, check_master_fault);
}

void update() {
    if (FaultController::is_faulted()) {
        return;
    }

    if (request_global_fault_if_needed()) {
        return;
    }

    status_packet->slave_state = sm_operational.get_current_state();

    auto cmds = command_packet->flags;
    if (bool(cmds & CommandFlags::ENABLE_LPU_BUFFER)) {
        LCU_Slave::lpu_array.enable_all();
    } else if (sm_operational.get_current_state() != SlaveState::LEVITATING) {
        LCU_Slave::lpu_array.disable_all();
    }
}

} // Namespace LCU_SM
