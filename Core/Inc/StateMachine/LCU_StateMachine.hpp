#ifndef LCU_STATE_MACHINE_HPP
#define LCU_STATE_MACHINE_HPP

#include "ST-LIB_LOW/StateMachine/StateMachine.hpp"
#include "LCU_SLAVE_Types.hpp"
#include "Control/Control.hpp"
#include "CommunicationsShared.hpp"

namespace LCU_SM {

inline volatile CommandPacket* command_packet = nullptr;
inline volatile StatusPacket* status_packet = nullptr;
#ifdef USE_SPI_ERROR
inline volatile uint32_t* spi_error_counter = nullptr;
#endif

inline void set_command_packet(volatile CommandPacket* ptr) { command_packet = ptr; }
#ifdef USE_SPI_ERROR
inline void set_spi_error_counter_ptr(volatile uint32_t* ptr) { spi_error_counter = ptr; }
#endif

// ============================================
// Operational State Machine
// ============================================

auto check_fault = []() {
    return !LCU_Slave::g_lpu_array->is_all_ok() ||
#ifdef USE_SPI_ERROR
            (spi_error_counter && (*spi_error_counter >= LCU_Slave::MAX_SPI_ERRORS)) ||
#endif
            LCU_Slave::master_fault_triggered;
};

static constexpr auto state_spi_connecting = make_state(
    SlaveState::SPI_CONNECTING,
    Transition{
        SlaveState::IDLE,
        []() {
#ifdef USE_SPI_ERROR
            // Transition to IDLE if connection
            // is stable (counter is 0)
            return *spi_error_counter == 0;
#else
            return true;
#endif
        }
    },
    Transition{SlaveState::FAULT, []() { return LCU_Slave::master_fault_triggered; }}
);

static constexpr auto state_idle = make_state(
    SlaveState::IDLE,
    Transition{
        SlaveState::LEVITATING,
        []() {
            auto cmds = command_packet->flags;
            return  bool(cmds & CommandFlags::LEVITATE) ||
                    bool(cmds & CommandFlags::CURRENT_CONTROL);
        }
    },
    Transition{
        SlaveState::FAULT,
        check_fault
    }
);

static constexpr auto state_levitating = make_state(
    SlaveState::LEVITATING,
    Transition{
        SlaveState::IDLE,
        []() {
            auto cmds = command_packet->flags;
            bool stop_requested =   !bool(cmds & CommandFlags::LEVITATE) &&
                                    !bool(cmds & CommandFlags::CURRENT_CONTROL);
            return stop_requested;
        }
    },
    Transition{
        SlaveState::FAULT,
        check_fault
    }
);

static constexpr auto state_fault = make_state(SlaveState::FAULT);

    uint32_t task_id = 0;
static constinit auto sm_operational = []() consteval {
    auto sm = make_state_machine(
        SlaveState::SPI_CONNECTING,
        state_spi_connecting,
        state_idle,
        state_levitating,
        state_fault
    );

    using namespace std::chrono_literals;



    sm.add_enter_action(
        []() {
            LCU_Slave::g_led_operational->turn_on();
            Control::init();
            LCU_Slave::g_lpu_array->enable_all();
            task_id = Scheduler::register_task(
                100,
                []() {
                    LCU_Slave::g_lpu_array->update_all();
                    LCU_Slave::g_airgap_array->update();
                }
            );
        },
        state_levitating
    );

    sm.add_exit_action(
        []() {
            LCU_Slave::g_led_operational->turn_off();
            Control::deinit();
            LCU_Slave::g_lpu_array->disable_all();
            Scheduler::unregister_task(task_id);
        },
        state_levitating
    );

    // Enter Fault: Safe State
    sm.add_enter_action(
        []() {
            LCU_Slave::g_slave_fault->turn_on();
            LCU_Slave::g_led_fault->turn_on();
            Control::deinit();
            LCU_Slave::g_lpu_array->disable_all();
            ErrorHandler("Entered Fault State");
            // while (1)
            //     ;
        },
        state_fault
    );

    // Levitation Control

    sm.add_cyclic_action(
        []() {
            auto target_voltage = Control::current_update();
            uint16_t current_mask = command_packet->current_control.lpu_id_bitmask;
            
#ifdef USE_1_DOF
            // 1-DOF: Single LPU
            if (current_mask & (1 << 0)) { LCU_Slave::g_lpu_array->get_lpu<0>().set_out_voltage(target_voltage); }
            
#elif defined(USE_5_DOF)
            // 5-DOF: Apply to all 10 LPUs as specified in bitmask
            if (current_mask & (1 << 0)) { LCU_Slave::g_lpu_array->get_lpu<0>().set_out_voltage(target_voltage); }
            if (current_mask & (1 << 1)) { LCU_Slave::g_lpu_array->get_lpu<1>().set_out_voltage(target_voltage); }
            if (current_mask & (1 << 2)) { LCU_Slave::g_lpu_array->get_lpu<2>().set_out_voltage(target_voltage); }
            if (current_mask & (1 << 3)) { LCU_Slave::g_lpu_array->get_lpu<3>().set_out_voltage(target_voltage); }
            if (current_mask & (1 << 4)) { LCU_Slave::g_lpu_array->get_lpu<4>().set_out_voltage(target_voltage); }
            if (current_mask & (1 << 5)) { LCU_Slave::g_lpu_array->get_lpu<5>().set_out_voltage(target_voltage); }
            if (current_mask & (1 << 6)) { LCU_Slave::g_lpu_array->get_lpu<6>().set_out_voltage(target_voltage); }
            if (current_mask & (1 << 7)) { LCU_Slave::g_lpu_array->get_lpu<7>().set_out_voltage(target_voltage); }
            if (current_mask & (1 << 8)) { LCU_Slave::g_lpu_array->get_lpu<8>().set_out_voltage(target_voltage); }
            if (current_mask & (1 << 9)) { LCU_Slave::g_lpu_array->get_lpu<9>().set_out_voltage(target_voltage); }
#endif
        },
        200us,
        state_levitating
    );

    sm.add_cyclic_action(
        []() {
            if (bool(command_packet->flags & CommandFlags::LEVITATE)) {
                Control::levitation_update(
                    command_packet->levitate.desired_distance
                );
            }
        },
        1000us,
        state_levitating
    );

    return sm;
}();

// ============================================
// Public Interface
// ============================================

inline void start() { sm_operational.start(); }

inline void update() {
    sm_operational.check_transitions();

    // General commands
    auto cmds = command_packet->flags;
    static bool was_enabled = false;
    if (bool(cmds & CommandFlags::ENABLE_LPU_BUFFER)) {
        uint16_t buffer_mask = command_packet->force_enable_lpu_buffer.lpu_buffer_id_bitmask;
        
#ifdef USE_1_DOF
        // 1-DOF: Single LPU pair
        if (buffer_mask & 0x03) { LCU_Slave::g_lpu_array->enable_pair<0>(); }
        else { LCU_Slave::g_lpu_array->get_lpu<0>().disable(); }
        
#elif defined(USE_5_DOF)
        // 5-DOF: Enable LPU pairs based on bitmask
        // Each pair corresponds to bits in the mask (0-1 -> pair 0, 2-3 -> pair 1, etc.)
        if (buffer_mask & 0x03) { LCU_Slave::g_lpu_array->enable_pair<0>(); }  // Pair 0 (LPU 0-1)
        else { LCU_Slave::g_lpu_array->get_lpu<0>().disable(); }
        if (buffer_mask & 0x0C) { LCU_Slave::g_lpu_array->enable_pair<1>(); }  // Pair 1 (LPU 2-3)
        else { LCU_Slave::g_lpu_array->get_lpu<1>().disable(); }
        if (buffer_mask & 0x30) { LCU_Slave::g_lpu_array->enable_pair<2>(); }  // Pair 2 (LPU 4-5)
        else { LCU_Slave::g_lpu_array->get_lpu<2>().disable(); }
        if (buffer_mask & 0xC0) { LCU_Slave::g_lpu_array->enable_pair<3>(); }  // Pair 3 (LPU 6-7)
        else { LCU_Slave::g_lpu_array->get_lpu<3>().disable(); }
        if (buffer_mask & 0x300) { LCU_Slave::g_lpu_array->enable_pair<4>(); }  // Pair 4 (LPU 8-9)
        else { LCU_Slave::g_lpu_array->get_lpu<4>().disable(); }
#endif
        
        was_enabled = true;
    } else if (was_enabled) {
        if (sm_operational.get_current_state() == SlaveState::LEVITATING) {
            LCU_Slave::g_lpu_array->disable_all();
        } else {
            LCU_Slave::g_lpu_array->disable_all();
        }
        was_enabled = false;
    }
}

} // namespace LCU_SM

#endif // LCU_STATE_MACHINE_HPP
