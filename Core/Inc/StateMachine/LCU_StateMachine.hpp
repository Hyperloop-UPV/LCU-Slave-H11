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
        },
        state_levitating
    );

    sm.add_exit_action(
        []() {
            LCU_Slave::g_led_operational->turn_off();
            Control::deinit();
            LCU_Slave::g_lpu_array->disable_all();
        },
        state_levitating
    );

    // Enter Fault: Safe State
    sm.add_enter_action(
        []() {
            LCU_Slave::g_slave_fault->turn_off();
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
            LCU_Slave::g_lpu_array->update_all();
            LCU_Slave::g_airgap->update();
        },
        100us,
        state_levitating
    );

    sm.add_cyclic_action(
        []() {
            LCU_Slave::g_lpu_array->update_all();
            LCU_Slave::g_airgap->update();
        },
        100us,
        state_idle
    );

    sm.add_cyclic_action(
        []() {
            auto target_voltage = Control::current_update(LCU_Slave::g_lpu->shunt_v);
            LCU_Slave::g_lpu->set_out_voltage(target_voltage);
        },
        200us,
        state_levitating
    );

    sm.add_cyclic_action(
        []() {
            if (bool(command_packet->flags & CommandFlags::LEVITATE)) {
                Control::levitation_update(
                    LCU_Slave::g_airgap->airgap_v,
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
    if (bool(cmds & CommandFlags::ENABLE_LPU_BUFFER)) { // (TODO) Distinguish the correct buffer, and disable somehow
        LCU_Slave::g_lpu_array->enable_all();
        was_enabled = true;
    } else if (was_enabled && sm_operational.get_current_state() != SlaveState::LEVITATING) {
        LCU_Slave::g_lpu_array->disable_all();
        was_enabled = false;
    }
}

} // namespace LCU_SM

#endif // LCU_STATE_MACHINE_HPP
