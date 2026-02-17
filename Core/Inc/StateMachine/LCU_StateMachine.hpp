#ifndef LCU_STATE_MACHINE_HPP
#define LCU_STATE_MACHINE_HPP

#include "ST-LIB_LOW/StateMachine/StateMachine.hpp"
#include "LCU_SLAVE_Types.hpp"
#include "Control/Control.hpp"
#include "CommunicationsShared.hpp"

namespace LCU_SM {

enum class OperationalState : uint8_t { SPI_CONNECTING = 0, IDLE = 1, LEVITATING = 2, FAULT = 3 };

inline volatile CommandPacket* command_packet = nullptr;
inline volatile uint32_t* spi_error_counter = nullptr;

inline void set_command_packet(volatile CommandPacket* ptr) { command_packet = ptr; }
inline void set_spi_error_counter_ptr(volatile uint32_t* ptr) { spi_error_counter = ptr; }

// ============================================
// Operational State Machine
// ============================================

static constexpr auto state_spi_connecting = make_state(
    OperationalState::SPI_CONNECTING,
    Transition{
        OperationalState::IDLE,
        []() {
            if (!command_packet || !spi_error_counter)
                return false;
            // Transition to IDLE if connection
            // is stable (counter is 0)
            return *spi_error_counter == 0;
        }
    },
    Transition{OperationalState::FAULT, []() { return LCU_Slave::master_fault_triggered; }}
);

static constexpr auto state_idle = make_state(
    OperationalState::IDLE,
    Transition{
        OperationalState::LEVITATING,
        []() {
            if (!command_packet)
                return false;
            auto cmds = command_packet->flags;
            return (bool)((cmds & CommandFlags::LEVITATE) != CommandFlags::NONE);
        }
    },
    Transition{
        OperationalState::FAULT,
        []() {
            // Go to fault if too many errors
            bool spi_fault = false; //spi_error_counter && (*spi_error_counter >= LCU_Slave::MAX_SPI_ERRORS);
            return spi_fault || LCU_Slave::master_fault_triggered;
        }
    }
);

static constexpr auto state_levitating = make_state(
    OperationalState::LEVITATING,
    Transition{
        OperationalState::IDLE,
        []() {
            auto cmds = command_packet->flags;
            bool stop_requested = (cmds & CommandFlags::LEVITATE) == CommandFlags::NONE;
            return stop_requested;
        }
    },
    Transition{
        OperationalState::FAULT,
        []() {
            bool hardware_fault = !LCU_Slave::g_lpu_array->is_all_ok();
            // bool comms_fault =
            //     spi_error_counter && (*spi_error_counter >= LCU_Slave::MAX_SPI_ERRORS);
            return hardware_fault || /*comms_fault ||*/ LCU_Slave::master_fault_triggered;
        }
    }
);

static constexpr auto state_fault = make_state(OperationalState::FAULT);

static constinit auto sm_operational = []() consteval {
    auto sm = make_state_machine(
        OperationalState::SPI_CONNECTING,
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
            LCU_Slave::g_slave_fault->turn_on();
            LCU_Slave::g_led_fault->turn_on();
            Control::deinit();
            LCU_Slave::g_lpu_array->disable_all();
            ErrorHandler("Entered Fault State");
            while (1);
        },
        state_fault
    );

    sm.add_cyclic_action(
        []() {
            LCU_Slave::g_lpu_array->update_all();
            LCU_Slave::g_airgap->update();
        },
        100us,
        state_levitating
    );

    // Current Control
    sm.add_cyclic_action([]() { Control::current_update(LCU_Slave::g_lpu->shunt_v); }, 200us, state_levitating);

    // Levitation Control
    sm.add_cyclic_action(
        []() { Control::levitation_update(LCU_Slave::g_airgap->airgap_v, command_packet->levitate.desired_distance); },
        1000us,
        state_levitating
    );

    return sm;
}();

// ============================================
// Public Interface
// ============================================

inline void start() { sm_operational.start(); }

inline void update() { sm_operational.check_transitions(); }
} // namespace LCU_SM

#endif // LCU_STATE_MACHINE_HPP
