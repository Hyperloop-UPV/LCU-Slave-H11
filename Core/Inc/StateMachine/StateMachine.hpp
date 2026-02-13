#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include "ST-LIB_LOW/StateMachine/StateMachine.hpp"
#include "LCU_SLAVE_Types.hpp"
#include "Control/Control.hpp"
#include "CommunicationsShared.hpp"

namespace LCU_SM {

    enum class OperationalState : uint8_t {
        IDLE = 0,
        LEVITATING = 1,
        FAULT = 2
    };

    inline volatile CommandPacket* command_packet = nullptr;

    inline void set_command_packet(volatile CommandPacket* ptr) {
        command_packet = ptr;
    }

    // ============================================
    // Operational State Machine
    // ============================================

    static constexpr auto state_idle = make_state(OperationalState::IDLE,
        Transition{OperationalState::LEVITATING, []() {
            if (!command_packet) return false;
            auto cmds = command_packet->commands;
            return (bool)((cmds & CommandFlags::LEVITATE) != CommandFlags::NONE);
        }}
    );

    static constexpr auto state_levitating = make_state(OperationalState::LEVITATING,
        Transition{OperationalState::IDLE, []() {
            auto cmds = command_packet->commands;
            bool stop_requested = (cmds & CommandFlags::STOP_LEVITATE) != CommandFlags::NONE;
            return stop_requested;
        }},
        Transition{OperationalState::FAULT, []() {
            return !LCU_Slave::g_lpu_array->is_all_ok();
        }}
    );
    
    static constexpr auto state_fault = make_state(OperationalState::FAULT);

    static constinit auto sm_operational = []() consteval {
        auto sm = make_state_machine(
            OperationalState::IDLE,
            state_idle,
            state_levitating,
            state_fault
        );

        using namespace std::chrono_literals;

        sm.add_enter_action([]() {
            LCU_Slave::g_led_fault->turn_on();
            Control::init();
            LCU_Slave::g_lpu_array->enable_all();
        }, state_levitating);

        sm.add_exit_action([]() {
            LCU_Slave::g_led_fault->turn_off();
            Control::deinit();
            LCU_Slave::g_lpu_array->disable_all();
        }, state_levitating);
        
        // Enter Fault: Safe State
        sm.add_enter_action([]() {
            LCU_Slave::g_led_fault->turn_on();
            Control::deinit();
            LCU_Slave::g_lpu_array->disable_all();
        }, state_fault);

        sm.add_cyclic_action([]() {
            LCU_Slave::g_lpu_array->update_all();
            LCU_Slave::g_airgap->update();
        }, 100us, state_levitating);

        // Current Control
        sm.add_cyclic_action([]() {
            Control::current_update(0.0f);
        }, 200us, state_levitating);

        // Levitation Control
        sm.add_cyclic_action([]() {
            Control::levitation_update(0.0f, 0.0f);
        }, 1000us, state_levitating);

        return sm;
    }();

    // ============================================
    // Public Interface
    // ============================================

    inline void start() {
        sm_operational.start();
    }

    inline void update() {
        sm_operational.check_transitions();
    }
}

#endif // STATE_MACHINE_HPP
