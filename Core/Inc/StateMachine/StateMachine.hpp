#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include "ST-LIB_LOW/StateMachine/StateMachine.hpp"
#include "LCU_SLAVE_Types.hpp"
#include "Control/Control.hpp"

// ============================================
// State Enums
// ============================================

enum class SystemStates : uint8_t {
    INITIAL = 0,
    IDLE = 1,
    OPERATIONAL = 2,
    FAULT = 3
};

enum class ControlStates : uint8_t {
    IDLE = 0,
    RUNNING = 1,
    STICK_DOWN = 2,
};

// ============================================
// Global State Machine Interface
// ============================================

namespace LCU_SM {
    // Control flags (set by Frame commands from Master)
    inline bool flag_enable = false;
    inline bool flag_start_running = false;
    inline bool flag_stick_down = false;
    inline bool flag_fault = false;
    
    // Control parameters
    inline float desired_gap_reference = 0.0f;

    // ============================================
    // Control State Machine Definition
    // ============================================
    static constexpr auto control_state_idle = make_state(ControlStates::IDLE,
        Transition{ControlStates::RUNNING, []() { return flag_start_running; }},
        Transition{ControlStates::STICK_DOWN, []() { return flag_stick_down; }}
    );

    static constexpr auto control_state_running = make_state(ControlStates::RUNNING,
        Transition{ControlStates::IDLE, []() { return !flag_start_running; }}
    );

    static constexpr auto control_state_stick_down = make_state(ControlStates::STICK_DOWN,
        Transition{ControlStates::IDLE, []() { return !flag_stick_down; }}
    );

    static constinit auto sm_control = []() consteval {
        auto sm = make_state_machine(
            ControlStates::IDLE,
            control_state_idle,
            control_state_running,
            control_state_stick_down
        );

        using namespace std::chrono_literals;

        // Enter RUNNING: initialize control
        sm.add_enter_action([]() {
            Control::init();
        }, control_state_running);

        // Exit RUNNING: deinitialize control
        sm.add_exit_action([]() {
            Control::deinit();
            if (LCU_Slave::g_lpu_array) LCU_Slave::g_lpu_array->disable_all_impl();
            flag_start_running = false;
        }, control_state_running);

        // Exit STICK_DOWN: cleanup
        sm.add_exit_action([]() {
            if (LCU_Slave::g_lpu_array) LCU_Slave::g_lpu_array->disable_all_impl();
            flag_stick_down = false;
        }, control_state_stick_down);

        // RUNNING: Execute cascaded control loops
        // Inner loop: Current control at 2 kHz
        sm.add_cyclic_action([]() {
            // TODO: Get measured current from LPU
            float measured_current = 0.0f; // LCU_Slave::g_lpu->get_current();
            Control::current_update(measured_current);
        }, 500us, control_state_running);

        // Outer loop: Levitation control at 1 kHz
        sm.add_cyclic_action([]() {
            // TODO: Get measured gap from airgap sensor
            float measured_gap = 0.0f; // LCU_Slave::g_airgap->get_gap();
            Control::levitation_update(measured_gap, desired_gap_reference);
        }, 1000us, control_state_running);

        // STICK_DOWN: Apply negative current to stick down
        sm.add_cyclic_action([]() {
            // TODO: Apply constant negative current
            // LCU_Slave::g_lpu_array->set_current(-50.0f);
        }, 1000us, control_state_stick_down);

        return sm;
    }();

    // ============================================
    // System State Machine Definition
    // ============================================
    static constexpr auto state_initial = make_state(SystemStates::INITIAL,
        Transition{SystemStates::IDLE, []() { return flag_enable; }}
    );

    static constexpr auto state_idle = make_state(SystemStates::IDLE,
        Transition{SystemStates::OPERATIONAL, []() { return flag_enable; }},
        Transition{SystemStates::FAULT, []() { return flag_fault; }}
    );

    static constexpr auto state_operational = make_state(SystemStates::OPERATIONAL,
        Transition{SystemStates::IDLE, []() { return !flag_enable; }},
        Transition{SystemStates::FAULT, []() { return flag_fault; }}
    );

    static constexpr auto state_fault = make_state(SystemStates::FAULT);

    static constinit auto sm_system = []() consteval {
        auto sm = make_state_machine(
            SystemStates::INITIAL,
            state_initial,
            state_idle,
            state_operational,
            state_fault
        );

        sm.add_enter_action([]() {
            if (LCU_Slave::g_lpu_array) LCU_Slave::g_lpu_array->enable_all_impl();
        }, state_operational);

        sm.add_enter_action([]() {
            if (LCU_Slave::g_lpu_array) LCU_Slave::g_lpu_array->disable_all_impl();
        }, state_fault);

        sm.add_exit_action([]() {
            if (LCU_Slave::g_lpu_array) LCU_Slave::g_lpu_array->disable_all_impl();
        }, state_operational);

        sm.add_state_machine(sm_control, state_operational);

        return sm;
    }();

    // ============================================
    // Public Interface
    // ============================================

    inline void start() {
        sm_system.start();
    }

    inline void update() {
        sm_system.check_transitions();
    }

    inline void set_enable(bool enable) { flag_enable = enable; }
    
    inline void start_levitation(float gap_reference) { 
        desired_gap_reference = gap_reference;
        flag_start_running = true; 
    }
    
    inline void stop_levitation() {
        flag_start_running = false;
    }
    
    inline void set_stick_down(bool active) { flag_stick_down = active; }
    inline void set_fault(bool fault) { flag_fault = fault; }
    
    inline void stop_control() {
        flag_start_running = false;
        flag_stick_down = false;
    }

    inline SystemStates get_system_state() { 
        return static_cast<SystemStates>(sm_system.get_current_state_id()); 
    }
    
    inline ControlStates get_control_state() { 
        return static_cast<ControlStates>(sm_control.get_current_state_id()); 
    }
}

#endif // STATE_MACHINE_HPP