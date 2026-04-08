#ifndef LCU_STATE_MACHINE_HPP
#define LCU_STATE_MACHINE_HPP

#include "C++Utilities/CppImports.hpp"
#include "LCU_SLAVE_Types.hpp"
#include "Control/Control.hpp"
#include "ControlShared.hpp"

// Forward declare Communications to avoid circular dependency
namespace Communications {
    extern ControlBase control;
    extern uint16_t fixed_pwm_active;
    bool is_spi_connected();
}

namespace LCU_SM {

// ============================================
// State Definitions
// ============================================

enum class GeneralState : uint8_t {
    Connecting = 0,
    Operational = 1,
    Fault = 2
};

enum class OperationalState : uint8_t {
    Idle = 0,
    Levitating = 1,
    Current_Control = 2,
    Debug = 3
};

// ============================================
// Helper Functions
// ============================================

// Check if buffers should be active based on control packet
static inline bool should_buffers_be_active() {
    auto& ctrl = Communications::control.control_packet;
    return  ctrl.mode == ControlMode::DISTANCE_CONTROL || 
            ctrl.mode == ControlMode::CURRENT_CONTROL ||
            Communications::fixed_pwm_active != 0;
}

// Check if slave is connected to master
static inline bool is_connected() {
    return Communications::is_spi_connected();
}

// ============================================
// Outer State Machine States
// ============================================

static constexpr auto connecting_state = make_state(
    GeneralState::Connecting,
    Transition<GeneralState>{
        GeneralState::Operational,
        []() { return is_connected(); }
    },
    Transition<GeneralState>{
        GeneralState::Fault,
        []() { return LCU_Slave::master_fault_triggered; }
    }
);

static constexpr auto operational_state = make_state(
    GeneralState::Operational,
    Transition<GeneralState>{
        GeneralState::Fault,
        []() {
            return !is_connected() || 
                   LCU_Slave::master_fault_triggered ||
                   !LCU_Slave::g_lpu_array->is_all_ok();
        }
    }
);

static constexpr auto fault_state = make_state(GeneralState::Fault);

// ============================================
// Nested Operational State Machine States
// ============================================

static constexpr auto nested_idle_state = make_state(
    OperationalState::Idle,
    Transition<OperationalState>{
        OperationalState::Levitating,
        []() { 
            return Communications::control.control_packet.mode == ControlMode::DISTANCE_CONTROL;
        }
    },
    Transition<OperationalState>{
        OperationalState::Current_Control,
        []() {
            return Communications::control.control_packet.mode == ControlMode::CURRENT_CONTROL;
        }
    },
    Transition<OperationalState>{
        OperationalState::Debug,
        []() { return Communications::fixed_pwm_active != 0; }
    }
);

static constexpr auto nested_levitating_state = make_state(
    OperationalState::Levitating,
    Transition<OperationalState>{
        OperationalState::Idle,
        []() {
            auto& ctrl = Communications::control.control_packet;
            return ctrl.mode != ControlMode::DISTANCE_CONTROL ||
                   !LCU_Slave::g_lpu_array->is_all_ok();
        }
    }
);

static constexpr auto nested_current_control_state = make_state(
    OperationalState::Current_Control,
    Transition<OperationalState>{
        OperationalState::Idle,
        []() {
            auto& ctrl = Communications::control.control_packet;
            return ctrl.mode == ControlMode::NONE ||
                   !LCU_Slave::g_lpu_array->is_all_ok();
        }
    }
);

static constexpr auto nested_debug_state = make_state(
    OperationalState::Debug,
    Transition<OperationalState>{
        OperationalState::Idle,
        []() { return Communications::fixed_pwm_active == 0 || !LCU_Slave::g_lpu_array->is_all_ok(); }
    }
);

// ============================================
// Nested State Machine for Operational
// ============================================

static inline constinit auto operational_state_machine = []() consteval {
    auto sm = make_state_machine(
        OperationalState::Idle,
        nested_idle_state,
        nested_levitating_state,
        nested_current_control_state,
        nested_debug_state
    );

    using namespace std::chrono_literals;

    // Enter Levitating: Enable LPUs and start control
    sm.add_enter_action(
        []() {
            LCU_Slave::g_led_operational->turn_on();
            Control::init();
            LCU_Slave::g_lpu_array->enable_all();
        },
        nested_levitating_state
    );

    // Exit Levitating: Disable LPUs and stop control
    sm.add_exit_action(
        []() {
            LCU_Slave::g_led_operational->turn_off();
            Control::deinit();
            LCU_Slave::g_lpu_array->disable_all();
        },
        nested_levitating_state
    );

    // Enter Current Control: Enable LPUs
    sm.add_enter_action(
        []() {
            Control::init();
            LCU_Slave::g_lpu_array->enable_all();
        },
        nested_current_control_state
    );

    // Exit Current Control: Disable LPUs
    sm.add_exit_action(
        []() {
            LCU_Slave::g_lpu_array->disable_all();
            Control::deinit();
        },
        nested_current_control_state
    );

    // Enter Debug: Enable LPUs
    sm.add_enter_action(
        []() { LCU_Slave::g_lpu_array->enable_all(); },
        nested_debug_state
    );

    // Exit Debug: Disable LPUs
    sm.add_exit_action(
        []() { LCU_Slave::g_lpu_array->disable_all(); },
        nested_debug_state
    );

    // Cyclic actions for Levitating
    sm.add_cyclic_action(
        []() {
            // Current control
            auto target_voltage = Control::current_update();
#ifdef USE_1_DOF
            LCU_Slave::g_lpu_array->get_lpu<0>().set_out_voltage(target_voltage);
#elif defined(USE_5_DOF)
            // TODO
#endif
        },
        500us,
        nested_levitating_state
    );

    sm.add_cyclic_action(
        []() {
            // Distance control
            Control::levitation_update(Communications::control.control_packet.distance_control.desired_distance);
        },
        1000us,
        nested_levitating_state
    );

    // Cyclic actions for Current Control
    sm.add_cyclic_action(
        []() {
            auto target_voltage = Control::current_update(
                Communications::control.control_packet.current_control.desired_current,
                true
            );

#ifdef USE_1_DOF
            LCU_Slave::g_lpu_array->get_lpu<0>().set_out_voltage(target_voltage);
#elif defined(USE_5_DOF)
            // TODO
#endif
        },
        500us,
        nested_current_control_state
    );

    return sm;
}();

// ============================================
// General State Machine (Outer)
// ============================================

static inline constinit auto general_state_machine = []() consteval {
    auto nested = StateMachineHelper::add_nested_machines(
        StateMachineHelper::add_nesting(operational_state, operational_state_machine)
    );
    auto sm = make_state_machine(
        GeneralState::Connecting,
        nested,
        connecting_state,
        operational_state,
        fault_state
    );

    using namespace std::chrono_literals;

    sm.add_enter_action(
        []() { LCU_Slave::g_led_operational->turn_on(); },
        operational_state
    );

    sm.add_exit_action(
        []() { LCU_Slave::g_led_operational->turn_off(); },
        operational_state
    );

    sm.add_enter_action(
        []() {
            LCU_Slave::g_slave_fault->turn_on();
            LCU_Slave::g_led_fault->turn_on();
            LCU_Slave::g_lpu_array->disable_all();
            ErrorHandler("Entered Fault State");
        },
        fault_state
    );

    sm.add_exit_action(
        []() { LCU_Slave::g_led_fault->turn_off(); },
        fault_state
    );

    sm.add_cyclic_action(
        []() {
            LCU_Slave::g_lpu_array->update_all();
            LCU_Slave::g_airgap_array->update();
        },
        100us,
        operational_state
    );

    return sm;
}();

// ============================================
// Public Interface
// ============================================

inline void start() {
    general_state_machine.start();
}

inline void update() {
    general_state_machine.check_transitions();
}

} // namespace LCU_SM

#endif // LCU_STATE_MACHINE_HPP
