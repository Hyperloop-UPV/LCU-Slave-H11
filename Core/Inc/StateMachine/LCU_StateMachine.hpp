#ifndef LCU_STATE_MACHINE_HPP
#define LCU_STATE_MACHINE_HPP

#include "ST-LIB_LOW/StateMachine/StateMachine.hpp"
#include "Control/Control.hpp"
#include "ConfigShared.hpp"

namespace LCU_SM {

void update();
void start();

inline StateMachineBase state_machine{};

bool transition_connecting_to_idle();
bool transition_idle_to_levitation();
bool transition_idle_to_current_control();
bool transition_idle_to_debug();
bool transition_levitation_to_idle();
bool transition_current_control_to_idle();
bool transition_debug_to_idle();

void on_fault_enter();
void on_idle_enter();
void on_levitation_enter();
void on_levitation_exit();
void on_current_control_enter();
void on_current_control_exit();
void on_debug_enter();
void on_debug_exit();

void cyclic_connecting();
void cyclic_idle_check_master_fault();
void cyclic_levitation_control_current();
void cyclic_levitation_control_distance();
void cyclic_current_control_current();
void cyclic_debug_fixed_pwm();

void update_sensors();

inline constexpr auto state_spi_connecting = make_state(
    SlaveState::SPI_CONNECTING,
    Transition{SlaveState::IDLE, transition_connecting_to_idle}
);

inline constexpr auto state_idle = make_state(
    SlaveState::IDLE,
    Transition{SlaveState::LEVITATION, transition_idle_to_levitation},
    Transition{SlaveState::CURRENT_CONTROL, transition_idle_to_current_control},
    Transition{SlaveState::DEBUG, transition_idle_to_debug}
);

inline constexpr auto state_levitation =
    make_state(SlaveState::LEVITATION, Transition{SlaveState::IDLE, transition_levitation_to_idle});

inline constexpr auto state_current_control = make_state(
    SlaveState::CURRENT_CONTROL,
    Transition{SlaveState::IDLE, transition_current_control_to_idle}
);

inline constexpr auto state_debug =
    make_state(SlaveState::DEBUG, Transition{SlaveState::IDLE, transition_debug_to_idle});

inline constinit auto sm_operational = []() consteval {
    auto sm = make_state_machine(
        SlaveState::SPI_CONNECTING,
        state_spi_connecting,
        state_idle,
        state_levitation,
        state_current_control,
        state_debug
    );

    using namespace std::chrono_literals;

    sm.add_cyclic_action(cyclic_connecting, 500ms, state_spi_connecting);

    sm.add_enter_action(on_idle_enter, state_idle);
    sm.add_cyclic_action(cyclic_idle_check_master_fault, 100ms, state_idle);

    sm.add_enter_action(on_levitation_enter, state_levitation);
    sm.add_exit_action(on_levitation_exit, state_levitation);
    sm.add_cyclic_action(cyclic_levitation_control_current, 500us, state_levitation);
    sm.add_cyclic_action(cyclic_levitation_control_distance, 1000us, state_levitation);

    sm.add_enter_action(on_current_control_enter, state_current_control);
    sm.add_exit_action(on_current_control_exit, state_current_control);
    sm.add_cyclic_action(cyclic_current_control_current, 500us, state_current_control);

    sm.add_enter_action(on_debug_enter, state_debug);
    sm.add_exit_action(on_debug_exit, state_debug);
    sm.add_cyclic_action(cyclic_debug_fixed_pwm, 1000us, state_debug);

    return sm;
}();

} // namespace LCU_SM

#endif // LCU_STATE_MACHINE_HPP
