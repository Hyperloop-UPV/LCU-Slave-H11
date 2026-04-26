#ifndef LCU_STATE_MACHINE_HPP
#define LCU_STATE_MACHINE_HPP

#include "ST-LIB_LOW/StateMachine/StateMachine.hpp"
#include "Control/Control.hpp"
#include "CommunicationsShared.hpp"

namespace LCU_SM {

/**
 * Public API
 */
void update();

// (TODO) Clean this up

inline volatile CommandPacket* command_packet = nullptr;
inline volatile StatusPacket* status_packet = nullptr;
#ifdef USE_SPI_ERROR
inline volatile uint32_t* spi_error_counter = nullptr;
#endif

inline void set_command_packet(volatile CommandPacket* ptr) { command_packet = ptr; }
#ifdef USE_SPI_ERROR
inline void set_spi_error_counter_ptr(volatile uint32_t* ptr) { spi_error_counter = ptr; }
#endif

// End of cleanup todo


/**
 * State actions and transition guards
 */

bool request_global_fault_if_needed();

bool transition_connecting_to_idle();
bool transition_idle_to_levitating();
bool transition_levitating_to_idle();

void on_fault_enter();
void on_levitate_enter();
void on_levitate_exit();

void cyclic_levitate_sensors();
void cyclic_levitate_control_current();
void cyclic_levitate_control_distance();
void cyclic_idle_sensors();

/**
 * State Machine definition
 */

static constexpr auto state_spi_connecting = make_state(
    SlaveState::SPI_CONNECTING,
    Transition{SlaveState::IDLE, transition_connecting_to_idle}
);

static constexpr auto state_idle = make_state(
    SlaveState::IDLE,
    Transition{SlaveState::LEVITATING, transition_idle_to_levitating}
);

static constexpr auto state_levitating = make_state(
    SlaveState::LEVITATING,
    Transition{SlaveState::IDLE, transition_levitating_to_idle}
);

static constinit auto sm_operational = []() consteval {
    auto sm = make_state_machine(
        SlaveState::SPI_CONNECTING,
        state_spi_connecting,
        state_idle,
        state_levitating
    );

    using namespace std::chrono_literals;

    // Idle state
    sm.add_cyclic_action(cyclic_idle_sensors, 1ms, state_idle);

    // Levitating state
    sm.add_enter_action(on_levitate_enter, state_levitating);
    sm.add_exit_action(on_levitate_exit, state_levitating);
    sm.add_cyclic_action(cyclic_levitate_sensors, 100us, state_levitating);
    sm.add_cyclic_action(cyclic_levitate_control_current, 500us, state_levitating);
    sm.add_cyclic_action(cyclic_levitate_control_distance, 1000us, state_levitating);

    return sm;
}();

} // namespace LCU_SM

#endif // LCU_STATE_MACHINE_HPP
