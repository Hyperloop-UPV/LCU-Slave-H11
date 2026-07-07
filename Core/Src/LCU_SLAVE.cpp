#include "LCU_SLAVE.hpp"

namespace LCU_Slave {

void init() {
    Board::init();
    AppProtectionEngine::initialize();

    slave_fault.turn_on();
    spi.set_software_nss(false);
    slave_ready.turn_off();

    lpu_array.init();

    MDMA::start();

    Communications::init();

    LCU_SM::start();

    Frame::init(
        lpu_array,
        airgap_array,
        LCU_SM::state_machine,
        Control::control,
        Communications::report
    );

    Watchdog::watchdog_time = std::chrono::milliseconds(1);
    Watchdog::start();
}

void update() {
    Communications::update();
    FaultController::check_transitions();
    LCU_SM::update();
    Scheduler::update();
    MDMA::update();
    AppProtectionEngine::evaluate();
    Diagnostics::Hub::flush();
    if (reset_counter >= 5) {
        HAL_Delay(100); // Delay in case more faults are coming in rapidly
        HAL_NVIC_SystemReset();
    }
    Watchdog::refresh();
}

} // Namespace LCU_Slave
