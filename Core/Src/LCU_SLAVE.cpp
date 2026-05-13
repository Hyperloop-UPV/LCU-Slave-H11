#include "LCU_SLAVE.hpp"

namespace LCU_Slave {

void init() {
    Board::init();
    AppProtectionEngine::initialize();

    slave_fault.turn_on();
    spi.set_software_nss(false);
    slave_ready.turn_off();

    timer15.set_pwm_frequency(20'000);
    timer3.set_pwm_frequency(20'000);
    timer8.set_pwm_frequency(20'000);
    timer4.set_pwm_frequency(20'000);
    timer17.set_pwm_frequency(20'000);
    timer16.set_pwm_frequency(20'000);
    timer12.set_pwm_frequency(20'000);
    timer1.set_pwm_frequency(20'000);

    lpu_array.init();

    MDMA::start();

    Communications::init();

    LCU_SM::set_command_packet(&Communications::comms.command_packet);
    LCU_SM::start();

    Frame::init(
        Communications::comms,
        std::get<0>(lpu_tuple),
        std::get<1>(lpu_tuple),
        std::get<2>(lpu_tuple),
        std::get<3>(lpu_tuple),
        std::get<0>(lpu_tuple),
        std::get<0>(lpu_tuple),
        std::get<0>(lpu_tuple),
        std::get<0>(lpu_tuple),
        std::get<0>(lpu_tuple),
        std::get<0>(lpu_tuple),
        std::get<0>(airgap_tuple),
        std::get<1>(airgap_tuple),
        std::get<2>(airgap_tuple),
        std::get<3>(airgap_tuple),
        std::get<0>(airgap_tuple),
        std::get<0>(airgap_tuple),
        std::get<0>(airgap_tuple),
        std::get<0>(airgap_tuple),
        Communications::comms,
        std::get<0>(lpu_tuple),
        std::get<1>(lpu_tuple),
        std::get<2>(lpu_tuple),
        std::get<3>(lpu_tuple),
        std::get<0>(lpu_tuple),
        std::get<0>(lpu_tuple),
        std::get<0>(lpu_tuple),
        std::get<0>(lpu_tuple),
        std::get<0>(lpu_tuple),
        std::get<0>(lpu_tuple)
    );
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
}

} // Namespace LCU_Slave
