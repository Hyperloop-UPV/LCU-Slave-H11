#include "LCU_SLAVE.hpp"

namespace LCU_Slave {

void init() {
    Board::init();
    AppProtectionEngine::initialize();
    FaultController::start();
    // test_protection_value = 200.0f;

    slave_fault.turn_on();  // Start UP (so that a reset will be noticeable by the master)
    spi.set_software_nss(false); // We'll control NSS via GPIO
    slave_ready.turn_off();

#ifdef USE_1_DOF
    timer.set_pwm_frequency(10'000); // 10khz

#elif defined(USE_5_DOF)
    timer15.set_pwm_frequency(10'000);
    timer3.set_pwm_frequency(10'000);
    timer8.set_pwm_frequency(10'000);
    timer4.set_pwm_frequency(10'000);
    timer17.set_pwm_frequency(10'000);
    timer16.set_pwm_frequency(10'000);
    timer12.set_pwm_frequency(10'000);
    timer1.set_pwm_frequency(10'000);

#endif

    MDMA::start();

    Communications::init();

    LCU_SM::set_command_packet(&Communications::comms.command_packet);

#ifdef USE_1_DOF
    Frame::init(Communications::comms, lpu, airgap, Communications::comms, lpu);
#elif defined(USE_5_DOF)
    Frame::init(Communications::comms,
                lpu1, lpu2, lpu3, lpu4, lpu5, lpu6, lpu7, lpu8, lpu9, lpu10,
                airgap1, airgap2, airgap3, airgap4, airgap5, airgap6, airgap7, airgap8,
                Communications::comms,
                lpu1, lpu2, lpu3, lpu4, lpu5, lpu6, lpu7, lpu8, lpu9, lpu10);
#endif
}

// ============================================
// Main Loop
// ============================================
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
