#include "LCU_SLAVE.hpp"
#include "Control/Control.hpp"
#include "Control/dpc_ai_controller.h"
#include "Timing/AppTiming.h"

namespace LCU_Slave {

void init() {
    Board::init();
    app_timing_init();
    AppProtectionEngine::initialize();

#if defined(USE_DPC_AI)
    if (dpc_ai_init() == 0) {
        FAULT("DPC AI network initialization failed");
    }
#endif

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
}

void update() {
    const uint32_t loop_start = app_timing_cycles();
    uint32_t section_start = loop_start;

    Communications::update();
    Control::control.output.Timing.communications_us = app_timing_elapsed_cycles(section_start);
    section_start = app_timing_cycles();

    FaultController::check_transitions();
    Control::control.output.Timing.fault_check_us = app_timing_elapsed_cycles(section_start);
    section_start = app_timing_cycles();

    LCU_SM::update();
    Control::control.output.Timing.state_machine_us = app_timing_elapsed_cycles(section_start);
    section_start = app_timing_cycles();

    Scheduler::update();
    Control::control.output.Timing.scheduler_us = app_timing_elapsed_cycles(section_start);
    section_start = app_timing_cycles();

    MDMA::update();
    Control::control.output.Timing.mdma_us = app_timing_elapsed_cycles(section_start);
    section_start = app_timing_cycles();

    AppProtectionEngine::evaluate();
    Control::control.output.Timing.protections_us = app_timing_elapsed_cycles(section_start);
    section_start = app_timing_cycles();

    Diagnostics::Hub::flush();
    Control::control.output.Timing.diagnostics_us = app_timing_elapsed_cycles(section_start);
    app_timing_record_u32(
        &Control::control.output.Timing.loop_total_us,
        &Control::control.output.Timing.loop_total_max_us,
        app_timing_elapsed_cycles(loop_start)
    );

    if (reset_counter >= 5) {
        HAL_Delay(100); // Delay in case more faults are coming in rapidly
        HAL_NVIC_SystemReset();
    }
}

} // Namespace LCU_Slave
