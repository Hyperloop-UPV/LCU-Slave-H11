#ifndef LCU_SLAVE_HPP
#define LCU_SLAVE_HPP

#include "LCU_SLAVE_Types.hpp"
#include "ConfigShared.hpp"
#include "StateMachine/LCU_StateMachine.hpp"
#include "Communications/Communications.hpp"

namespace LCU_Slave {

// ============================================
// Global Hardware Instances (definitions)
// ============================================

// ============================================
// Initialization
// ============================================
inline void init() {
    Board::init();

    g_led_operational = &Board::instance_of<led_operational_req>();
    g_led_fault = &Board::instance_of<led_fault_req>();

    g_slave_fault = &Board::instance_of<slave_fault_req>();

    static auto my_tim = get_timer_instance(Board, timer);
    my_tim.set_pwm_frequency(10'000); // 10khz

    static auto my_pwm_positive = my_tim.template get_pwm<pwm_positive>();
    static auto my_pwm_negative = my_tim.template get_pwm<pwm_negative>();
    g_pwm_positive = &my_pwm_positive;
    g_pwm_negative = &my_pwm_negative;
    
    g_enable_pin = &Board::instance_of<en_buff_1>();

    // Create LPU
    static auto my_lpu = LPUType(
        *g_pwm_positive,
        *g_pwm_negative,
        Board::instance_of<adc_vbat>(),
        Board::instance_of<adc_shunt>(),
        0.0f,
        1.0f,
        0.0f,
        1.0f
    );
    g_lpu = &my_lpu;

    // Create Airgap
    static auto my_airgap = Airgap(Board::instance_of<adc_airgap>(), 0.0f, 1.0f);
    g_airgap = &my_airgap;

    // Create LPU Array
    static auto my_lpu_array = LpuArray(std::tie(*g_lpu), std::tie(*g_enable_pin));
    g_lpu_array = &my_lpu_array;

    // SPI
    static auto my_spi_wrapper = SpiType(Board::instance_of<spi_req>());
    my_spi_wrapper.set_software_nss(false); // We'll control NSS via GPIO
    Communications::g_spi = &my_spi_wrapper;

    Communications::g_slave_ready = &Board::instance_of<slave_ready>();
    Communications::g_slave_ready->turn_off();

    Scheduler::start();
    MDMA::start();

    Communications::init();

    LCU_SM::set_command_packet(&Communications::comms.command_packet);
    LCU_SM::start();

    Frame::init(Communications::comms, *g_lpu, *g_airgap, Communications::comms, *g_lpu);
}

// ============================================
// Main Loop
// ============================================
inline void update() {
    Communications::update();
    LCU_SM::update();
    Scheduler::update();
    MDMA::update();
}

} // namespace LCU_Slave

#endif // LCU_SLAVE_HPP
