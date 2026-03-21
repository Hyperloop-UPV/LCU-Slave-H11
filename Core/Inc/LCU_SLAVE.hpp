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
    g_slave_fault->turn_on();  // Start LOW (inactive for push-pull)

    g_master_fault = &Board::instance_of<master_fault_req>();

    // SPI
    static auto my_spi_wrapper = SpiType(Board::instance_of<spi_req>());
    my_spi_wrapper.set_software_nss(false); // We'll control NSS via GPIO
    Communications::g_spi = &my_spi_wrapper;

    Communications::g_slave_ready = &Board::instance_of<slave_ready>();
    Communications::g_slave_ready->turn_off();

#ifdef USE_1_DOF
    static auto my_tim = get_timer_instance(Board, timer);
    my_tim.set_pwm_frequency(10'000); // 10khz

    static auto my_pwm_positive = my_tim.template get_pwm<pwm_positive>();
    static auto my_pwm_negative = my_tim.template get_pwm<pwm_negative>();

    auto enable_pin = &Board::instance_of<en_buff>();

    // Create LPU
    static auto my_lpu = LPUType(
        my_pwm_positive,
        my_pwm_negative,
        Board::instance_of<adc_vbat>(),
        Board::instance_of<adc_shunt>(),
        0.0f,
        1.0f,
        0.0f,
        1.0f
    );

    // Create LPU Array
    static auto my_lpu_array = LpuArray(std::tie(my_lpu), std::tie(*enable_pin));
    g_lpu_array = &my_lpu_array;


    // Create Airgap
    static auto my_airgap = Airgap(Board::instance_of<adc_airgap>(), 0.0f, 1.0f);

    // Create Airgap Array
    static auto my_airgap_array = AirgapArrayType(my_airgap);
    g_airgap_array = &my_airgap_array;



#elif defined(USE_5_DOF)
    static auto tim15 = get_timer_instance(Board, timer15);
    static auto tim3 = get_timer_instance(Board, timer3);
    static auto tim8 = get_timer_instance(Board, timer8);
    static auto tim4 = get_timer_instance(Board, timer4);
    static auto tim17 = get_timer_instance(Board, timer17);
    static auto tim16 = get_timer_instance(Board, timer16);
    static auto tim12 = get_timer_instance(Board, timer12);
    static auto tim1 = get_timer_instance(Board, timer1);

    tim15.set_pwm_frequency(10'000);
    tim3.set_pwm_frequency(10'000);
    tim8.set_pwm_frequency(10'000);
    tim4.set_pwm_frequency(10'000);
    tim17.set_pwm_frequency(10'000);
    tim16.set_pwm_frequency(10'000);
    tim12.set_pwm_frequency(10'000);
    tim1.set_pwm_frequency(10'000);

    static auto my_pwm_positive_1 = tim15.template get_pwm<pwm_positive_1>();
    static auto my_pwm_negative_1 = tim15.template get_pwm<pwm_negative_1>();
    static auto my_pwm_positive_2 = tim3.template get_pwm<pwm_positive_2>();
    static auto my_pwm_negative_2 = tim3.template get_pwm<pwm_negative_2>();
    static auto my_pwm_positive_3 = tim3.template get_pwm<pwm_positive_3>();
    static auto my_pwm_negative_3 = tim3.template get_pwm<pwm_negative_3>();
    static auto my_pwm_positive_4 = tim8.template get_pwm<pwm_positive_4>();
    static auto my_pwm_negative_4 = tim8.template get_pwm<pwm_negative_4>();
    static auto my_pwm_positive_5 = tim4.template get_pwm<pwm_positive_5>();
    static auto my_pwm_negative_5 = tim4.template get_pwm<pwm_negative_5>();
    static auto my_pwm_positive_6 = tim4.template get_pwm<pwm_positive_6>();
    static auto my_pwm_negative_6 = tim4.template get_pwm<pwm_negative_6>();
    static auto my_pwm_positive_7 = tim17.template get_pwm<pwm_positive_7>();
    static auto my_pwm_negative_7 = tim16.template get_pwm<pwm_negative_7>();
    static auto my_pwm_positive_8 = tim12.template get_pwm<pwm_positive_8>();
    static auto my_pwm_negative_8 = tim12.template get_pwm<pwm_negative_8>();
    static auto my_pwm_positive_9 = tim1.template get_pwm<pwm_positive_9>();
    static auto my_pwm_negative_9 = tim1.template get_pwm<pwm_negative_9>();
    static auto my_pwm_positive_10 = tim1.template get_pwm<pwm_positive_10>();
    static auto my_pwm_negative_10 = tim1.template get_pwm<pwm_negative_10>();

    auto enable_pin_1 = &Board::instance_of<en_buff_1>();
    auto enable_pin_2 = &Board::instance_of<en_buff_2>();
    auto enable_pin_3 = &Board::instance_of<en_buff_3>();
    auto enable_pin_4 = &Board::instance_of<en_buff_4>();
    auto enable_pin_5 = &Board::instance_of<en_buff_5>();

    // Create LPUs
    static auto my_lpu_1 = std::tuple_element_t<0, LPUTypes>(
        my_pwm_positive_1,
        my_pwm_negative_1,
        Board::instance_of<adc_vbat_1>(),
        Board::instance_of<adc_shunt_1>(),
        0.0f, 1.0f,
        0.0f, 1.0f
    );

    static auto my_lpu_2 = std::tuple_element_t<1, LPUTypes>(
        my_pwm_positive_2,
        my_pwm_negative_2,
        Board::instance_of<adc_vbat_2>(),
        Board::instance_of<adc_shunt_2>(),
        0.0f, 1.0f,
        0.0f, 1.0f
    );

    static auto my_lpu_3 = std::tuple_element_t<2, LPUTypes>(
        my_pwm_positive_3,
        my_pwm_negative_3,
        Board::instance_of<adc_vbat_3>(),
        Board::instance_of<adc_shunt_3>(),
        0.0f, 1.0f,
        0.0f, 1.0f
    );

    static auto my_lpu_4 = std::tuple_element_t<3, LPUTypes>(
        my_pwm_positive_4,
        my_pwm_negative_4,
        Board::instance_of<adc_vbat_4>(),
        Board::instance_of<adc_shunt_4>(),
        0.0f, 1.0f,
        0.0f, 1.0f
    );

    static auto my_lpu_5 = std::tuple_element_t<4, LPUTypes>(
        my_pwm_positive_5,
        my_pwm_negative_5,
        Board::instance_of<adc_vbat_5>(),
        Board::instance_of<adc_shunt_5>(),
        0.0f, 1.0f,
        0.0f, 1.0f
    );

    static auto my_lpu_6 = std::tuple_element_t<5, LPUTypes>(
        my_pwm_positive_6,
        my_pwm_negative_6,
        Board::instance_of<adc_vbat_1>(),
        Board::instance_of<adc_shunt_1>(),
        0.0f, 1.0f,
        0.0f, 1.0f
    );

    static auto my_lpu_7 = std::tuple_element_t<6, LPUTypes>(
        my_pwm_positive_7,
        my_pwm_negative_7,
        Board::instance_of<adc_vbat_2>(),
        Board::instance_of<adc_shunt_2>(),
        0.0f, 1.0f,
        0.0f, 1.0f
    );

    static auto my_lpu_8 = std::tuple_element_t<7, LPUTypes>(
        my_pwm_positive_8,
        my_pwm_negative_8,
        Board::instance_of<adc_vbat_3>(),
        Board::instance_of<adc_shunt_3>(),
        0.0f, 1.0f,
        0.0f, 1.0f
    );

    static auto my_lpu_9 = std::tuple_element_t<8, LPUTypes>(
        my_pwm_positive_9,
        my_pwm_negative_9,
        Board::instance_of<adc_vbat_4>(),
        Board::instance_of<adc_shunt_4>(),
        0.0f, 1.0f,
        0.0f, 1.0f
    );

    static auto my_lpu_10 = std::tuple_element_t<9, LPUTypes>(
        my_pwm_positive_10,
        my_pwm_negative_10,
        Board::instance_of<adc_vbat_5>(),
        Board::instance_of<adc_shunt_5>(),
        0.0f, 1.0f,
        0.0f, 1.0f
    );

    // Create LPU Array
    static auto my_lpu_array = LpuArrayType(
        std::tie(my_lpu_1, my_lpu_2, my_lpu_3, my_lpu_4, my_lpu_5, my_lpu_6, my_lpu_7, my_lpu_8, my_lpu_9, my_lpu_10),
        std::tie(*enable_pin_1, *enable_pin_2, *enable_pin_3, *enable_pin_4, *enable_pin_5)
    );
    g_lpu_array = &my_lpu_array;

    // Create Airgaps
    static auto my_airgap_1 = Airgap(Board::instance_of<adc_airgap_1>(), 0.0f, 1.0f);
    static auto my_airgap_2 = Airgap(Board::instance_of<adc_airgap_2>(), 0.0f, 1.0f);
    static auto my_airgap_3 = Airgap(Board::instance_of<adc_airgap_3>(), 0.0f, 1.0f);
    static auto my_airgap_4 = Airgap(Board::instance_of<adc_airgap_4>(), 0.0f, 1.0f);
    static auto my_airgap_5 = Airgap(Board::instance_of<adc_airgap_5>(), 0.0f, 1.0f);
    static auto my_airgap_6 = Airgap(Board::instance_of<adc_airgap_1>(), 0.0f, 1.0f);
    static auto my_airgap_7 = Airgap(Board::instance_of<adc_airgap_2>(), 0.0f, 1.0f);
    static auto my_airgap_8 = Airgap(Board::instance_of<adc_airgap_3>(), 0.0f, 1.0f);

    // Create Airgap Array
    static auto my_airgap_array = AirgapArrayType(
        std::tie(my_airgap_1, my_airgap_2, my_airgap_3, my_airgap_4, my_airgap_5, my_airgap_6, my_airgap_7, my_airgap_8)
    );
    g_airgap_array = &my_airgap_array;

#endif

    MDMA::start();

    Communications::init();

    LCU_SM::set_command_packet(&Communications::comms.command_packet);
    LCU_SM::start();

#ifdef USE_1_DOF
    Frame::init(Communications::comms, my_lpu, my_airgap, Communications::comms, my_lpu);
#elif defined(USE_5_DOF)
    Frame::init(Communications::comms,
                my_lpu_1, my_lpu_2, my_lpu_3, my_lpu_4, my_lpu_5, my_lpu_6, my_lpu_7, my_lpu_8, my_lpu_9, my_lpu_10,
                my_airgap_1, my_airgap_2, my_airgap_3, my_airgap_4, my_airgap_5, my_airgap_6, my_airgap_7, my_airgap_8,
                Communications::comms,
                my_lpu_1, my_lpu_2, my_lpu_3, my_lpu_4, my_lpu_5, my_lpu_6, my_lpu_7, my_lpu_8, my_lpu_9, my_lpu_10);
#endif
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
