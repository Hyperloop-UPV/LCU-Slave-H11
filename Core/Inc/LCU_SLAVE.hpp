#ifndef LCU_SLAVE_TYPES_HPP
#define LCU_SLAVE_TYPES_HPP

#include "ST-LIB.hpp"
#include "LPU/LPU.hpp"
#include "Airgap/Airgap.hpp"
#include "Pinout/Pinout.hpp"
#include "ConfigShared.hpp"
#include "SpiShared.hpp"
#include "FlagsShared.hpp"
#include "StateMachine/LCU_StateMachine.hpp"
#include "Communications/Communications.hpp"

// ============================================
// Hardware Configuration
// ============================================

namespace LCU_Slave {

/**
 * Main API
 */

void init();
void update();

#ifdef USE_SPI_ERROR
constexpr uint32_t MAX_SPI_ERRORS = 10;
constexpr uint32_t SPI_TIMEOUT_LIMIT = 1000;
#endif

inline constexpr auto led_operational_req =
    ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::led_operational);
inline constexpr auto led_fault_req = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::led_fault);

inline bool master_fault_triggered = false;

inline uint32_t reset_counter = 0;
inline constexpr auto master_fault_req = ST_LIB::EXTIDomain::Device(
    Pinout::master_fault,
    ST_LIB::EXTIDomain::Trigger::FALLING_EDGE,
    []() {
        master_fault_triggered = true;
        reset_counter++;
    }
);
inline constexpr auto slave_fault_req =
    ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::slave_fault);

// Timer and PWM configuration
#ifdef USE_1_DOF
inline constexpr auto pwm_positive = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm1_1, .channel = Pinout::pwm1_channel_1}
);

inline constexpr auto pwm_negative = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm1_2, .channel = Pinout::pwm1_channel_2}
);

inline constexpr auto timer =
    ST_LIB::TimerDomain::Timer({.request = Pinout::timer15}, pwm_positive, pwm_negative);

inline constexpr auto en_buff = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::en_buff_1);

inline float vbat_buffer = 0.0f;
inline float shunt_buffer = 0.0f;
inline float airgap_buffer = 0.0f;

inline constexpr auto adc_vbat = ST_LIB::ADCDomain::ADC(Pinout::vbat_1, vbat_buffer);
inline constexpr auto adc_shunt = ST_LIB::ADCDomain::ADC(Pinout::shunt_1, shunt_buffer);
inline constexpr auto adc_airgap = ST_LIB::ADCDomain::ADC(Pinout::airgap_1, airgap_buffer);
#elif defined(USE_5_DOF)
inline constexpr auto pwm_positive_1_req = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm1_1, .channel = Pinout::pwm1_channel_1}
);
inline constexpr auto pwm_negative_1_req = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm1_2, .channel = Pinout::pwm1_channel_2}
);
inline constexpr auto timer15_req = ST_LIB::TimerDomain::Timer(
    {.request = Pinout::timer15},
    pwm_positive_1_req,
    pwm_negative_1_req
);

inline constexpr auto pwm_positive_2_req = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm2_1, .channel = Pinout::pwm2_channel_1}
);
inline constexpr auto pwm_negative_2_req = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm2_2, .channel = Pinout::pwm2_channel_2}
);
inline constexpr auto pwm_positive_3_req = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm3_1, .channel = Pinout::pwm3_channel_1}
);
inline constexpr auto pwm_negative_3_req = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm3_2, .channel = Pinout::pwm3_channel_2}
);
inline constexpr auto timer3_req = ST_LIB::TimerDomain::Timer(
    {.request = Pinout::timer3},
    pwm_positive_2_req,
    pwm_negative_2_req,
    pwm_positive_3_req,
    pwm_negative_3_req
);

inline constexpr auto pwm_positive_4_req = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm4_1, .channel = Pinout::pwm4_channel_1}
);
inline constexpr auto pwm_negative_4_req = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm4_2, .channel = Pinout::pwm4_channel_2}
);
inline constexpr auto timer8_req =
    ST_LIB::TimerDomain::Timer({.request = Pinout::timer8}, pwm_positive_4_req, pwm_negative_4_req);

inline constexpr auto pwm_positive_5_req = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm5_1, .channel = Pinout::pwm5_channel_1}
);
inline constexpr auto pwm_negative_5_req = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm5_2, .channel = Pinout::pwm5_channel_2}
);
inline constexpr auto pwm_positive_6_req = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm6_1, .channel = Pinout::pwm6_channel_1}
);
inline constexpr auto pwm_negative_6_req = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm6_2, .channel = Pinout::pwm6_channel_2}
);
inline constexpr auto timer4_req = ST_LIB::TimerDomain::Timer(
    {.request = Pinout::timer4},
    pwm_positive_5_req,
    pwm_negative_5_req,
    pwm_positive_6_req,
    pwm_negative_6_req
);

inline constexpr auto pwm_positive_7_req = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm7_1, .channel = Pinout::pwm7_channel_1}
);
inline constexpr auto pwm_negative_7_req = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm7_2, .channel = Pinout::pwm7_channel_2}
);
inline constexpr auto timer17_req =
    ST_LIB::TimerDomain::Timer({.request = Pinout::timer17}, pwm_positive_7_req);
inline constexpr auto timer16_req =
    ST_LIB::TimerDomain::Timer({.request = Pinout::timer16}, pwm_negative_7_req);

inline constexpr auto pwm_positive_8_req = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm8_1, .channel = Pinout::pwm8_channel_1}
);
inline constexpr auto pwm_negative_8_req = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm8_2, .channel = Pinout::pwm8_channel_2}
);
inline constexpr auto timer12_req = ST_LIB::TimerDomain::Timer(
    {.request = Pinout::timer12},
    pwm_positive_8_req,
    pwm_negative_8_req
);

inline constexpr auto pwm_positive_9_req = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm9_1, .channel = Pinout::pwm9_channel_1}
);
inline constexpr auto pwm_negative_9_req = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm9_2, .channel = Pinout::pwm9_channel_2}
);
inline constexpr auto pwm_positive_10_req = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm10_1, .channel = Pinout::pwm10_channel_1}
);
inline constexpr auto pwm_negative_10_req = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm10_2, .channel = Pinout::pwm10_channel_2}
);
inline constexpr auto timer1_req = ST_LIB::TimerDomain::Timer(
    {.request = Pinout::timer1},
    pwm_positive_9_req,
    pwm_negative_9_req,
    pwm_positive_10_req,
    pwm_negative_10_req
);

inline constexpr auto en_buff_1_req = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::en_buff_1);
inline constexpr auto en_buff_2_req = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::en_buff_2);
inline constexpr auto en_buff_3_req = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::en_buff_3);
inline constexpr auto en_buff_4_req = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::en_buff_4);
inline constexpr auto en_buff_5_req = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::en_buff_5);

inline float vbat_1_buffer = 0.0f;
inline float vbat_2_buffer = 0.0f;
inline float vbat_3_buffer = 0.0f;
inline float vbat_4_buffer = 0.0f;
inline float vbat_5_buffer = 0.0f;
inline float vbat_6_buffer = 0.0f;
inline float vbat_7_buffer = 0.0f;
inline float vbat_8_buffer = 0.0f;
inline float vbat_9_buffer = 0.0f;
inline float vbat_10_buffer = 0.0f;
inline float shunt1_buffer = 0.0f;
inline float shunt2_buffer = 0.0f;
inline float shunt3_buffer = 0.0f;
inline float shunt4_buffer = 0.0f;
inline float shunt5_buffer = 0.0f;
inline float shunt6_buffer = 0.0f;
inline float shunt7_buffer = 0.0f;
inline float shunt8_buffer = 0.0f;
inline float shunt9_buffer = 0.0f;
inline float shunt10_buffer = 0.0f;
inline float airgap_1_buffer = 0.0f;
inline float airgap_2_buffer = 0.0f;
inline float airgap_3_buffer = 0.0f;
inline float airgap_4_buffer = 0.0f;
inline float airgap_5_buffer = 0.0f;
inline float airgap_6_buffer = 0.0f;
inline float airgap_7_buffer = 0.0f;
inline float airgap_8_buffer = 0.0f;

inline constexpr auto adc_vbat_1_req = ST_LIB::ADCDomain::ADC(Pinout::vbat_1, vbat_1_buffer);
inline constexpr auto adc_vbat_2_req = ST_LIB::ADCDomain::ADC(Pinout::vbat_2, vbat_2_buffer);
inline constexpr auto adc_vbat_3_req = ST_LIB::ADCDomain::ADC(Pinout::vbat_3, vbat_3_buffer);
inline constexpr auto adc_vbat_4_req = ST_LIB::ADCDomain::ADC(Pinout::vbat_4, vbat_4_buffer);
inline constexpr auto adc_vbat_5_req = ST_LIB::ADCDomain::ADC(Pinout::vbat_5, vbat_5_buffer);
inline constexpr auto adc_vbat_6_req = ST_LIB::ADCDomain::ADC(Pinout::vbat_6, vbat_6_buffer);
inline constexpr auto adc_vbat_7_req = ST_LIB::ADCDomain::ADC(Pinout::vbat_7, vbat_7_buffer);
inline constexpr auto adc_vbat_8_req = ST_LIB::ADCDomain::ADC(Pinout::vbat_8, vbat_8_buffer);
inline constexpr auto adc_vbat_9_req = ST_LIB::ADCDomain::ADC(Pinout::vbat_9, vbat_9_buffer);
inline constexpr auto adc_vbat_10_req = ST_LIB::ADCDomain::ADC(Pinout::vbat_10, vbat_10_buffer);
inline constexpr auto adc_shunt_1_req = ST_LIB::ADCDomain::ADC(Pinout::shunt_1, shunt1_buffer);
inline constexpr auto adc_shunt_2_req = ST_LIB::ADCDomain::ADC(Pinout::shunt_2, shunt2_buffer);
inline constexpr auto adc_shunt_3_req = ST_LIB::ADCDomain::ADC(Pinout::shunt_3, shunt3_buffer);
inline constexpr auto adc_shunt_4_req = ST_LIB::ADCDomain::ADC(Pinout::shunt_4, shunt4_buffer);
inline constexpr auto adc_shunt_5_req = ST_LIB::ADCDomain::ADC(Pinout::shunt_5, shunt5_buffer);
inline constexpr auto adc_shunt_6_req = ST_LIB::ADCDomain::ADC(Pinout::shunt_6, shunt6_buffer);
inline constexpr auto adc_shunt_7_req = ST_LIB::ADCDomain::ADC(Pinout::shunt_7, shunt7_buffer);
inline constexpr auto adc_shunt_8_req = ST_LIB::ADCDomain::ADC(Pinout::shunt_8, shunt8_buffer);
inline constexpr auto adc_shunt_9_req = ST_LIB::ADCDomain::ADC(Pinout::shunt_9, shunt9_buffer);
inline constexpr auto adc_shunt_10_req = ST_LIB::ADCDomain::ADC(Pinout::shunt_10, shunt10_buffer);
inline constexpr auto adc_airgap_1_req = ST_LIB::ADCDomain::ADC(Pinout::airgap_1, airgap_1_buffer);
inline constexpr auto adc_airgap_2_req = ST_LIB::ADCDomain::ADC(Pinout::airgap_2, airgap_2_buffer);
inline constexpr auto adc_airgap_3_req = ST_LIB::ADCDomain::ADC(Pinout::airgap_3, airgap_3_buffer);
inline constexpr auto adc_airgap_4_req = ST_LIB::ADCDomain::ADC(Pinout::airgap_4, airgap_4_buffer);
inline constexpr auto adc_airgap_5_req = ST_LIB::ADCDomain::ADC(Pinout::airgap_5, airgap_5_buffer);
inline constexpr auto adc_airgap_6_req = ST_LIB::ADCDomain::ADC(Pinout::airgap_6, airgap_6_buffer);
inline constexpr auto adc_airgap_7_req = ST_LIB::ADCDomain::ADC(Pinout::airgap_7, airgap_7_buffer);
inline constexpr auto adc_airgap_8_req = ST_LIB::ADCDomain::ADC(Pinout::airgap_8, airgap_8_buffer);

#endif

inline constexpr auto spi_req = ST_LIB::SPIDomain::
    Device<ST_LIB::DMADomain::Stream::dma1_stream5, ST_LIB::DMADomain::Stream::dma1_stream6>(
        ST_LIB::SPIDomain::SPIMode::SLAVE,
        Pinout::spi_peripheral,
        2000000,
        Pinout::spi_sck,
        Pinout::spi_miso,
        Pinout::spi_mosi,
        spi_conf
    );
inline constexpr auto slave_ready_req = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::spi_nss);

using Frame = SystemFrame<false>; // false for Slave

using BoardPolicy = ST_LIB::FaultPolicy<LCU_SM::sm_operational, LCU_SM::on_fault_enter>;

using Board = ST_LIB::Board<
    BoardPolicy,
    led_operational_req,
    led_fault_req,
    master_fault_req,
    slave_fault_req,
    spi_req,
    slave_ready_req,
#ifdef USE_1_DOF
    timer,
    en_buff,
    adc_vbat,
    adc_shunt,
    adc_airgap
#elif defined(USE_5_DOF)
    timer15_req,
    timer3_req,
    timer8_req,
    timer4_req,
    timer17_req,
    timer16_req,
    timer12_req,
    timer1_req,
    en_buff_1_req,
    en_buff_2_req,
    en_buff_3_req,
    en_buff_4_req,
    en_buff_5_req,
    adc_vbat_1_req,
    adc_vbat_2_req,
    adc_vbat_3_req,
    adc_vbat_4_req,
    adc_vbat_5_req,
    adc_vbat_6_req,
    adc_vbat_7_req,
    adc_vbat_8_req,
    adc_vbat_9_req,
    adc_vbat_10_req,
    adc_shunt_1_req,
    adc_shunt_2_req,
    adc_shunt_3_req,
    adc_shunt_4_req,
    adc_shunt_5_req,
    adc_shunt_6_req,
    adc_shunt_7_req,
    adc_shunt_8_req,
    adc_shunt_9_req,
    adc_shunt_10_req,
    adc_airgap_1_req,
    adc_airgap_2_req,
    adc_airgap_3_req,
    adc_airgap_4_req,
    adc_airgap_5_req,
    adc_airgap_6_req,
    adc_airgap_7_req,
    adc_airgap_8_req
#endif
    >;

inline constexpr auto& led_operational = Board::instance_of<led_operational_req>();
inline constexpr auto& led_fault = Board::instance_of<led_fault_req>();
inline constexpr auto& master_fault = Board::instance_of<master_fault_req>();
inline constexpr auto& slave_fault = Board::instance_of<slave_fault_req>();
inline auto spi = ST_LIB::SPIDomain::SPIWrapper<spi_req>(Board::instance_of<spi_req>()
); // Should make SPI get instance in compile-time
inline constexpr auto& slave_ready = Board::instance_of<slave_ready_req>();
#ifdef USE_1_DOF
inline auto timer = get_timer_instance(Board, timer_req);
inline auto pwm_positive = timer.template get_pwm<pwm_positive_req>();
inline auto pwm_negative = timer.template get_pwm<pwm_negative_req>();
inline constexpr auto& en_buff = Board::instance_of<en_buff>();
inline constexpr auto& adc_vbat = Board::instance_of<adc_vbat>();
inline constexpr auto& adc_shunt = Board::instance_of<adc_shunt>();
inline constexpr auto& adc_airgap = Board::instance_of<adc_airgap>();

inline auto lpu = make_lpu<1, 1>(
    my_pwm_positive,
    my_pwm_negative,
    adc_vbat,
    adc_shunt,
    0.0f,
    1.0f,
    344.5f,
    -197.1f
);
inline auto lpu_array = LpuArrayType(std::tie(lpu), std::tie(en_buff));
inline auto airgap = Airgap<8>(adc_airgap, 0.00020f, 0.006987);
inline auto airgap_array = AirgapArray(airgap);
#elif defined(USE_5_DOF)
inline auto timer15 =
    get_timer_instance(Board, timer15_req); // Should get timers get instance in compile-time
inline auto timer3 = get_timer_instance(Board, timer3_req);
inline auto timer8 = get_timer_instance(Board, timer8_req);
inline auto timer4 = get_timer_instance(Board, timer4_req);
inline auto timer17 = get_timer_instance(Board, timer17_req);
inline auto timer16 = get_timer_instance(Board, timer16_req);
inline auto timer12 = get_timer_instance(Board, timer12_req);
inline auto timer1 = get_timer_instance(Board, timer1_req);
inline auto pwm_positive_1 = timer15.template get_pwm<pwm_positive_1_req>(
); // Should get PWM channels get instance in compile-time
inline auto pwm_negative_1 = timer15.template get_pwm<pwm_negative_1_req>();
inline auto pwm_positive_2 = timer3.template get_pwm<pwm_positive_2_req>();
inline auto pwm_negative_2 = timer3.template get_pwm<pwm_negative_2_req>();
inline auto pwm_positive_3 = timer3.template get_pwm<pwm_positive_3_req>();
inline auto pwm_negative_3 = timer3.template get_pwm<pwm_negative_3_req>();
inline auto pwm_positive_4 = timer8.template get_pwm<pwm_positive_4_req>();
inline auto pwm_negative_4 = timer8.template get_pwm<pwm_negative_4_req>();
inline auto pwm_positive_5 = timer4.template get_pwm<pwm_positive_5_req>();
inline auto pwm_negative_5 = timer4.template get_pwm<pwm_negative_5_req>();
inline auto pwm_positive_6 = timer4.template get_pwm<pwm_positive_6_req>();
inline auto pwm_negative_6 = timer4.template get_pwm<pwm_negative_6_req>();
inline auto pwm_positive_7 = timer17.template get_pwm<pwm_positive_7_req>();
inline auto pwm_negative_7 = timer16.template get_pwm<pwm_negative_7_req>();
inline auto pwm_positive_8 = timer12.template get_pwm<pwm_positive_8_req>();
inline auto pwm_negative_8 = timer12.template get_pwm<pwm_negative_8_req>();
inline auto pwm_positive_9 = timer1.template get_pwm<pwm_positive_9_req>();
inline auto pwm_negative_9 = timer1.template get_pwm<pwm_negative_9_req>();
inline auto pwm_positive_10 = timer1.template get_pwm<pwm_positive_10_req>();
inline auto pwm_negative_10 = timer1.template get_pwm<pwm_negative_10_req>();
inline constexpr auto& en_buff_1 = Board::instance_of<en_buff_1_req>();
inline constexpr auto& en_buff_2 = Board::instance_of<en_buff_2_req>();
inline constexpr auto& en_buff_3 = Board::instance_of<en_buff_3_req>();
inline constexpr auto& en_buff_4 = Board::instance_of<en_buff_4_req>();
inline constexpr auto& en_buff_5 = Board::instance_of<en_buff_5_req>();
inline constexpr auto& adc_vbat_1 = Board::instance_of<adc_vbat_1_req>();
inline constexpr auto& adc_vbat_2 = Board::instance_of<adc_vbat_2_req>();
inline constexpr auto& adc_vbat_3 = Board::instance_of<adc_vbat_3_req>();
inline constexpr auto& adc_vbat_4 = Board::instance_of<adc_vbat_4_req>();
inline constexpr auto& adc_vbat_5 = Board::instance_of<adc_vbat_5_req>();
inline constexpr auto& adc_vbat_6 = Board::instance_of<adc_vbat_6_req>();
inline constexpr auto& adc_vbat_7 = Board::instance_of<adc_vbat_7_req>();
inline constexpr auto& adc_vbat_8 = Board::instance_of<adc_vbat_8_req>();
inline constexpr auto& adc_vbat_9 = Board::instance_of<adc_vbat_9_req>();
inline constexpr auto& adc_vbat_10 = Board::instance_of<adc_vbat_10_req>();
inline constexpr auto& adc_shunt_1 = Board::instance_of<adc_shunt_1_req>();
inline constexpr auto& adc_shunt_2 = Board::instance_of<adc_shunt_2_req>();
inline constexpr auto& adc_shunt_3 = Board::instance_of<adc_shunt_3_req>();
inline constexpr auto& adc_shunt_4 = Board::instance_of<adc_shunt_4_req>();
inline constexpr auto& adc_shunt_5 = Board::instance_of<adc_shunt_5_req>();
inline constexpr auto& adc_shunt_6 = Board::instance_of<adc_shunt_6_req>();
inline constexpr auto& adc_shunt_7 = Board::instance_of<adc_shunt_7_req>();
inline constexpr auto& adc_shunt_8 = Board::instance_of<adc_shunt_8_req>();
inline constexpr auto& adc_shunt_9 = Board::instance_of<adc_shunt_9_req>();
inline constexpr auto& adc_shunt_10 = Board::instance_of<adc_shunt_10_req>();
inline constexpr auto& adc_airgap_1 = Board::instance_of<adc_airgap_1_req>();
inline constexpr auto& adc_airgap_2 = Board::instance_of<adc_airgap_2_req>();
inline constexpr auto& adc_airgap_3 = Board::instance_of<adc_airgap_3_req>();
inline constexpr auto& adc_airgap_4 = Board::instance_of<adc_airgap_4_req>();
inline constexpr auto& adc_airgap_5 = Board::instance_of<adc_airgap_5_req>();
inline constexpr auto& adc_airgap_6 = Board::instance_of<adc_airgap_6_req>();
inline constexpr auto& adc_airgap_7 = Board::instance_of<adc_airgap_7_req>();
inline constexpr auto& adc_airgap_8 = Board::instance_of<adc_airgap_8_req>();

inline auto lpu10 =
    make_lpu<2, 1>(pwm_positive_1, pwm_negative_1, adc_vbat_1, adc_shunt_1, 0.0f, 1.0f, 0.0f, 1.0f);
inline auto lpu6 =
    make_lpu<2, 1>(pwm_positive_2, pwm_negative_2, adc_vbat_2, adc_shunt_2, 0.0f, 1.0f, 0.0f, 1.0f);
inline auto lpu8 =
    make_lpu<2, 1>(pwm_positive_3, pwm_negative_3, adc_vbat_3, adc_shunt_3, 0.0f, 1.0f, 0.0f, 1.0f);
inline auto lpu2 = make_lpu<2, 1>(
    pwm_positive_4,
    pwm_negative_4,
    adc_vbat_4,
    adc_shunt_4,
    0.0f,
    1.0f,
    // 310.3f, -181.0f // Characterized for LPU2
    304.1f,
    -177.5f // Characterized for LPU3
);
inline auto lpu5 =
    make_lpu<2, 1>(pwm_positive_5, pwm_negative_5, adc_vbat_5, adc_shunt_5, 0.0f, 1.0f, 0.0f, 1.0f);
inline auto lpu4 = make_lpu<2, 1>(
    pwm_positive_6,
    pwm_negative_6,
    adc_vbat_6,
    adc_shunt_6,
    0.0f,
    1.0f,
    317.0f,
    -184.8f // Characterized for LPU7
);
inline auto lpu7 =
    make_lpu<2, 1>(pwm_positive_7, pwm_negative_7, adc_vbat_7, adc_shunt_7, 0.0f, 1.0f, 0.0f, 1.0f);
inline auto lpu3 = make_lpu<2, 1>(
    pwm_positive_8,
    pwm_negative_8,
    adc_vbat_8,
    adc_shunt_8,
    0.0f,
    1.0f,
    329.0f,
    -193.4f // Characterized for LPU4
);
inline auto lpu9 =
    make_lpu<2, 1>(pwm_positive_9, pwm_negative_9, adc_vbat_9, adc_shunt_9, 0.0f, 1.0f, 0.0f, 1.0f);
inline auto lpu1 = make_lpu<2, 1>(
    pwm_positive_10,
    pwm_negative_10,
    adc_vbat_10,
    adc_shunt_10,
    0.0f,
    1.0f,
    317.0f,
    -185.1f // Characterized for LPU1
);
inline auto lpu_array = LpuArray(
    std::tie(lpu1, lpu2, lpu3, lpu4, lpu5, lpu6, lpu7, lpu8, lpu9, lpu10),
    std::tie(en_buff_1, en_buff_2, en_buff_3, en_buff_4, en_buff_5)
);
inline auto airgap5 = Airgap<8>(adc_airgap_1, 0.00000f, 1);
inline auto airgap1 = Airgap<8>(adc_airgap_2, 0.07934f + 0.0023 - 0.0075f - 0.0018f, 0.01159f);
inline auto airgap4 = Airgap<8>(adc_airgap_3, 0.07937f + 0.0016 - 0.0075f - 0.002f, 0.01155f);
inline auto airgap3 = Airgap<8>(adc_airgap_4, 0.07928f + 0.0016 - 0.0075f - 0.0027f, 0.01175f);
inline auto airgap2 = Airgap<8>(adc_airgap_5, 0.07926f + 0.0023 - 0.0075f - 0.0022f, 0.01158f);
// inline auto airgap2 = Airgap<8>(adc_airgap_2, 0.0f, 1.0f);
// inline auto airgap3 = Airgap<8>(adc_airgap_3, 0.0f, 1.0f);
// inline auto airgap4 = Airgap<8>(adc_airgap_4, 0.0f, 1.0f);
// inline auto airgap5 = Airgap<8>(adc_airgap_5, 0.0f, 1.0f);
inline auto airgap6 = Airgap<8>(adc_airgap_6, 0.00000f, 1);
inline auto airgap7 = Airgap<8>(adc_airgap_7, 0.00000f, 1);
inline auto airgap8 = Airgap<8>(adc_airgap_8, 0.00000f, 1);
inline auto airgap_array =
    AirgapArray(airgap1, airgap2, airgap3, airgap4, airgap5, airgap6, airgap7, airgap8);

inline auto protection_current_1 = Protections::protection<
    "current_1",
    lpu1.shunt_v>(
    Protections::Rules::range(-60.0f, 60.0f)
);
inline auto protection_current_2 = Protections::protection<
    "current_2",
    lpu2.shunt_v>(
    Protections::Rules::range(-60.0f, 60.0f)
);
inline auto protection_current_3 = Protections::protection<
    "current_3",
    lpu3.shunt_v>(
    Protections::Rules::range(-60.0f, 60.0f)
);
inline auto protection_current_4 = Protections::protection<
    "current_4",
    lpu4.shunt_v>(
    Protections::Rules::range(-60.0f, 60.0f)
);

using AppProtectionEngine = Protections::ProtectionEngine<
    protection_current_1,
    protection_current_2,
    protection_current_3,
    protection_current_4
>;

#endif

} // namespace LCU_Slave

#endif // LCU_SLAVE_TYPES_HPP
