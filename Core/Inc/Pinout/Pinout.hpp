#ifndef PINOUT_HPP
#define PINOUT_HPP

#include "HALAL/Models/Pin.hpp" // New pins
#include "HALAL/Models/PinModel/Pin.hpp" // Old pins
#include "Common/Flags.hpp"

namespace Pinout {
    // ============================================
    // General bits and bobs
    // ============================================

    /* LED (Digital Output) */
    auto &led_fault = ST_LIB::PE8; // Change it to the actual pin
    auto &led_operational = ST_LIB::PF15; // Change it to the actual pin

    /* Fault Lines (EXTI) */
    auto &master_fault = ST_LIB::PE0;
    auto &slave_fault = ST_LIB::PE1;

    // ============================================
    // LPU
    // ============================================

    /* EN_BUFF (Digital Output) */
    auto &en_buff_1 = ST_LIB::PG7;
    auto &en_buff_2 = ST_LIB::PG6;
    auto &en_buff_3 = ST_LIB::PG5;
    auto &en_buff_4 = ST_LIB::PG4;
    auto &en_buff_5 = ST_LIB::PB2;

    /* PWM (Timer) */
    auto &pwm1_1 = ST_LIB::PE5;
    auto &pwm1_2 = ST_LIB::PE6;

    auto &pwm2_1 = ST_LIB::PB4;
    auto &pwm2_2 = ST_LIB::PB5;

    auto &pwm3_1 = ST_LIB::PC8;
    auto &pwm3_2 = ST_LIB::PC9;

    auto &pwm4_1 = ST_LIB::PC6;
    auto &pwm4_2 = ST_LIB::PC7;

    auto &pwm5_1 = ST_LIB::PD15;
    auto &pwm5_2 = ST_LIB::PD14;

    auto &pwm6_1 = ST_LIB::PD12;
    auto &pwm6_2 = ST_LIB::PD13;

    auto &pwm7_1 = ST_LIB::PB9;
    auto &pwm7_2 = ST_LIB::PB8;

    auto &pwm8_1 = ST_LIB::PB14;
    auto &pwm8_2 = ST_LIB::PB15;

    auto &pwm9_1 = ST_LIB::PE13;
    auto &pwm9_2 = ST_LIB::PE14;

    auto &pwm10_1 = ST_LIB::PE9;
    auto &pwm10_2 = ST_LIB::PE11;

    /* SHUNT (ADC) */
    auto &shunt1 = ST_LIB::PC1;
    auto &shunt2 = ST_LIB::PC0;
    auto &shunt3 = ST_LIB::PA0;
    auto &shunt4 = ST_LIB::PA1;
    auto &shunt5 = ST_LIB::PA2;
    auto &shunt6 = ST_LIB::PA3;
    auto &shunt7 = ST_LIB::PF14;
    auto &shunt8 = ST_LIB::PF13;
    auto &shunt9 = ST_LIB::PF12;
    auto &shunt10 = ST_LIB::PF11;

    /* VBAT (ADC) */
    auto &vbat_1 = ST_LIB::PF3;
    auto &vbat_2 = ST_LIB::PF4;
    auto &vbat_3 = ST_LIB::PF5;
    auto &vbat_4 = ST_LIB::PF6;
    auto &vbat_5 = ST_LIB::PF8;
    auto &vbat_6 = ST_LIB::PF7;
    auto &vbat_7 = ST_LIB::PF10;
    auto &vbat_8 = ST_LIB::PF9;
    auto &vbat_9 = ST_LIB::PC3;
    auto &vbat_10 = ST_LIB::PC2;

    // ============================================
    // Airgap
    // ============================================

    /* AIRGAP (ADC) */
    auto &airgap_1 = ST_LIB::PA7;
    auto &airgap_2 = ST_LIB::PA6;
    auto &airgap_3 = ST_LIB::PA5;
    auto &airgap_4 = ST_LIB::PA4;
    auto &airgap_5 = ST_LIB::PB1;
    auto &airgap_6 = ST_LIB::PB0;
    auto &airgap_7 = ST_LIB::PC5;
    auto &airgap_8 = ST_LIB::PC4;

    // ============================================
    // SPI
    // ============================================

    // auto spi_inst = /* TODO */;
    auto &spi_sck = ST_LIB::PC10;
    auto &spi_miso = ST_LIB::PC11;
    auto &spi_mosi = ST_LIB::PC12;
    auto &spi_nss = ST_LIB::PD3; // Will use as GPIO with software NSS management (always active)
};

#endif // PINOUT_HPP