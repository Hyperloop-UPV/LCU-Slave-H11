#ifndef PINOUT_HPP
#define PINOUT_HPP

#include "HALAL/Models/Pin.hpp"          // New pins
#include "HALAL/Models/PinModel/Pin.hpp" // Old pins
#include "Common/Flags.hpp"

namespace Pinout {
// ============================================
// General bits and bobs
// ============================================

/* LED (Digital Output) */
inline auto& led_fault = ST_LIB::PE8;        // Change it to the actual pin
inline auto& led_operational = ST_LIB::PF15; // Change it to the actual pin

/* Fault Lines (EXTI) */
inline auto& master_fault = ST_LIB::PG12;
inline auto& slave_fault = ST_LIB::PD2;

// ============================================
// LPU
// ============================================

/* EN_BUFF (Digital Output) */
inline auto& en_buff_1 = ST_LIB::PG7;
inline auto& en_buff_2 = ST_LIB::PG6;
inline auto& en_buff_3 = ST_LIB::PG5;
inline auto& en_buff_4 = ST_LIB::PG4;
inline auto& en_buff_5 = ST_LIB::PB2;

/* PWM (Timer) */
inline auto constexpr timer15 = ST_LIB::TimerRequest::GeneralPurpose_15;
inline auto& pwm1_1 = ST_LIB::PE5;
inline auto& pwm1_2 = ST_LIB::PE6;
inline auto constexpr pwm1_channel_1 = ST_LIB::TimerChannel::CHANNEL_1;
inline auto constexpr pwm1_channel_2 = ST_LIB::TimerChannel::CHANNEL_2;

inline auto constexpr timer3 = ST_LIB::TimerRequest::GeneralPurpose_3;
inline auto& pwm2_1 = ST_LIB::PB4;
inline auto& pwm2_2 = ST_LIB::PB5;
inline auto constexpr pwm2_channel_1 = ST_LIB::TimerChannel::CHANNEL_1;
inline auto constexpr pwm2_channel_2 = ST_LIB::TimerChannel::CHANNEL_2;
inline auto& pwm3_1 = ST_LIB::PC8;
inline auto& pwm3_2 = ST_LIB::PC9;
inline auto constexpr pwm3_channel_1 = ST_LIB::TimerChannel::CHANNEL_3;
inline auto constexpr pwm3_channel_2 = ST_LIB::TimerChannel::CHANNEL_4;

inline auto constexpr timer8 = ST_LIB::TimerRequest::Advanced_8;
inline auto& pwm4_1 = ST_LIB::PC6;
inline auto& pwm4_2 = ST_LIB::PC7;
inline auto constexpr pwm4_channel_1 = ST_LIB::TimerChannel::CHANNEL_1;
inline auto constexpr pwm4_channel_2 = ST_LIB::TimerChannel::CHANNEL_2;

inline auto constexpr timer4 = ST_LIB::TimerRequest::GeneralPurpose_4;
inline auto& pwm5_1 = ST_LIB::PD14;
inline auto& pwm5_2 = ST_LIB::PD15;
inline auto constexpr pwm5_channel_1 = ST_LIB::TimerChannel::CHANNEL_3;
inline auto constexpr pwm5_channel_2 = ST_LIB::TimerChannel::CHANNEL_4;
inline auto& pwm6_1 = ST_LIB::PD12;
inline auto& pwm6_2 = ST_LIB::PD13;
inline auto constexpr pwm6_channel_1 = ST_LIB::TimerChannel::CHANNEL_1;
inline auto constexpr pwm6_channel_2 = ST_LIB::TimerChannel::CHANNEL_2;

inline auto constexpr timer17 = ST_LIB::TimerRequest::GeneralPurpose_17;
inline auto constexpr timer16 = ST_LIB::TimerRequest::GeneralPurpose_16;
inline auto& pwm7_1 = ST_LIB::PB9;
inline auto& pwm7_2 = ST_LIB::PB8;
inline auto constexpr pwm7_channel_1 = ST_LIB::TimerChannel::CHANNEL_1;
inline auto constexpr pwm7_channel_2 = ST_LIB::TimerChannel::CHANNEL_1;

inline auto constexpr timer12 = ST_LIB::TimerRequest::SlaveTimer_12;
inline auto& pwm8_1 = ST_LIB::PB14;
inline auto& pwm8_2 = ST_LIB::PB15;
inline auto constexpr pwm8_channel_1 = ST_LIB::TimerChannel::CHANNEL_1;
inline auto constexpr pwm8_channel_2 = ST_LIB::TimerChannel::CHANNEL_2;

inline auto constexpr timer1 = ST_LIB::TimerRequest::Advanced_1;
inline auto& pwm9_1 = ST_LIB::PE13;
inline auto& pwm9_2 = ST_LIB::PE14;
inline auto constexpr pwm9_channel_1 = ST_LIB::TimerChannel::CHANNEL_3;
inline auto constexpr pwm9_channel_2 = ST_LIB::TimerChannel::CHANNEL_4;
inline auto& pwm10_1 = ST_LIB::PE9;
inline auto& pwm10_2 = ST_LIB::PE11;
inline auto constexpr pwm10_channel_1 = ST_LIB::TimerChannel::CHANNEL_1;
inline auto constexpr pwm10_channel_2 = ST_LIB::TimerChannel::CHANNEL_2;

/* SHUNT (ADC) */
inline auto& shunt_1 = ST_LIB::PC1;
inline auto& shunt_2 = ST_LIB::PC0;
inline auto& shunt_3 = ST_LIB::PA0;
inline auto& shunt_4 = ST_LIB::PA1;
inline auto& shunt_5 = ST_LIB::PA2;
inline auto& shunt_6 = ST_LIB::PA3;
inline auto& shunt_7 = ST_LIB::PF14;
inline auto& shunt_8 = ST_LIB::PF13;
inline auto& shunt_9 = ST_LIB::PF12;
inline auto& shunt_10 = ST_LIB::PF11;

/* VBAT (ADC) */
inline auto& vbat_1 = ST_LIB::PF3;
inline auto& vbat_2 = ST_LIB::PF4;
inline auto& vbat_3 = ST_LIB::PF5;
inline auto& vbat_4 = ST_LIB::PF6;
inline auto& vbat_5 = ST_LIB::PF8;
inline auto& vbat_6 = ST_LIB::PF7;
inline auto& vbat_7 = ST_LIB::PF10;
inline auto& vbat_8 = ST_LIB::PF9;
inline auto& vbat_9 = ST_LIB::PC3;
inline auto& vbat_10 = ST_LIB::PC2;

// ============================================
// Airgap
// ============================================

/* AIRGAP (ADC) */
inline auto& airgap_1 = ST_LIB::PA7;
inline auto& airgap_2 = ST_LIB::PA6;
inline auto& airgap_3 = ST_LIB::PA5;
inline auto& airgap_4 = ST_LIB::PA4;
inline auto& airgap_5 = ST_LIB::PB1;
inline auto& airgap_6 = ST_LIB::PB0;
inline auto& airgap_7 = ST_LIB::PC5;
inline auto& airgap_8 = ST_LIB::PC4;

// ============================================
// SPI
// ============================================

inline auto constexpr spi_peripheral = ST_LIB::SPIDomain::SPIPeripheral::spi3;
inline auto& spi_sck = ST_LIB::PC10;
inline auto& spi_miso = ST_LIB::PC11;
inline auto& spi_mosi = ST_LIB::PC12;
inline auto& spi_nss = ST_LIB::PD3; // Will use as GPIO with software NSS management (always active)
};                                  // namespace Pinout

#endif // PINOUT_HPP
