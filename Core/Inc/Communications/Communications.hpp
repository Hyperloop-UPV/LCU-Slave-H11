#ifndef COMMUNICATIONS_HPP
#define COMMUNICATIONS_HPP

#include "LCU_SLAVE_Types.hpp"
#include "ConfigShared.hpp"
#include "ControlShared.hpp"
#include "StateMachineShared.hpp"
#include "SharedCommunicationsLogic.hpp"
#include "Common/Flags.hpp"

namespace Communications {

// Communications objects to hold control and state
inline ControlBase control;
inline StateMachineBase state_machine;

// SPI
inline LCU_Slave::SpiType* g_spi = nullptr;
inline ST_LIB::DigitalOutputDomain::Instance* g_slave_ready = nullptr;

// Debug/Control Flags
inline uint16_t fixed_pwm_active = 0;  // Track if any LPU has fixed PWM enabled, as a bitmask

// Connection tracking
inline bool spi_connected = false;  // Set to true on first successful packet, false on error

// ============================================
// SPI Callbacks for Slave Mode
// ============================================
struct SlaveSPICallbacks {
    static void on_prepare_tx() {
        // Assert NSS
        g_spi->assert_nss();
    }

    static void on_spi_start() {
        g_slave_ready->turn_on();
    }

    static void on_spi_complete() {
        g_slave_ready->turn_off();
        g_spi->deassert_nss();
    }

    static void on_data_received() {
        spi_connected = true;
        fixed_pwm_active = 0;
#ifdef USE_1_DOF
        if (LCU_Slave::g_lpu_array->get_lpu<0>().is_fixed_duty_cycle) {
            fixed_pwm_active |= (1 << 0);
        }
#elif defined(USE_5_DOF)
        for (int i = 0; i < 10; ++i) {
            if (LCU_Slave::g_lpu_array->get_lpu<i>().is_fixed_duty_cycle) {
                fixed_pwm_active |= (1 << i);
            }
        }
#endif
    }

    static void on_frame_error() {
        spi_connected = false;
        g_slave_ready->turn_off();
        g_spi->deassert_nss();
    }
};

// ============================================
// SPI Communication Logic Instance
// ============================================
using SlaveSPILogic = SharedSPICommunicationLogic<
    LCU_Slave::Frame,
    ST_LIB::SPIDomain::SPIWrapper<LCU_Slave::spi_req>,
    SlaveSPICallbacks,
    false,                      // IsMaster (Slave mode)
    ENABLE_SPI_ERROR_HANDLING,  // EnableErrorHandling
    10,                         // MaxErrors (matches LCU_Slave::MAX_SPI_ERRORS)
    ENABLE_SPI_TIMEOUT,         // EnableTimeout
    1000                        // TimeoutMs
>;

inline SlaveSPILogic spi_logic;

// ============================================
// Main Update
// ============================================

inline void init() {
    spi_logic.init(g_spi);
}

inline void update() {
    // Slave is always ready for SPI transfer when requested
    if (spi_logic.waiting_for_ready) {
        spi_logic.ready_for_transfer(); // Slave is always ready
    }
    spi_logic.update();
}

inline bool is_spi_connected() {
    return spi_connected;
}
} // namespace Communications

#endif // COMMUNICATIONS_HPP
