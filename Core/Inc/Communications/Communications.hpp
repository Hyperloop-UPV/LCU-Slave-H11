#ifndef COMMUNICATIONS_HPP
#define COMMUNICATIONS_HPP

#include "LCU_SLAVE_Types.hpp"
#include "StateMachine/LCU_StateMachine.hpp"
#include "ConfigShared.hpp"
#include "CommunicationsShared.hpp"

namespace Communications {

// Communications object to hold command/status packets
inline CommunicationsBase comms;

// SPI
inline LCU_Slave::SpiType* g_spi = nullptr;
inline ST_LIB::DigitalOutputDomain::Instance* g_slave_ready = nullptr;

// Inner State Machine flags
volatile bool send_flag = false;
volatile bool spi_flag = false;
volatile bool receive_flag = false;
volatile bool operation_flag = false;

// Error handling
#ifdef USE_SPI_ERROR
uint32_t spi_error_counter = LCU_Slave::MAX_SPI_ERRORS; // Start with errors to force sync
#ifdef USE_SPI_TIMEOUT
uint32_t spi_timeout_counter = 0;
#endif // USE_SPI_TIMEOUT
#endif // USE_SPI_ERROR


// ============================================
// Status Reporting
// ============================================

inline void update_status() {
    auto& status = comms.status_packet;
    status.control_state = static_cast<uint8_t>(LCU_SM::sm_operational.get_current_state());
}


// ============================================
// Main Update
// ============================================

inline void init() {
    LCU_Slave::Frame::init(comms, *LCU_Slave::g_lpu, *LCU_Slave::g_airgap, comms, *LCU_Slave::g_lpu);
    LCU_SM::set_command_packet(&comms.command_packet);
    #ifdef USE_SPI_ERROR
    LCU_SM::set_spi_error_counter_ptr(&spi_error_counter);
    #endif
}

inline void update() {
    update_status();

    #ifdef USE_SPI_ERROR
    if (operation_flag) {
        if (g_spi->was_aborted()) {
            g_spi->clear_abort_flag();
            spi_error_counter++;
            operation_flag = false;
            send_flag = false;
            spi_flag = false;
            receive_flag = false;
            g_slave_ready->turn_off();
            g_spi->set_software_nss(false);
        }
    }
    
    #ifdef USE_SPI_TIMEOUT
    if (operation_flag) {
        spi_timeout_counter++;
        if (spi_timeout_counter > LCU_Slave::SPI_TIMEOUT_LIMIT) {
            spi_error_counter++;
            spi_timeout_counter = 0;

            // Reset state machine (SPI is already reset on error)
            operation_flag = false;
            send_flag = false;
            spi_flag = false;
            receive_flag = false;
            g_slave_ready->turn_off();
            g_spi->set_software_nss(false);
        }
    } else {
        spi_timeout_counter = 0;
    }
    #endif // USE_SPI_TIMEOUT
    #endif // USE_SPI_ERROR

    if (!operation_flag) {
        operation_flag = true;
        LCU_Slave::Frame::update_tx(&send_flag);

    } else if (send_flag) {
        send_flag = false;
        g_spi->transceive(LCU_Slave::Frame::tx_buffer, LCU_Slave::Frame::rx_buffer, &spi_flag);
        g_spi->set_software_nss(true);
        g_slave_ready->turn_on();

    } else if (spi_flag) {
        spi_flag = false;
        g_slave_ready->turn_off();
        g_spi->set_software_nss(false);

        #ifdef USE_SPI_ERROR
        // Preemptive packet validation
        if (((LCU_Slave::Frame::rx_buffer[1] << 8) + LCU_Slave::Frame::rx_buffer[0]) != CommandPacket::START_BYTE) {
            spi_error_counter++;

            if (spi_error_counter > LCU_Slave::MAX_SPI_ERRORS) {
                spi_error_counter = LCU_Slave::MAX_SPI_ERRORS;
            }
            operation_flag = false; // Reset state machine on error
        } else {
            // Success
            LCU_Slave::Frame::update_rx(&receive_flag);
            if (spi_error_counter > 0)
                spi_error_counter--;
        }
        #else
        LCU_Slave::Frame::update_rx(&receive_flag);
        #endif

        
    } else if (receive_flag) {
        receive_flag = false;
        operation_flag = false;

        #ifdef USE_SPI_ERROR
        // Packet Validation
        auto& cmd = comms.command_packet;

        if (cmd.start_byte != CommandPacket::START_BYTE ||
            cmd.end_byte != CommandPacket::END_BYTE) {

            spi_error_counter++;

            if (spi_error_counter > LCU_Slave::MAX_SPI_ERRORS) {
                spi_error_counter = LCU_Slave::MAX_SPI_ERRORS;
            }
        } else {
            // Success
            if (spi_error_counter > 0)
                spi_error_counter--;
        }
        #endif
    }
}
} // namespace Communications

#endif // COMMUNICATIONS_HPP
