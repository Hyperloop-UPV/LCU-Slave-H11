#ifndef COMMUNICATIONS_HPP
#define COMMUNICATIONS_HPP

#include "LCU_SLAVE_Types.hpp"
#include "StateMachine/LCU_StateMachine.hpp"
#include "ConfigShared.hpp"
#include "CommunicationsShared.hpp"

namespace Communications {

using Frame = LCU_Slave::Frame;

// Communications object to hold command/status packets
inline CommunicationsBase comms;

// SPI Wrapper (owned by Communications)
inline LCU_Slave::SpiType* g_spi = nullptr;
inline ST_LIB::DigitalOutputDomain::Instance* g_slave_ready = nullptr;

volatile bool send_flag = false;
volatile bool spi_flag = false;
volatile bool receive_flag = false;
volatile bool operation_flag = false;

uint32_t spi_error_counter = LCU_Slave::MAX_SPI_ERRORS; // Start with errors to force sync
uint32_t spi_timeout_counter = 0;

// ============================================
// Status Reporting
// ============================================

inline void update_status() {
    auto& status = comms.status_packet;

    // Ensure start/end bytes are correct
    status.start_byte = StatusPacket::START_BYTE;
    status.end_byte = StatusPacket::END_BYTE;

    status.control_state = static_cast<uint8_t>(LCU_SM::sm_operational.get_current_state());
    status.acks = comms.command_packet.commands;
}

// ============================================
// Main Update
// ============================================

inline void init() {
    Frame::init(comms, *LCU_Slave::g_lpu, *LCU_Slave::g_airgap, comms, *LCU_Slave::g_lpu);
    LCU_SM::set_command_packet(&comms.command_packet);
    LCU_SM::set_spi_error_counter_ptr(&spi_error_counter);
}

inline void update() {
    update_status();

    // Watchdog for SPI
    if (operation_flag) {
        spi_timeout_counter++;
        if (spi_timeout_counter > LCU_Slave::SPI_TIMEOUT_LIMIT) {
            spi_error_counter++;

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

    if (!operation_flag) {
        operation_flag = true;
        Frame::update_tx(&send_flag);
    } else if (send_flag) {
        send_flag = false;
        g_spi->transceive(Frame::tx_buffer, Frame::rx_buffer, &spi_flag);
        g_spi->set_software_nss(true);
        g_slave_ready->turn_on();
    } else if (spi_flag) {
        spi_flag = false;
        g_slave_ready->turn_off();
        g_spi->set_software_nss(false);
        Frame::update_rx(&receive_flag);
    } else if (receive_flag) {
        receive_flag = false;
        operation_flag = false;

        // Packet Validation
        auto& cmd = comms.command_packet;

        if (cmd.start_byte != CommandPacket::START_BYTE ||
            cmd.end_byte != CommandPacket::END_BYTE) {

            spi_error_counter++;
            // Invalid packet, ignore commands to prevent acting on garbage data
            comms.command_packet.commands = CommandFlags::NONE;

            if (spi_error_counter > LCU_Slave::MAX_SPI_ERRORS) {
                spi_error_counter = LCU_Slave::MAX_SPI_ERRORS;
            }
        } else {
            // Success
            if (spi_error_counter > 0)
                spi_error_counter--;
        }
    }
}
} // namespace Communications

#endif // COMMUNICATIONS_HPP
