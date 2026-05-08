#include "Communications/Communications.hpp"
#include "LCU_SLAVE.hpp"
#include "StateMachine/LCU_StateMachine.hpp"

namespace Communications {

void update_status() {
    auto& status = comms.status_packet;
    status.slave_state = LCU_SM::sm_operational.get_current_state();
}

void init() {
    LCU_SM::set_command_packet(&comms.command_packet);
    LCU_SM::set_status_packet(&comms.status_packet);
#ifdef USE_SPI_ERROR
    LCU_SM::set_spi_error_counter_ptr(&spi_error_counter);
#endif
}

void update() {
    update_status();

#ifdef USE_SPI_ERROR
    if (operation_flag) {
        if (LCU_Slave::g_spi->was_aborted()) {
            LCU_Slave::g_spi->clear_abort_flag();
            spi_error_counter++;
            operation_flag = false;
            send_flag = false;
            spi_flag = false;
            receive_flag = false;
            LCU_Slave::slave_ready.turn_off();
            LCU_Slave::spi.set_software_nss(false);
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
            LCU_Slave::slave_ready.turn_off();
            LCU_Slave::spi.set_software_nss(false);
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
        LCU_Slave::spi
            .transceive(LCU_Slave::Frame::tx_buffer, LCU_Slave::Frame::rx_buffer, &spi_flag);
        LCU_Slave::spi.set_software_nss(true);
        LCU_Slave::slave_ready.turn_on();

    } else if (spi_flag) {
        spi_flag = false;
        LCU_Slave::slave_ready.turn_off();
        LCU_Slave::spi.set_software_nss(false);

#ifdef USE_SPI_ERROR
        // Preemptive packet validation
        if (((LCU_Slave::Frame::rx_buffer[1] << 8) + LCU_Slave::Frame::rx_buffer[0]) !=
            CommandPacket::START_BYTE) {
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

} // Namespace Communications
