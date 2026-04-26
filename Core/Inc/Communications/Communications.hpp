#ifndef COMMUNICATIONS_HPP
#define COMMUNICATIONS_HPP

#include "ConfigShared.hpp"
#include "CommunicationsShared.hpp"

namespace Communications {

// Communications object to hold command/status packets
inline CommunicationsBase comms{};

// SPI

// Inner State Machine flags
inline volatile bool send_flag = false;
inline volatile bool spi_flag = false;
inline volatile bool receive_flag = false;
inline volatile bool operation_flag = false;

// Error handling
#ifdef USE_SPI_ERROR
inline uint32_t spi_error_counter = LCU_Slave::MAX_SPI_ERRORS; // Start with errors to force sync
#ifdef USE_SPI_TIMEOUT
inline uint32_t spi_timeout_counter = 0;
#endif // USE_SPI_TIMEOUT
#endif // USE_SPI_ERROR

/**
 * API
 */

void update_status();
void init();
void update();

} // namespace Communications

#endif // COMMUNICATIONS_HPP
