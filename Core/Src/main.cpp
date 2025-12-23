#include "main.h"
#include "ST-LIB.hpp"
#include "SEGGER_RTT.h"

int main(void) {
#ifdef SIM_ON
    SharedMemory::start();
#endif

    DigitalOutput led_on(PB0);
    STLIB::start();
    SEGGER_RTT_Init();
    SEGGER_RTT_WriteString(0, "Hello from SEGGER RTT\r\n\r\n") ;              

    Time::register_low_precision_alarm(500, [&]() { led_on.toggle(); 
    });

    while (1) {
        STLIB::update();
    }
}

void Error_Handler(void) {
    ErrorHandler("HAL error handler triggered");
    while (1) {
    }
}
