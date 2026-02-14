#include "main.h"
#include "LCU_SLAVE.hpp"

int main(void) {
    // Hard_fault_check();

    LCU_Slave::init();

    while (1) {
        LCU_Slave::update();
    }
}

void Error_Handler(void) {
    ErrorHandler("HAL error handler triggered");
    while (1) {
    }
}
