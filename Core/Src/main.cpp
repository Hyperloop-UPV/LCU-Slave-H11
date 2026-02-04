#include "main.h"
#include "ST-LIB.hpp"
#include "LPU/LPU.hpp"
#include "ConfigShared.hpp"

using frame = SystemFrame<false>; // false for Slave

LPU my_lpu;



int main(void) { 
  frame::init(
    my_lpu, my_lpu,       // Tx Parts
    my_lpu                // Rx Parts
  );
  
  Hard_fault_check();
  STLIB::start();

  using myBoard = ST_LIB::Board<>;
  myBoard::init();

  while (1) {
    STLIB::update();
  }
}
void Error_Handler(void) {
    ErrorHandler("HAL error handler triggered");
    while (1) {
    }
}

