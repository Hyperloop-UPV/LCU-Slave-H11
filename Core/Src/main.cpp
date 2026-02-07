#include "main.h"
#include "ST-LIB.hpp"
#include "LPU/LPU.hpp"
#include "Airgap/Airgap.hpp"
#include "ConfigShared.hpp"
#include "Pinout/Pinout.hpp"

using frame = SystemFrame<false>; // false for Slave

auto constexpr my_pwm_positive = ST_LIB::TimerPin({.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm1_1, .channel = ST_LIB::TimerChannel::CHANNEL_1});
auto constexpr my_pwm_negative = ST_LIB::TimerPin({.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm1_2, .channel = ST_LIB::TimerChannel::CHANNEL_2});
auto constexpr my_timer = ST_LIB::TimerDomain::Timer({.request = ST_LIB::TimerRequest::GeneralPurpose_15}, my_pwm_positive, my_pwm_negative);

int main(void) { 
  STLIB::start();
  Hard_fault_check();

  using myBoard = ST_LIB::Board<my_timer>;
  myBoard::init();

  auto my_tim = get_timer_instance(myBoard, my_timer);

  auto my_pwm1 = my_tim.get_pwm<my_pwm_positive>();
  auto my_pwm2 = my_tim.get_pwm<my_pwm_negative>();

  auto my_lpu = LPU(
    my_pwm1,
    my_pwm2,
    Pinout::shunt1,
    Pinout::vbat_1,
    0.0f, 1.0f, 0.0f, 1.0f
  );

  [[maybe_unused]] auto my_airgap = Airgap(Pinout::airgap_1, 0.0f, 1.0f);

  [[maybe_unused]] auto lpu_array = LpuArray(std::tie(my_lpu), std::tie(Pinout::en_buff_1));

  // Initialize frame with same array for both TX and RX
  frame::init(my_lpu, my_airgap, my_lpu);

  while (1) {
    STLIB::update();
  }
}

void Error_Handler(void) {
    ErrorHandler("HAL error handler triggered");
    while (1) {
    }
}

