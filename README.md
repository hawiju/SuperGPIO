# SuperGPIO

C++17 driver for STM32 F4/F7 GPIO

Requires "stm32f4xx.h" or "stm32f7xx.h" files

-------------

## Usage

```C++
#include "supergpio.hpp"

gpio::init<9, 10>(GPIOF, gpio::mode::output, gpio::type::PP);

gpio::toggle<9, 10>(GPIOF);

using namespace gpio;

gpio::init<0, 1, 2, 3>(GPIOF, mode::output_af, type::PP, speed::very_high, pupd::nopupd, af::af12);
```
