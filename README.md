# SpindleETH
Intended for lathe spindle control by Ethernet.
No axes control.

# Hardware
This repository is for STM32F103C8 or STM32F103CB and W5500 boards.
The connection wires between STM32 and W5500 must be as short as possible.

# Tests
Tested on real machine with 2500 p/r encoder and ~1500 rpm.
Direct to PC's eth interface or thru switch (some switches may not work).

# Improvements
New index_enable philosofy.
The encoder's index signal (mostly named Z) is not transmitted to HAL component (it causes delays and threatens to lose a signal).
Controller sends raw counts and raw counts latched at every index pulse instead.
The component knows the index occured by comparing current and previous latched values.

# Adds
1. PWM output (1 kHz and 16 bit resolution)
2. Digital I/O

# Tools
Added encoder emulator for tests.
Important: Arduino can't turn off ESP32C3 watchdog and resets chips after every two seconds.
If it is problem for you, rewrite code for esp-idf or use other microprocessor.

# Other info
Read comments at the tops of source files.
