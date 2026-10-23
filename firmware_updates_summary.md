> **Historical document:** This describes an earlier UART-based firmware revision. The current firmware uses CAN and FreeRTOS with a configured 200 ms command timeout. See [README.md](README.md) for the current implementation and open validation findings.

# STM32 Firmware Updates Summary

This document summarizes the codebase review and the subsequent fixes applied to the `encoder_reading` (Rear STM32 Motor Controller) firmware.

## 1. Safety & Robustness Improvements
* **UART Angle Validation (Critical):** Added range checking (`50°` to `130°`) for the parsed angle command. This prevents the motors from reacting to garbage data or noise on the UART line.
* **UART Error Recovery (Critical):** Added `HAL_UART_ErrorCallback` to automatically clear UART error flags (overrun, noise, framing) and re-enable reception. This prevents the system from permanently losing communication after a single glitch.
* **Reduced Safety Timeout (High):** Reduced the `ANGLE_TIMEOUT_MS` from 30 seconds to 2 seconds. The motors will now stop safely if the steering STM32 stops sending data for 2 seconds.
* **Thread-Safe Variable Access (High):** Added a critical section (`__disable_irq()` / `__enable_irq()`) when reading the angle commands in the main loop to prevent race conditions with the UART interrupt handler.

## 2. Control System (PID) Refactoring
* **Struct-Based PID:** Consolidated the duplicated left and right PID variables into a single `PID_t` structure.
* **Unified PID Function:** Replaced duplicated update functions with a reusable `PID_Update` function.
* **Derivative on Measurement:** Modified the derivative calculation to use `-(meas - prev_meas)/Ts` instead of `(error - prev_error)/Ts`. This prevents a sudden "kick" in motor command when the target speed (setpoint) changes abruptly.

## 3. Signal & Hardware Improvements
* **Speed Low-Pass Filter:** Added a first-order IIR filter ($\alpha = 0.3$) on the calculated wheel speeds. This reduces quantization noise from the encoders, especially at low speeds, leading to smoother PID responses.
* **Increased PWM Frequency:** Changed the TIM4 prescaler and period (both in `main.c` and `encoder_reading.ioc`) to run the motor PWM at `10 kHz` (up from `1 kHz`). This eliminates audible high-pitched motor whine.

## 4. Code Cleanup
* **Motor Command Refactoring:** Unified the left and right motor command functions into a single parameterised `Motor_SetCommand` function.
* **Dead Code Removal:** Removed unreachable code (`ZERO_REF_EPS` check) that was never executed because the reference speed could never be zero while the angle was valid.
* **Documentation:** Added clear comments documenting the encoder PPR derivation (`390 lines × 4 = 1560`).

## 5. Performance Optimizations
* **Clock Speed Boost (64 MHz):** Enabled and configured the hardware PLL to multiply the internal 8 MHz HSI clock to `64 MHz`. This speeds up all math and logic processing by a factor of 8x without changing external hardware.
* **Non-Blocking Debug Output:** Migrated the UART2 test prints from `HAL_UART_Transmit` (blocking) to `HAL_UART_Transmit_IT` (interrupt-driven). This eliminates a ~7ms latency penalty in the critical 10ms PID control loop.
* **Floating-Point Arithmetic Fast-Path:** Converted slow hardware-emulated CPU `float` division (`/ Ts`) into much faster `float` multiplication using the reciprocal variable (`* INV_TS_SEC`).

## Next Steps
Please build the project in STM32CubeIDE and flash it to the microcontroller to verify the new behavior, particularly the 10 kHz PWM and the 2-second timeout safety feature.
