EncoderMotion v7 - latest-version repair package

Copy all files in this directory to the current GitHub project root and overwrite
files with the same names. This package intentionally does not replace motor.c,
motor.h, oled.c, oled.h or empty.syscfg.

Root causes repaired
====================
1. Current main mixed a new encoder snapshot implementation with the old speed-PI
   controller and an old queued-tick scheduler.
2. The old scheduler replayed delayed 5 ms updates without real time passing.
3. The raw quadrature decoder committed every valid quarter-step, so stationary
   contact bounce could form valid +1/-1 sequences and sometimes drift.
4. motion.c read left/right encoder values through separate critical sections.

Encoder repair
==============
- Keeps the 16-entry quadrature transition table.
- Illegal jumps resynchronise state but do not count.
- Valid quarter-steps first accumulate.
- Only a complete ordered four-transition AB cycle commits +/-4 counts.
- Short stationary bounce cancels inside the accumulator and does not change count.
- Existing 4x calibration remains valid because a complete cycle still adds 4.
- Left/right count and delta are captured atomically.
- ISR drains newly pending GPIO edges up to four passes.
- UART adds cycle, edge, coalesced-edge and accumulator diagnostics.

Control repair
==============
- Removes the uncalibrated nested speed PI loop.
- Uses deterministic position profile plus conservative P-only wheel synchronisation.
- Straight and in-place turn use separate parameters.
- Both wheels must reach the target before active braking.
- A wheel that reaches early is set to zero instead of being driven farther.
- Encoder fault timing uses real SysTick time with a one-second startup grace.

First validation
================
1. Keep the car lifted and motors stopped for 60 seconds.
   lc and rc should remain unchanged.
   la/lb/ra/rb may increase from electrical noise, but lac/rac should cancel and
   lcy/rcy should not increase unless a complete ordered AB cycle occurs.
2. Turn each wheel manually 50 revolutions.
   Expected absolute count is approximately 73450 counts, in multiples of 4.
3. Test 500 mm forward at PWM 300.
4. Test left and right 90 degree turns at PWM 250.

Important hardware requirement
==============================
Software cannot correct a floating reference indefinitely. Encoder GND, MCU GND,
TB6612 GND and battery negative must be connected reliably. Add 0.1 uF near each
encoder supply and use external 4.7-10 kOhm pull-ups to 3.3 V only when the encoder
outputs are confirmed open-drain/open-collector.

CCS steps
=========
1. Copy and overwrite all files.
2. Open and save empty.syscfg once.
3. Project -> Clean.
4. Build Project.
5. Flash and open UART0/XDS110 at 115200 8N1.
