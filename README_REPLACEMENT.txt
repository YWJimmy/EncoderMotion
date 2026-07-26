EncoderMotion replacement package v4 - wheel-speed PI inner loop

Prerequisite
============
Apply this package on top of EncoderMotion v3 UART telemetry. Copy every file in
this directory to the project root and overwrite files with the same name.

Files replaced
==============
app_config.h
motion.c
motion.h
serial_log.c
serial_log.h
empty.c
.gitignore

Implemented control structure
=============================
1. The position profile still determines the nominal command from remaining count.
2. The nominal command is converted to a target encoder speed in counts per 5 ms.
3. Left/right cumulative-position error changes the two wheel target speeds.
4. Each wheel has an independent speed PI controller.
5. PI output plus feed-forward becomes the actual PWM command.
6. Average wheel progress still determines when to brake.

Speed representation
====================
tl, tr, sl and sr in the MOT log use counts/control-tick x 16.
For example:
  sl=176 means 176/16 = 11 encoder counts per 5 ms.

New MOT fields
==============
tl/tr  left/right target speed x16
sl/sr  left/right filtered measured speed x16
cl/cr  left/right speed-PI PWM correction
sc     synchronization target-speed correction x16
ef     encoder-fault flags: bit0 left, bit1 right

Encoder fault protection
========================
When a wheel receives PWM >= 150 and has a meaningful target speed, but accumulates
less than 8 net forward counts during a 200 ms window, both motors stop and state
changes to ENCFAULT. This detects disconnected signals, bad ground, a single-phase
AB failure and a mechanically stalled wheel.

Initial tuning order
====================
1. Keep motion_start_distance_mm(..., 300).
2. Observe tl/tr and sl/sr during steady travel.
3. If both measured speeds are consistently lower than targets and cl/cr remain
   strongly positive, reduce APP_SPEED_FULL_SCALE_COUNTS_PER_TICK_X16.
4. If both measured speeds are consistently higher and cl/cr remain negative,
   increase APP_SPEED_FULL_SCALE_COUNTS_PER_TICK_X16.
5. If speed reacts too slowly, increase APP_SPEED_PI_KP_NUM slightly.
6. If a constant speed error remains, decrease APP_SPEED_PI_KI_DIV slightly.
7. If PWM oscillates, reduce Kp or increase Ki divisor and speed-filter divisor.

Recommended first test
======================
motion_start_distance_mm(3000, 300)

During the constant-speed section, save several MOT lines. A good initial result is:
- sl close to tl
- sr close to tr
- cl and cr not permanently at +/-250
- sc usually small
- ef remains 0
- ov and qd remain 0

CCS steps
=========
1. Copy and overwrite the files.
2. Refresh the project.
3. Project -> Clean.
4. Build Project.
5. Flash and open XDS110 UART at 115200 8N1.
