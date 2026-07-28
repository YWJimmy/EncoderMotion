EncoderMotion direction repair replacement
==========================================

Overwrite these three files in the project root:
- app_config.h
- motor.c
- motor.h

What is fixed
-------------
1. Positive commands now drive both physical wheels in vehicle-forward direction.
2. The mirror-mounted right motor direction is inverted inside motor.c.
3. Encoder signs are reversed so both counts increase during vehicle-forward travel.
4. UART pl/pr remain logical vehicle-coordinate commands; right pin inversion is hidden.
5. No wheel trim is changed because the previous distance difference was measured while
   the two wheels were physically rotating in opposite directions.

After replacement
-----------------
1. Delete Debug/.
2. Project -> Clean.
3. Build Project and flash.
4. Lift the chassis for the first test.
5. Manually rotate each wheel in vehicle-forward direction: lc and rc must increase.
6. Start FWD 500MM: lp and rp must both increase toward tg=3542.

Expected state sequence:
DIST -> BRAKE -> DONE

Do not swap the right motor wires after applying this software fix. Doing both would
cancel the correction.
