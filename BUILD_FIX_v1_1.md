# EncoderMotion v1.1 build correction

## Corrected issue

The original rebuilt project placed OLED PB8/PB9, select button PA18, and confirm button PB21 in one SysConfig GPIO instance named `USER_IO`.
Because those pins span GPIOA and GPIOB, SysConfig correctly did not generate one shared `USER_IO_PORT` macro. `button.c` and `oled.c` incorrectly expected that macro, causing the build errors.

## Correction

`empty.syscfg` now uses three single-port instances:

- `OLED_IO`: PB8/PB9
- `BUTTON_SELECT`: PA18
- `BUTTON_CONFIRM`: PB21

The sources now use the generated macros:

- `OLED_IO_PORT`, `OLED_IO_SCL_PIN`, `OLED_IO_SDA_PIN`
- `BUTTON_SELECT_PORT`, `BUTTON_SELECT_INPUT_PIN`
- `BUTTON_CONFIRM_PORT`, `BUTTON_CONFIRM_INPUT_PIN`

No motion-control or encoder-decoding behavior was changed in this build correction.
