#include "serial_log.h"

#include <stdbool.h>
#include <stddef.h>

#include "app_config.h"
#include "app_tick.h"
#include "encoder.h"
#include "motion.h"
#include "oled.h"
#include "ti_msp_dl_config.h"

typedef struct {
    char *data;
    uint16_t capacity;
    uint16_t length;
    bool overflow;
} TextBuilder;

static uint8_t gTxBuffer[APP_SERIAL_TX_BUFFER_SIZE];
static uint16_t gTxHead;
static uint16_t gTxTail;
static uint16_t gTxCount;
static uint32_t gDroppedMessageCount;
static uint32_t gLastReportTick;
static MotionState gLastMotionState;

static void builder_put_char(TextBuilder *builder, char character)
{
    if ((builder == NULL) || builder->overflow) {
        return;
    }
    if (builder->length >= builder->capacity) {
        builder->overflow = true;
        return;
    }
    builder->data[builder->length++] = character;
}

static void builder_put_string(TextBuilder *builder, const char *text)
{
    if (text == NULL) {
        return;
    }
    while (*text != '\0') {
        builder_put_char(builder, *text++);
    }
}

static void builder_put_u32(TextBuilder *builder, uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (count < (uint8_t)sizeof(digits)));

    while (count != 0U) {
        builder_put_char(builder, digits[--count]);
    }
}

static void builder_put_i32(TextBuilder *builder, int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        builder_put_char(builder, '-');
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }
    builder_put_u32(builder, magnitude);
}

static const char *motion_state_text(MotionState state)
{
    switch (state) {
    case MOTION_RUNNING_DISTANCE:
        return "DIST";
    case MOTION_RUNNING_TURN:
        return "TURN";
    case MOTION_BRAKING:
        return "BRAKE";
    case MOTION_DONE:
        return "DONE";
    case MOTION_TIMEOUT:
        return "TIMEOUT";
    case MOTION_ENCODER_FAULT:
        return "ENCFAULT";
    case MOTION_IDLE:
    default:
        return "IDLE";
    }
}

static uint16_t tx_free_space(void)
{
    return (uint16_t)(APP_SERIAL_TX_BUFFER_SIZE - gTxCount);
}

static bool enqueue_message(const char *data, uint16_t length)
{
    uint16_t index;

    if ((data == NULL) || (length == 0U)) {
        return true;
    }
    if (length > tx_free_space()) {
        gDroppedMessageCount++;
        return false;
    }

    for (index = 0U; index < length; index++) {
        gTxBuffer[gTxHead] = (uint8_t)data[index];
        gTxHead++;
        if (gTxHead >= APP_SERIAL_TX_BUFFER_SIZE) {
            gTxHead = 0U;
        }
    }

    gTxCount = (uint16_t)(gTxCount + length);
    return true;
}

static void enqueue_builder(TextBuilder *builder)
{
    if ((builder == NULL) || builder->overflow) {
        gDroppedMessageCount++;
        return;
    }
    (void)enqueue_message(builder->data, builder->length);
}

static void queue_event(uint32_t now_tick, MotionState state)
{
    char line[96];
    TextBuilder builder = {line, (uint16_t)sizeof(line), 0U, false};

    builder_put_string(&builder, "EVT,t=");
    builder_put_u32(&builder, now_tick * APP_CONTROL_PERIOD_MS);
    builder_put_string(&builder, ",state=");
    builder_put_string(&builder, motion_state_text(state));
    builder_put_string(&builder, "\r\n");
    enqueue_builder(&builder);
}

static void queue_motion_line(uint32_t now_tick)
{
    char line[288];
    MotionDebugData debug;
    TextBuilder builder = {line, (uint16_t)sizeof(line), 0U, false};

    motion_get_debug(&debug);

    builder_put_string(&builder, "MOT,t=");
    builder_put_u32(&builder, now_tick * APP_CONTROL_PERIOD_MS);
    builder_put_string(&builder, ",st=");
    builder_put_string(&builder, motion_state_text(motion_get_state()));
    builder_put_string(&builder, ",lp=");
    builder_put_i32(&builder, debug.left_progress);
    builder_put_string(&builder, ",rp=");
    builder_put_i32(&builder, debug.right_progress);
    builder_put_string(&builder, ",tg=");
    builder_put_i32(&builder, debug.target_count);
    builder_put_string(&builder, ",rm=");
    builder_put_i32(&builder, debug.remaining_count);
    builder_put_string(&builder, ",bp=");
    builder_put_i32(&builder, debug.base_pwm);
    builder_put_string(&builder, ",tl=");
    builder_put_i32(&builder, debug.left_target_speed_x16);
    builder_put_string(&builder, ",tr=");
    builder_put_i32(&builder, debug.right_target_speed_x16);
    builder_put_string(&builder, ",sl=");
    builder_put_i32(&builder, debug.left_measured_speed_x16);
    builder_put_string(&builder, ",sr=");
    builder_put_i32(&builder, debug.right_measured_speed_x16);
    builder_put_string(&builder, ",cl=");
    builder_put_i32(&builder, debug.left_speed_pi_pwm);
    builder_put_string(&builder, ",cr=");
    builder_put_i32(&builder, debug.right_speed_pi_pwm);
    builder_put_string(&builder, ",sc=");
    builder_put_i32(&builder, debug.synchronization_speed_x16);
    builder_put_string(&builder, ",pl=");
    builder_put_i32(&builder, debug.left_pwm);
    builder_put_string(&builder, ",pr=");
    builder_put_i32(&builder, debug.right_pwm);
    builder_put_string(&builder, ",ef=");
    builder_put_u32(&builder, debug.encoder_fault_flags);
    builder_put_string(&builder, "\r\n");
    enqueue_builder(&builder);
}

static void queue_system_line(uint32_t now_tick)
{
    char line[256];
    EncoderStatistics left;
    EncoderStatistics right;
    TextBuilder builder = {line, (uint16_t)sizeof(line), 0U, false};

    encoder_get_statistics(&left, &right);

    builder_put_string(&builder, "SYS,t=");
    builder_put_u32(&builder, now_tick * APP_CONTROL_PERIOD_MS);
    builder_put_string(&builder, ",lc=");
    builder_put_i32(&builder, left.count);
    builder_put_string(&builder, ",rc=");
    builder_put_i32(&builder, right.count);
    builder_put_string(&builder, ",lv=");
    builder_put_u32(&builder, left.valid_transition_count);
    builder_put_string(&builder, ",li=");
    builder_put_u32(&builder, left.invalid_transition_count);
    builder_put_string(&builder, ",ld=");
    builder_put_u32(&builder, left.duplicate_state_count);
    builder_put_string(&builder, ",rv=");
    builder_put_u32(&builder, right.valid_transition_count);
    builder_put_string(&builder, ",ri=");
    builder_put_u32(&builder, right.invalid_transition_count);
    builder_put_string(&builder, ",rd=");
    builder_put_u32(&builder, right.duplicate_state_count);
    builder_put_string(&builder, ",ol=");
    builder_put_u32(&builder, oled_is_online() ? 1U : 0U);
    builder_put_string(&builder, ",oe=");
    builder_put_u32(&builder, oled_get_error_count());
    builder_put_string(&builder, ",or=");
    builder_put_u32(&builder, oled_get_reconnect_count());
    builder_put_string(&builder, ",ov=");
    builder_put_u32(&builder, app_tick_get_overrun_count());
    builder_put_string(&builder, ",qd=");
    builder_put_u32(&builder, gDroppedMessageCount);
    builder_put_string(&builder, "\r\n");
    enqueue_builder(&builder);
}

void serial_log_init(void)
{
    static const char boot_message[] =
        "BOOT,EncoderMotion speed-PI telemetry,115200,8N1\r\n";

    gTxHead = 0U;
    gTxTail = 0U;
    gTxCount = 0U;
    gDroppedMessageCount = 0U;
    gLastReportTick = 0U;
    gLastMotionState = motion_get_state();

    (void)enqueue_message(
        boot_message,
        (uint16_t)(sizeof(boot_message) - 1U));
    queue_event(0U, gLastMotionState);
}

void serial_log_task(uint32_t now_tick)
{
    MotionState state = motion_get_state();

    if (state != gLastMotionState) {
        gLastMotionState = state;
        queue_event(now_tick, state);
    }

    if ((uint32_t)(now_tick - gLastReportTick) <
        APP_SERIAL_REPORT_PERIOD_TICKS) {
        return;
    }

    gLastReportTick = now_tick;
    queue_motion_line(now_tick);
    queue_system_line(now_tick);
}

void serial_log_service(void)
{
    while ((gTxCount != 0U) &&
           !DL_UART_Main_isTXFIFOFull(UART_DEBUG_INST)) {
        DL_UART_Main_transmitData(UART_DEBUG_INST, gTxBuffer[gTxTail]);
        gTxTail++;
        if (gTxTail >= APP_SERIAL_TX_BUFFER_SIZE) {
            gTxTail = 0U;
        }
        gTxCount--;
    }
}

uint32_t serial_log_get_dropped_message_count(void)
{
    return gDroppedMessageCount;
}

uint16_t serial_log_get_queued_byte_count(void)
{
    return gTxCount;
}
