#include "serial_log.h"

#include <stdbool.h>
#include <stddef.h>

#include "app_config.h"
#include "app_time.h"
#include "encoder.h"
#include "motion.h"
#include "oled.h"
#include "ti_msp_dl_config.h"

typedef struct {
    char *buffer;
    uint16_t capacity;
    uint16_t length;
    bool overflow;
} Builder;

static uint8_t gTxBuffer[APP_UART_TX_BUFFER_SIZE];
static uint16_t gHead;
static uint16_t gTail;
static uint16_t gCount;
static uint32_t gDropped;
static uint32_t gLastReportMs;
static MotionState gLastState;

static void put_char(Builder *builder, char value)
{
    if ((builder == NULL) || builder->overflow) {
        return;
    }
    if (builder->length >= builder->capacity) {
        builder->overflow = true;
        return;
    }
    builder->buffer[builder->length++] = value;
}

static void put_text(Builder *builder, const char *text)
{
    while ((text != NULL) && (*text != '\0')) {
        put_char(builder, *text++);
    }
}

static void put_u32(Builder *builder, uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do {
        digits[count++] = (char)('0' + value % 10U);
        value /= 10U;
    } while ((value != 0U) && (count < sizeof(digits)));

    while (count != 0U) {
        put_char(builder, digits[--count]);
    }
}

static void put_i32(Builder *builder, int32_t value)
{
    if (value < 0) {
        put_char(builder, '-');
        put_u32(builder, (uint32_t)(-(value + 1)) + 1U);
    } else {
        put_u32(builder, (uint32_t)value);
    }
}

static const char *state_text(MotionState state)
{
    switch (state) {
    case MOTION_DISTANCE: return "DIST";
    case MOTION_TURN: return "TURN";
    case MOTION_BRAKING: return "BRAKE";
    case MOTION_DONE: return "DONE";
    case MOTION_TIMEOUT: return "TIMEOUT";
    case MOTION_ENCODER_FAULT: return "ENCFAULT";
    case MOTION_IDLE:
    default: return "IDLE";
    }
}

static bool enqueue(const char *data, uint16_t length)
{
    uint16_t index;

    if (length > (uint16_t)(APP_UART_TX_BUFFER_SIZE - gCount)) {
        gDropped++;
        return false;
    }

    for (index = 0U; index < length; index++) {
        gTxBuffer[gHead++] = (uint8_t)data[index];
        if (gHead >= APP_UART_TX_BUFFER_SIZE) {
            gHead = 0U;
        }
    }
    gCount = (uint16_t)(gCount + length);
    return true;
}

static void enqueue_builder(Builder *builder)
{
    if (builder->overflow) {
        gDropped++;
        return;
    }
    (void)enqueue(builder->buffer, builder->length);
}

static void queue_event(uint32_t now_ms, MotionState state)
{
    char line[80];
    Builder builder = {line, sizeof(line), 0U, false};

    put_text(&builder, "EVT,t=");
    put_u32(&builder, now_ms);
    put_text(&builder, ",state=");
    put_text(&builder, state_text(state));
    put_text(&builder, "\r\n");
    enqueue_builder(&builder);
}

static void queue_motion(uint32_t now_ms)
{
    char line[240];
    MotionDebug debug;
    Builder builder = {line, sizeof(line), 0U, false};

    motion_get_debug(&debug);
    put_text(&builder, "MOT,t="); put_u32(&builder, now_ms);
    put_text(&builder, ",st="); put_text(&builder, state_text(debug.state));
    put_text(&builder, ",lp="); put_i32(&builder, debug.left_progress);
    put_text(&builder, ",rp="); put_i32(&builder, debug.right_progress);
    put_text(&builder, ",tg="); put_i32(&builder, debug.target_counts);
    put_text(&builder, ",lr="); put_i32(&builder, debug.left_remaining);
    put_text(&builder, ",rr="); put_i32(&builder, debug.right_remaining);
    put_text(&builder, ",bp="); put_i32(&builder, debug.base_pwm);
    put_text(&builder, ",sp="); put_i32(&builder, debug.synchronization_pwm);
    put_text(&builder, ",pl="); put_i32(&builder, debug.left_pwm);
    put_text(&builder, ",pr="); put_i32(&builder, debug.right_pwm);
    put_text(&builder, ",dl="); put_i32(&builder, debug.left_delta);
    put_text(&builder, ",dr="); put_i32(&builder, debug.right_delta);
    put_text(&builder, ",ef="); put_u32(&builder, debug.encoder_fault_flags);
    put_text(&builder, "\r\n");
    enqueue_builder(&builder);
}

static void queue_system(uint32_t now_ms)
{
    char line[320];
    EncoderStatistics left;
    EncoderStatistics right;
    Builder builder = {line, sizeof(line), 0U, false};

    encoder_get_statistics(&left, &right);
    put_text(&builder, "SYS,t="); put_u32(&builder, now_ms);
    put_text(&builder, ",lc="); put_i32(&builder, left.count);
    put_text(&builder, ",rc="); put_i32(&builder, right.count);
    put_text(&builder, ",lv="); put_u32(&builder, left.valid_transition_count);
    put_text(&builder, ",li="); put_u32(&builder, left.invalid_transition_count);
    put_text(&builder, ",ld="); put_u32(&builder, left.duplicate_state_count);
    put_text(&builder, ",la="); put_u32(&builder, left.a_edge_count);
    put_text(&builder, ",lb="); put_u32(&builder, left.b_edge_count);
    put_text(&builder, ",lm="); put_u32(&builder, left.merged_edge_count);
    put_text(&builder, ",ls="); put_u32(&builder, left.ab_state);
    put_text(&builder, ",rv="); put_u32(&builder, right.valid_transition_count);
    put_text(&builder, ",ri="); put_u32(&builder, right.invalid_transition_count);
    put_text(&builder, ",rd="); put_u32(&builder, right.duplicate_state_count);
    put_text(&builder, ",ra="); put_u32(&builder, right.a_edge_count);
    put_text(&builder, ",rb="); put_u32(&builder, right.b_edge_count);
    put_text(&builder, ",rm="); put_u32(&builder, right.merged_edge_count);
    put_text(&builder, ",rs="); put_u32(&builder, right.ab_state);
    put_text(&builder, ",ol="); put_u32(&builder, oled_is_online() ? 1U : 0U);
    put_text(&builder, ",oe="); put_u32(&builder, oled_get_error_count());
    put_text(&builder, ",or="); put_u32(&builder, oled_get_reconnect_count());
    put_text(&builder, ",ov="); put_u32(&builder, app_time_get_overrun_count());
    put_text(&builder, ",qd="); put_u32(&builder, gDropped);
    put_text(&builder, "\r\n");
    enqueue_builder(&builder);
}

void serial_log_init(void)
{
    static const char boot[] =
        "BOOT,EncoderMotion rebuild v1.1,115200,8N1\r\n";

    gHead = 0U;
    gTail = 0U;
    gCount = 0U;
    gDropped = 0U;
    gLastReportMs = 0U;
    gLastState = motion_get_state();
    (void)enqueue(boot, (uint16_t)(sizeof(boot) - 1U));
    queue_event(0U, gLastState);
}

void serial_log_periodic(uint32_t now_ms)
{
    MotionState state = motion_get_state();

    if (state != gLastState) {
        gLastState = state;
        queue_event(now_ms, state);
    }

    if ((uint32_t)(now_ms - gLastReportMs) >= APP_SERIAL_PERIOD_MS) {
        gLastReportMs = now_ms;
        queue_motion(now_ms);
        queue_system(now_ms);
    }
}

void serial_log_service(void)
{
    while ((gCount != 0U) &&
           !DL_UART_Main_isTXFIFOFull(UART_DEBUG_INST)) {
        DL_UART_Main_transmitData(UART_DEBUG_INST, gTxBuffer[gTail++]);
        if (gTail >= APP_UART_TX_BUFFER_SIZE) {
            gTail = 0U;
        }
        gCount--;
    }
}

uint32_t serial_log_get_dropped_message_count(void)
{
    return gDropped;
}
