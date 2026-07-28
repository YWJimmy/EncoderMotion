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
} Builder;

static uint8_t gTxBuffer[APP_SERIAL_TX_BUFFER_SIZE];
static uint16_t gHead;
static uint16_t gTail;
static uint16_t gCount;
static uint32_t gDropped;
static uint32_t gLastReportTick;
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
    builder->data[builder->length++] = value;
}

static void put_string(Builder *builder, const char *text)
{
    if (text == NULL) {
        return;
    }
    while (*text != '\0') {
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
    uint32_t magnitude;

    if (value < 0) {
        put_char(builder, '-');
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }
    put_u32(builder, magnitude);
}

static const char *state_text(MotionState state)
{
    switch (state) {
    case MOTION_RUNNING_DISTANCE: return "DIST";
    case MOTION_RUNNING_TURN: return "TURN";
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

    if (length > (uint16_t)(APP_SERIAL_TX_BUFFER_SIZE - gCount)) {
        gDropped++;
        return false;
    }
    for (index = 0U; index < length; index++) {
        gTxBuffer[gHead] = (uint8_t)data[index];
        gHead = (uint16_t)((gHead + 1U) % APP_SERIAL_TX_BUFFER_SIZE);
    }
    gCount = (uint16_t)(gCount + length);
    return true;
}

static void submit(Builder *builder)
{
    if ((builder == NULL) || builder->overflow) {
        gDropped++;
        return;
    }
    (void)enqueue(builder->data, builder->length);
}

static void queue_event(uint32_t tick, MotionState state)
{
    char line[80];
    Builder builder = {line, sizeof(line), 0U, false};

    put_string(&builder, "EVT,t=");
    put_u32(&builder, tick * APP_CONTROL_PERIOD_MS);
    put_string(&builder, ",state=");
    put_string(&builder, state_text(state));
    put_string(&builder, "\r\n");
    submit(&builder);
}

static void queue_motion(uint32_t tick)
{
    char line[256];
    MotionDebugData debug;
    Builder builder = {line, sizeof(line), 0U, false};

    motion_get_debug(&debug);
    put_string(&builder, "MOT,t=");
    put_u32(&builder, tick * APP_CONTROL_PERIOD_MS);
    put_string(&builder, ",st=");
    put_string(&builder, state_text(motion_get_state()));
    put_string(&builder, ",lp="); put_i32(&builder, debug.left_progress);
    put_string(&builder, ",rp="); put_i32(&builder, debug.right_progress);
    put_string(&builder, ",tg="); put_i32(&builder, debug.target_count);
    put_string(&builder, ",lr="); put_i32(&builder, debug.left_remaining);
    put_string(&builder, ",rr="); put_i32(&builder, debug.right_remaining);
    put_string(&builder, ",bp="); put_i32(&builder, debug.base_pwm);
    put_string(&builder, ",sp="); put_i32(&builder, debug.synchronization_pwm);
    put_string(&builder, ",pl="); put_i32(&builder, debug.left_pwm);
    put_string(&builder, ",pr="); put_i32(&builder, debug.right_pwm);
    put_string(&builder, ",dl="); put_i32(&builder, debug.left_delta);
    put_string(&builder, ",dr="); put_i32(&builder, debug.right_delta);
    put_string(&builder, ",dt="); put_u32(&builder, debug.elapsed_control_ticks);
    put_string(&builder, ",ef="); put_u32(&builder, debug.encoder_fault_flags);
    put_string(&builder, "\r\n");
    submit(&builder);
}

static void queue_system(uint32_t tick)
{
    char line[320];
    EncoderStatistics left;
    EncoderStatistics right;
    Builder builder = {line, sizeof(line), 0U, false};

    encoder_get_statistics(&left, &right);
    put_string(&builder, "SYS,t=");
    put_u32(&builder, tick * APP_CONTROL_PERIOD_MS);
    put_string(&builder, ",lc="); put_i32(&builder, left.count);
    put_string(&builder, ",rc="); put_i32(&builder, right.count);
    put_string(&builder, ",lv="); put_u32(&builder, left.valid_transition_count);
    put_string(&builder, ",lcy="); put_u32(&builder, left.committed_cycle_count);
    put_string(&builder, ",li="); put_u32(&builder, left.invalid_transition_count);
    put_string(&builder, ",ld="); put_u32(&builder, left.duplicate_state_count);
    put_string(&builder, ",la="); put_u32(&builder, left.a_edge_count);
    put_string(&builder, ",lb="); put_u32(&builder, left.b_edge_count);
    put_string(&builder, ",lx="); put_u32(&builder, left.coalesced_edge_count);
    put_string(&builder, ",ls="); put_u32(&builder, left.ab_state);
    put_string(&builder, ",lac="); put_i32(&builder, left.transition_accumulator);
    put_string(&builder, ",rv="); put_u32(&builder, right.valid_transition_count);
    put_string(&builder, ",rcy="); put_u32(&builder, right.committed_cycle_count);
    put_string(&builder, ",ri="); put_u32(&builder, right.invalid_transition_count);
    put_string(&builder, ",rd="); put_u32(&builder, right.duplicate_state_count);
    put_string(&builder, ",ra="); put_u32(&builder, right.a_edge_count);
    put_string(&builder, ",rb="); put_u32(&builder, right.b_edge_count);
    put_string(&builder, ",rx="); put_u32(&builder, right.coalesced_edge_count);
    put_string(&builder, ",rs="); put_u32(&builder, right.ab_state);
    put_string(&builder, ",rac="); put_i32(&builder, right.transition_accumulator);
    put_string(&builder, ",ol="); put_u32(&builder, oled_is_online() ? 1U : 0U);
    put_string(&builder, ",oe="); put_u32(&builder, oled_get_error_count());
    put_string(&builder, ",or="); put_u32(&builder, oled_get_reconnect_count());
    put_string(&builder, ",ov="); put_u32(&builder, app_tick_get_overrun_count());
    put_string(&builder, ",qd="); put_u32(&builder, gDropped);
    put_string(&builder, "\r\n");
    submit(&builder);
}

void serial_log_init(void)
{
    static const char boot[] =
        "BOOT,EncoderMotion v7 stable encoder,115200,8N1\r\n";

    gHead = 0U;
    gTail = 0U;
    gCount = 0U;
    gDropped = 0U;
    gLastReportTick = 0U;
    gLastState = motion_get_state();
    (void)enqueue(boot, sizeof(boot) - 1U);
    queue_event(0U, gLastState);
}

void serial_log_task(uint32_t now_tick)
{
    MotionState state = motion_get_state();

    if (state != gLastState) {
        gLastState = state;
        queue_event(now_tick, state);
    }
    if ((uint32_t)(now_tick - gLastReportTick) <
        APP_SERIAL_REPORT_PERIOD_TICKS) {
        return;
    }
    gLastReportTick = now_tick;
    queue_motion(now_tick);
    queue_system(now_tick);
}

void serial_log_service(void)
{
    while ((gCount != 0U) &&
           !DL_UART_Main_isTXFIFOFull(UART_DEBUG_INST)) {
        DL_UART_Main_transmitData(UART_DEBUG_INST, gTxBuffer[gTail]);
        gTail = (uint16_t)((gTail + 1U) % APP_SERIAL_TX_BUFFER_SIZE);
        gCount--;
    }
}

uint32_t serial_log_get_dropped_message_count(void)
{
    return gDropped;
}

uint16_t serial_log_get_queued_byte_count(void)
{
    return gCount;
}
