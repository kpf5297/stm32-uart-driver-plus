#include "fault_module.h"
#include <string.h>
#include <stdio.h>

FaultStatus fault_state;

static const char *fault_strings[] = {
    "NONE",
    "OVERCURRENT",
    "OVERVOLTAGE",
    "UNDERVOLTAGE",
    "STARTUP_FAILED",
    "COMMUNICATION",
    "SENSOR",
    "ENCODER_ERROR",
    "CONTROL_LOOP_ERROR"
};

static Timestamp ts_now(void)
{
    TickType_t ticks = xTaskGetTickCount();
    Timestamp ts;
    ts.seconds = ticks / 1000;
    ts.subseconds = ticks % 1000;
    return ts;
}

void fault_init(void)
{
    memset(&fault_state, 0, sizeof(fault_state));
}

void fault_raise(FaultCode code)
{
    if (code <= FAULT_NONE || code >= FAULT_COUNT) return;
    taskENTER_CRITICAL();
    fault_state.active_mask |= (1u << code);
    fault_state.last_set[code] = ts_now();
    taskEXIT_CRITICAL();
    log_write(LOG_LEVEL_ERROR, "Fault raised: %s", fault_to_string(code));
}

void fault_clear(FaultCode code)
{
    if (code <= FAULT_NONE || code >= FAULT_COUNT) return;
    taskENTER_CRITICAL();
    fault_state.active_mask &= ~(1u << code);
    taskEXIT_CRITICAL();
}

bool fault_is_active(FaultCode code)
{
    if (code <= FAULT_NONE || code >= FAULT_COUNT) return false;
    return (fault_state.active_mask & (1u << code)) != 0;
}

void fault_clear_all(void)
{
    taskENTER_CRITICAL();
    fault_state.active_mask = 0;
    taskEXIT_CRITICAL();
}

const char* fault_to_string(FaultCode code)
{
    if (code >= FAULT_COUNT) return "UNKNOWN";
    return fault_strings[code];
}
