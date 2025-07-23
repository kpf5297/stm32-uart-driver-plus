#include "fault_module.h"
#include <string.h>
#include <stdio.h>
#include "uart_driver_config.h"

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

static Timestamp get_current_timestamp_local(void)
{
    uint32_t ticks = GET_TICKS();
    Timestamp ts;
    ts.seconds    = ticks / TICKS_PER_SECOND;
    ts.subseconds = ticks % TICKS_PER_SECOND;
    return ts;
}

void fault_init(void)
{
    memset(&fault_state, 0, sizeof(fault_state));
}

void fault_raise(FaultCode code)
{
    if (code <= FAULT_NONE || code >= FAULT_COUNT) return;

    FAULT_ENTER_CRITICAL();
    fault_state.active_mask   |= (1u << code);
    fault_state.last_set[code] = get_current_timestamp_local();
    FAULT_EXIT_CRITICAL();

    log_write(LOG_LEVEL_ERROR, "Fault raised: %s", fault_to_string(code));
}

void fault_clear(FaultCode code)
{
    if (code <= FAULT_NONE || code >= FAULT_COUNT) return;

    FAULT_ENTER_CRITICAL();
    fault_state.active_mask &= ~(1u << code);
    FAULT_EXIT_CRITICAL();
}

bool fault_is_active(FaultCode code)
{
    if (code <= FAULT_NONE || code >= FAULT_COUNT) return false;
    return (fault_state.active_mask & (1u << code)) != 0;
}

void fault_clear_all(void)
{
    FAULT_ENTER_CRITICAL();
    fault_state.active_mask = 0;
    FAULT_EXIT_CRITICAL();
}

const char* fault_to_string(FaultCode code)
{
    if (code >= FAULT_COUNT) return "UNKNOWN";
    return fault_strings[code];
}