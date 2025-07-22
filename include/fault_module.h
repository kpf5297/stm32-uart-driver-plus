#ifndef FAULT_MODULE_H
#define FAULT_MODULE_H

#include <stdint.h>
#include <stdbool.h>
#include "logging.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FAULT_NONE = 0,
    FAULT_OVERCURRENT,
    FAULT_OVERVOLTAGE,
    FAULT_UNDERVOLTAGE,
    FAULT_STARTUP_FAILED,
    FAULT_COMMUNICATION,
    FAULT_SENSOR,
    FAULT_ENCODER_ERROR,
    FAULT_CONTROL_LOOP_ERROR,
    FAULT_COUNT
} FaultCode;

typedef struct {
    uint32_t active_mask;
    Timestamp last_set[FAULT_COUNT];
} FaultStatus;

extern FaultStatus fault_state;

void fault_init(void);
void fault_raise(FaultCode code);
void fault_clear(FaultCode code);
bool fault_is_active(FaultCode code);
void fault_clear_all(void);
const char* fault_to_string(FaultCode code);

#ifdef __cplusplus
}
#endif

#endif // FAULT_MODULE_H
