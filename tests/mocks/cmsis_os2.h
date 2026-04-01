#ifndef CMSIS_OS2_H
#define CMSIS_OS2_H

#include <stdint.h>

typedef void *osMutexId_t;
typedef void *osThreadId_t;
typedef uint32_t osStatus_t;
typedef int32_t osPriority_t;

typedef struct {
    osPriority_t priority;
    uint32_t stack_size;
} osThreadAttr_t;

enum {
    osOK = 0U,
    osError = 1U,
    osErrorTimeout = 2U,
};

#define osWaitForever 0xFFFFFFFFU
#define osFlagsWaitAny 0x00000000U
#define osPriorityNormal ((osPriority_t)0)

#endif
