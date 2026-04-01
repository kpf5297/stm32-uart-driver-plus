#include "unity.h"

#include <stdio.h>

int UnityTestsRun = 0;
int UnityTestsFailed = 0;

void UnityBegin(const char *suite_name) {
    printf("[UNITY] Suite: %s\n", suite_name ? suite_name : "(unnamed)");
}

int UnityEnd(void) {
    printf("[UNITY] Tests run: %d\n", UnityTestsRun);
    printf("[UNITY] Tests failed: %d\n", UnityTestsFailed);
    return UnityTestsFailed;
}

void UnityFail(const char *message, const char *file, int line) {
    UnityTestsFailed++;
    printf("[UNITY][FAIL] %s at %s:%d\n", message ? message : "(null)", file, line);
}

void UnityConcludeTest(void) {
    /* Minimal runner does not track per-test pass/fail state beyond global counters. */
}

void UnityPrint(const char *message) {
    printf("%s", message ? message : "");
}
