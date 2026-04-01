#ifndef UNITY_H
#define UNITY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void UnityBegin(const char *suite_name);
int UnityEnd(void);
void UnityFail(const char *message, const char *file, int line);
void UnityConcludeTest(void);
void UnityPrint(const char *message);

extern int UnityTestsRun;
extern int UnityTestsFailed;

#ifndef UNITY_CURRENT_TEST_NAME
#define UNITY_CURRENT_TEST_NAME ""
#endif

void setUp(void);
void tearDown(void);

#define TEST_ASSERT_TRUE(condition) \
    do { \
        if (!(condition)) { \
            UnityFail("TEST_ASSERT_TRUE failed: " #condition, __FILE__, __LINE__); \
            return; \
        } \
    } while (0)

#define TEST_ASSERT_EQUAL_UINT(expected, actual) \
    do { \
        uint32_t _exp = (uint32_t)(expected); \
        uint32_t _act = (uint32_t)(actual); \
        if (_exp != _act) { \
            UnityFail("TEST_ASSERT_EQUAL_UINT failed", __FILE__, __LINE__); \
            return; \
        } \
    } while (0)

#define TEST_ASSERT_EQUAL_INT(expected, actual) \
    do { \
        int _exp = (int)(expected); \
        int _act = (int)(actual); \
        if (_exp != _act) { \
            UnityFail("TEST_ASSERT_EQUAL_INT failed", __FILE__, __LINE__); \
            return; \
        } \
    } while (0)

#define RUN_TEST(func) \
    do { \
        UnityTestsRun++; \
        setUp(); \
        func(); \
        tearDown(); \
        UnityConcludeTest(); \
    } while (0)

#ifdef __cplusplus
}
#endif

#endif
