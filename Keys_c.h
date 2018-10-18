#ifndef KEYS_C_H
#define KEYS_C_H
#ifdef __cplusplus
#include "Keys.h"
extern "C" {
#else  /* if in C, we should define the types */
#include "BoardsTopology_c.h"
typedef void* Keys;
#endif

Keys* Keys_new(void);

void Keys_delete(Keys* that);

int Keys_start(Keys* that, BoardsTopology* bt, volatile int* shouldStop);

void Keys_stop(Keys* that);

void Keys_stopAndWait(Keys* that);

float Keys_getNoteValue(Keys* that, int note);

int Keys_loadInverseSquareCalibrationFile(Keys* that, const char* path, int offset);

int Keys_loadLinearCalibrationFile(Keys* that, const char* path);

int Keys_saveLinearCalibrationFile(Keys* that, const char* path);

void Keys_startTopCalibration(Keys* that);

int Keys_isTopCalibrationDone(Keys* that);

void Keys_stopTopCalibration(Keys* that);

void Keys_dumpTopCalibration(Keys* that);

void Keys_startBottomCalibration(Keys* that);

void Keys_stopBottomCalibration(Keys* that);

void Keys_dumpBottomCalibration(Keys* that);

void Keys_useCalibration(Keys* that, int shouldUse);

void Keys_setPostCallback(Keys* that, void(*postCallback)(void*, float*, unsigned int), void* arg);

#ifdef __cplusplus
}
#endif
#endif /* KEYS_C_H */
