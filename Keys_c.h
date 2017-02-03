#ifndef KEYS_C_H
#define KEYS_C_H
#ifdef __cplusplus
#include "Keys.h"
extern "C" {
#else  /* if in C, we should define the types */
typedef void* Keys;
typedef void* BoardsTopology;
#endif

Keys* keys_new();

void keys_delete(Keys* that);

int keys_start(Keys* that, BoardsTopology* bt, volatile int* shouldStop);

void keys_stop(Keys* that);

float keys_getNoteValue(Keys* that, int note);

#ifdef __cplusplus
}
#endif
#endif /* KEYS_C_H */
