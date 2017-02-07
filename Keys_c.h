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

float Keys_getNoteValue(Keys* that, int note);

#ifdef __cplusplus
}
#endif
#endif /* KEYS_C_H */
