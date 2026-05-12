#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool object_present;
    int x1;
    int y1;
    int x2;
    int y2;
    char side[8];
    char kind[12];
    char gesture[16];
    float score;
    uint32_t frame_id;
} detection_t;

#ifdef __cplusplus
}
#endif
