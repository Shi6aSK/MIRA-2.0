#pragma once

#include <stdbool.h>
#include "vision_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void servo_init(void);
void servo_set_pan(int deg);
void servo_set_tilt(int deg);
void servo_center(void);
void servo_set_tracking(bool enable);
bool servo_is_tracking(void);
void servo_update_tracking(const detection_t *det, int frame_w, int frame_h);
/* Runtime invert flags – toggled from web UI without reflashing */
void servo_set_invert(bool pan_invert, bool tilt_invert);
void servo_get_invert(bool *pan_invert, bool *tilt_invert);

#ifdef __cplusplus
}
#endif
