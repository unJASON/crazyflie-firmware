#ifndef POS_LOCALIZATION_H_
#define POS_LOCALIZATION_H_
#include "system.h"

typedef enum {
  STATE_rlX, STATE_rlY, STATE_rlYaw, STATE_DIM_rl
} relative_stateIdx_t;

typedef struct
{
    float x;
    float y;
    float z;
    float yaw;
    float distance;
    uint32_t old_tick;
    float P[STATE_DIM_rl][STATE_DIM_rl];
} position_state;

uint8_t initLocalization();
void localizationTask();
void EKF(float vx,float vy,float yawRate,float dtEKF);//EKF model
#endif