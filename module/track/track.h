#pragma once

#include "gray.h"
#include "pid.h"
#include <stdbool.h>
#include <stdint.h>

#define TRACK_MAX_CHANNELS 16U

typedef struct Track Track_t;

typedef struct
{
    const Gray_t *gray;
    uint8_t channel_count;
    float weights[TRACK_MAX_CHANNELS];
    float base_speed;
    float max_turn_speed;
    PidInitConfig_t pid_config;
} TrackInitConfig_t;

typedef struct
{
    int32_t error;
    float normalized_error;
    float last_nonzero_error;
    float line_lost_time_s;
    float turn_speed;
    uint32_t black_mask;
    uint8_t black_count;
    uint8_t direction_change_count;
} TrackData_t;

struct Track
{
    const Gray_t *gray;
    bool initialized;
    TrackInitConfig_t init_config;
    TrackData_t data;
    Pid_t pid;

    bool (*init)(Track_t *track, const TrackInitConfig_t *config);
    float (*calculate_error)(Track_t *track);
    float (*update)(Track_t *track, float dt);
    void (*get_data)(Track_t *track, TrackData_t *data);
};

bool TrackInit(Track_t *track, const TrackInitConfig_t *config);
float TrackCalculateError(Track_t *track);
float TrackUpdate(Track_t *track, float dt);
void TrackGetData(Track_t *track, TrackData_t *data);

#define TRACK_OBJECT_DEFAULT              \
        .init               = TrackInit,   \
        .calculate_error    = TrackCalculateError, \
        .update             = TrackUpdate, \
        .get_data           = TrackGetData
