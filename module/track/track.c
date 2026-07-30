#include "track.h"
#include <string.h>

static void TrackBindMethods(Track_t *track);
static bool TrackConfigIsValid(const TrackInitConfig_t *config);

bool TrackInit(Track_t *track, const TrackInitConfig_t *config)
{
    if ((track == NULL) || (config == NULL))
        return false;

    TrackBindMethods(track);
    if (track->initialized)
        return true;

    if (!TrackConfigIsValid(config))
        return false;
    track->gray = config->gray;
    track->init_config = *config;
    memset(&track->data, 0, sizeof(track->data));
    if (!track->pid.init(&track->pid, &config->pid_config))
        return false;
    track->initialized = true;
    return true;
}

float TrackCalculateError(Track_t *track)
{
    uint32_t black_mask;
    uint8_t channel;
    float error = 0.0f;
    float last_normalized_error;
    uint8_t black_count = 0;

    if ((track == NULL) || (!track->initialized) || (track->gray == NULL))
        return 0.0f;

    last_normalized_error = track->data.normalized_error;
    track->gray->update((Gray_t *)track->gray);
    black_mask = track->gray->get_black_mask((Gray_t *)track->gray);

    for (channel = 0U; channel < track->init_config.channel_count; ++channel)
    {
        if (black_mask & (1UL << channel))
        {
            error += track->init_config.weights[channel];
            black_count++;
        }
    }

    track->data.black_mask = black_mask;
    track->data.black_count = black_count;

    if (black_count > 0)
    {
        track->data.normalized_error = error / (float)black_count;
        if ((track->data.normalized_error > 0.1f) ||
            (track->data.normalized_error < -0.1f))
        {
            track->data.last_nonzero_error =
                track->data.normalized_error;
        }
    }
    else
    {
        track->data.normalized_error = last_normalized_error;
        if ((track->data.normalized_error <= 0.1f) &&
            (track->data.normalized_error >= -0.1f))
        {
            track->data.normalized_error =
                track->data.last_nonzero_error;
        }
    }

    /*
     * Use the weighted average so the PID gain does not change when the line
     * covers one sensor versus two sensors.  If the line is temporarily lost,
     * retain the last direction instead of commanding an abrupt straight run.
     */
    track->data.error =
        (int32_t)(track->data.normalized_error * 100.0f);
    return track->data.normalized_error;
}

float TrackUpdate(Track_t *track, float dt)
{
    float error;
    float control_error;
    float minimum_search_error;
    float pid_output;

    if ((track == NULL) || (!track->initialized) || !(dt > 0.0f))
        return 0.0f;

    error = TrackCalculateError(track);
    control_error = error;

    if (track->data.black_count == 0U)
    {
        track->data.line_lost_time_s += dt;
        minimum_search_error = 0.0f;
        if (track->data.line_lost_time_s >= 0.4f)
            minimum_search_error = 3.5f;
        else if (track->data.line_lost_time_s >= 0.15f)
            minimum_search_error = 2.5f;

        if ((control_error > 0.0f) &&
            (control_error < minimum_search_error))
        {
            control_error = minimum_search_error;
        }
        else if ((control_error < 0.0f) &&
                 (control_error > -minimum_search_error))
        {
            control_error = -minimum_search_error;
        }
    }
    else
    {
        track->data.line_lost_time_s = 0.0f;
    }

    pid_output = track->pid.calculate(&track->pid, 0, control_error, dt);

    track->data.turn_speed = pid_output;

    if (track->data.turn_speed > track->init_config.max_turn_speed)
        track->data.turn_speed = track->init_config.max_turn_speed;
    if (track->data.turn_speed < -track->init_config.max_turn_speed)
        track->data.turn_speed = -track->init_config.max_turn_speed;

    return track->data.turn_speed;
}

void TrackGetData(Track_t *track, TrackData_t *data)
{
    if ((track == NULL) || (data == NULL))
        return;

    if (track->initialized)
        TrackUpdate(track, 0.02f);

    *data = track->data;
}

static void TrackBindMethods(Track_t *track)
{
    if (track == NULL)
        return;

    track->init = TrackInit;
    track->calculate_error = TrackCalculateError;
    track->update = TrackUpdate;
    track->get_data = TrackGetData;

    track->pid.init               = PidInit;
    track->pid.calculate          = PidCalculate;
    track->pid.reset              = PidReset;
    track->pid.set_target         = PidSetTarget;
    track->pid.set_param          = PidSetParam;
    track->pid.set_output_limit   = PidSetOutputLimit;
    track->pid.set_integral_limit = PidSetIntegralLimit;
    track->pid.get_data           = PidGetData;
}

static bool TrackConfigIsValid(const TrackInitConfig_t *config)
{
    if ((config == NULL) || (config->gray == NULL))
        return false;

    if ((config->channel_count == 0U) || (config->channel_count > TRACK_MAX_CHANNELS))
        return false;

    return true;
}
