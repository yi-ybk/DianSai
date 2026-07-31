#include "track.h"
#include <string.h>

#define TRACK_DIRECTION_CHANGE_CONFIRM_SAMPLES 5U
#define TRACK_SEARCH_START_DELAY_S             0.15f
#define TRACK_SEARCH_FULL_DELAY_S              0.35f

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
    float run_error_sum = 0.0f;
    float run_error;
    float run_distance;
    float best_run_distance = 0.0f;
    float selected_error = 0.0f;
    float last_normalized_error;
    uint8_t run_black_count = 0U;
    uint8_t selected_black_count = 0U;
    bool run_selected = false;
    bool direction_change_confirmed = false;

    if ((track == NULL) || (!track->initialized) || (track->gray == NULL))
        return 0.0f;

    last_normalized_error = track->data.normalized_error;
    track->gray->update((Gray_t *)track->gray);
    black_mask = track->gray->get_black_mask((Gray_t *)track->gray);

    for (channel = 0U; channel <= track->init_config.channel_count; ++channel)
    {
        if ((channel < track->init_config.channel_count) &&
            (black_mask & (1UL << channel)))
        {
            run_error_sum += track->init_config.weights[channel];
            run_black_count++;
            continue;
        }

        if (run_black_count > 0U)
        {
            run_error = run_error_sum / (float)run_black_count;
            run_distance = run_error - last_normalized_error;
            if (run_distance < 0.0f)
                run_distance = -run_distance;

            if ((!run_selected) ||
                (run_distance < best_run_distance) ||
                ((run_distance == best_run_distance) &&
                 (run_black_count > selected_black_count)))
            {
                selected_error = run_error;
                selected_black_count = run_black_count;
                best_run_distance = run_distance;
                run_selected = true;
            }

            run_error_sum = 0.0f;
            run_black_count = 0U;
        }
    }

    if (run_selected &&
        ((last_normalized_error > 0.1f) ||
         (last_normalized_error < -0.1f)))
    {
        float error_jump;

        /*
         * Four or more adjacent black sensors indicate a stop line, crossing,
         * or a wide reflection rather than the normal guide line.  Keep the
         * entry direction while exposing the raw mask to higher-level finish
         * detection.
         */
        if (selected_black_count >= 4U)
            selected_error = last_normalized_error;

        /*
         * A fold or a reflection can briefly create a plausible run on the
         * opposite side of the array.  Do not let one or two samples reverse
         * the remembered recovery direction.  A real crossing of the centre
         * persists and is accepted after three consecutive samples.
         */
        if (((track->data.last_nonzero_error > 0.1f) &&
             (selected_error < -0.1f)) ||
            ((track->data.last_nonzero_error < -0.1f) &&
             (selected_error > 0.1f)))
        {
            if (track->data.direction_change_count <
                TRACK_DIRECTION_CHANGE_CONFIRM_SAMPLES)
            {
                track->data.direction_change_count++;
            }

            if (track->data.direction_change_count <
                TRACK_DIRECTION_CHANGE_CONFIRM_SAMPLES)
            {
                selected_error = last_normalized_error;
            }
            else
            {
                direction_change_confirmed = true;
                track->data.direction_change_count = 0U;
            }
        }
        else
        {
            track->data.direction_change_count = 0U;
        }

        error_jump = selected_error - last_normalized_error;
        if (error_jump < 0.0f)
            error_jump = -error_jump;
        if ((error_jump > 1.5f) && (!direction_change_confirmed))
        {
            /*
             * A real line cannot cross several sensors in one control period.
             * Treat such a sample as noise so lost-line recovery keeps turning
             * in the last valid direction.
             */
            run_selected = false;
            selected_black_count = 0U;
        }
    }
    else
    {
        track->data.direction_change_count = 0U;
    }

    track->data.black_mask = black_mask;
    track->data.black_count = selected_black_count;

    if (run_selected)
    {
        /*
         * A valid line produces one contiguous sensor run.  When reflections,
         * track edges, or crossings produce separated runs, follow the run
         * nearest to the previous line position instead of averaging unrelated
         * black regions and jumping across the sensor array.
         */
        track->data.normalized_error = selected_error;
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
        if (track->data.line_lost_time_s >= TRACK_SEARCH_FULL_DELAY_S)
            minimum_search_error = 3.5f;
        else if (track->data.line_lost_time_s >= TRACK_SEARCH_START_DELAY_S)
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
