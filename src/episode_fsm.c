#include <string.h>

#include <zephyr/kernel.h>

#include "episode_fsm.h"

#define HISTORY_SIZE   CONFIG_TREMOR_EPISODE_WINDOW_COUNT
#define ENTER_COUNT    CONFIG_TREMOR_EPISODE_ENTER_COUNT
#define COOLDOWN_COUNT CONFIG_TREMOR_EPISODE_COOLDOWN_WINDOWS

enum fsm_state {
	FSM_IDLE,
	FSM_EPISODE,
};

static enum fsm_state state;

/* K-of-N entry history: ring buffer of whether each of the last N windows
 * was "positive" (tremor or dyskinesia), used only while idle.
 */
static bool history[HISTORY_SIZE];
static int history_head;
static int history_positive_count;

/* Running accumulators for the episode currently in progress. */
static int64_t episode_start_ms;
static float episode_freq_sum;
static float episode_severity_sum;
static int episode_window_count;
static int episode_tremor_windows;
static int episode_dyskinesia_windows;
static int consecutive_negative;

static bool is_positive(enum tremor_classification c)
{
	return c == TREMOR_CLASS_TREMOR || c == TREMOR_CLASS_DYSKINESIA;
}

void episode_fsm_init(void)
{
	state = FSM_IDLE;
	memset(history, 0, sizeof(history));
	history_head = 0;
	history_positive_count = 0;
	episode_window_count = 0;
	consecutive_negative = 0;
}

static void history_push(bool positive)
{
	if (history[history_head]) {
		history_positive_count--;
	}
	history[history_head] = positive;
	if (positive) {
		history_positive_count++;
	}
	history_head = (history_head + 1) % HISTORY_SIZE;
}

static void episode_start(void)
{
	state = FSM_EPISODE;
	episode_start_ms = k_uptime_get();
	episode_freq_sum = 0.0f;
	episode_severity_sum = 0.0f;
	episode_window_count = 0;
	episode_tremor_windows = 0;
	episode_dyskinesia_windows = 0;
	consecutive_negative = 0;
}

static void episode_accumulate(const struct tremor_window_result *result)
{
	episode_freq_sum += result->peak_freq_hz;
	episode_severity_sum += result->severity;
	episode_window_count++;

	if (result->classification == TREMOR_CLASS_TREMOR) {
		episode_tremor_windows++;
	} else if (result->classification == TREMOR_CLASS_DYSKINESIA) {
		episode_dyskinesia_windows++;
	}
}

static void episode_close(struct episode_record *out_record)
{
	out_record->type = episode_tremor_windows >= episode_dyskinesia_windows
				    ? EPISODE_TYPE_TREMOR
				    : EPISODE_TYPE_DYSKINESIA;
	out_record->start_time_ms = episode_start_ms;
	out_record->duration_ms = k_uptime_get() - episode_start_ms;
	out_record->dominant_freq_hz =
		episode_window_count > 0 ? episode_freq_sum / episode_window_count : 0.0f;
	out_record->mean_severity =
		episode_window_count > 0 ? episode_severity_sum / episode_window_count : 0.0f;

	state = FSM_IDLE;
	memset(history, 0, sizeof(history));
	history_head = 0;
	history_positive_count = 0;
}

bool episode_fsm_update(const struct tremor_window_result *result,
			 struct episode_record *out_record)
{
	bool positive = is_positive(result->classification);

	if (state == FSM_IDLE) {
		history_push(positive);
		if (history_positive_count >= ENTER_COUNT) {
			episode_start();
			episode_accumulate(result);
		}
		return false;
	}

	/* FSM_EPISODE */
	episode_accumulate(result);

	if (positive) {
		consecutive_negative = 0;
	} else {
		consecutive_negative++;
		if (consecutive_negative >= COOLDOWN_COUNT) {
			episode_close(out_record);
			return true;
		}
	}

	return false;
}
