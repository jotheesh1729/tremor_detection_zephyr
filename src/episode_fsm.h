#ifndef TREMOR_EPISODE_FSM_H_
#define TREMOR_EPISODE_FSM_H_

#include <stdbool.h>
#include <stdint.h>

#include "dsp/pipeline.h"

enum episode_type {
	EPISODE_TYPE_NONE = 0,
	EPISODE_TYPE_TREMOR,
	EPISODE_TYPE_DYSKINESIA,
};

struct episode_record {
	enum episode_type type;
	int64_t start_time_ms;
	int64_t duration_ms;
	float dominant_freq_hz; /* mean peak frequency across the episode's windows */
	float mean_severity;
};

void episode_fsm_init(void);

/*
 * Feeds one window's classification result into the hysteresis state
 * machine (idle -> candidate (K of N) -> episode -> cooldown -> idle).
 * Returns true and fills *out_record when this call closes out an episode
 * (the cooldown just completed); returns false otherwise.
 */
bool episode_fsm_update(const struct tremor_window_result *result,
			 struct episode_record *out_record);

#endif /* TREMOR_EPISODE_FSM_H_ */
