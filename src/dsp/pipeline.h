#ifndef TREMOR_DSP_PIPELINE_H_
#define TREMOR_DSP_PIPELINE_H_

#include <stdbool.h>

enum tremor_classification {
	TREMOR_CLASS_NONE = 0,
	TREMOR_CLASS_TREMOR,
	TREMOR_CLASS_DYSKINESIA,
	TREMOR_CLASS_ACTIVITY, /* window rejected by the activity gate */
};

struct tremor_window_result {
	enum tremor_classification classification;
	float peak_freq_hz;
	float tremor_band_power;
	float dyskinesia_band_power;
	float broadband_power;
	float spectral_entropy;  /* normalized to [0, 1]; informational, not gating */
	float relative_power;    /* peak-band power / reference-band power; primary
				   * concentration metric -- see pipeline.c for the
				   * published methodology this is based on
				   */
	float gyro_rms;           /* rad/s; secondary activity-gate signal -- rejects
				    * deliberate rotation (e.g. a hand wave) that the
				    * accel-only concentration metric alone can't
				    * distinguish from tremor
				    */
	float severity;          /* == relative_power, clamped to [0, 1] */
};

/* Must be called once before the first tremor_pipeline_add_sample() call. */
void tremor_pipeline_init(void);

/*
 * Feeds one accelerometer sample (m/s^2) and one gyroscope sample (rad/s)
 * into the pipeline. Internally applies gravity removal (high-pass filter
 * per axis) to the accelerometer data, computes the orientation-invariant
 * accel magnitude, and accumulates both into the current analysis window.
 *
 * Returns true and fills *result when this sample completed a window
 * (i.e. a new classification is available). Returns false otherwise.
 */
bool tremor_pipeline_add_sample(float ax, float ay, float az, float gx, float gy, float gz,
				 struct tremor_window_result *result);

#endif /* TREMOR_DSP_PIPELINE_H_ */
