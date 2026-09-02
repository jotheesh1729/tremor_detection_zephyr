#include <math.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include "arm_math.h"
#include "pipeline.h"

LOG_MODULE_REGISTER(tremor_pipeline, LOG_LEVEL_INF);

#define PI_F 3.14159265358979323846f

#define WINDOW_SIZE   CONFIG_TREMOR_WINDOW_SIZE
#define SAMPLE_RATE   CONFIG_TREMOR_SAMPLE_RATE_HZ
#define NUM_BINS      (WINDOW_SIZE / 2 + 1) /* real FFT: bins 0..N/2 inclusive */

/* 50% overlap between successive windows (re-analyze every HOP_SIZE
 * samples, not every WINDOW_SIZE): negligible extra CPU cost, better
 * temporal resolution on episode boundaries.
 */
#define HOP_SIZE (WINDOW_SIZE / 2)

/* One-pole high-pass per axis, cutoff well below the tremor band, removes
 * gravity's near-DC bias before the orientation-invariant magnitude is
 * computed. This is what lets the pipeline skip orientation estimation.
 */
#define HPF_CUTOFF_HZ 0.5f

/* Classification is driven by spectral concentration, not absolute power:
 * P_rel = P(peak +/- PEAK_BAND_HALFWIDTH_HZ) / P(REFERENCE_LOW_HZ..REFERENCE_HIGH_HZ),
 * thresholded at REL_POWER_THRESHOLD. Genuine tremor/dyskinesia concentrates
 * power around one frequency; ordinary movement spreads it across a
 * low-frequency band and a higher "physiological tremor" band instead.
 * Scale-invariant, unlike an absolute power threshold (robust to grip
 * strength / wearing tightness / individual variation). Method and
 * threshold from Sensors 2026, doi:10.3390/s26051459.
 */
#define REFERENCE_LOW_HZ          0.5f
#define REFERENCE_HIGH_HZ         15.0f
#define PEAK_BAND_HALFWIDTH_HZ    0.5f
#define REL_POWER_THRESHOLD       0.40f

/* Hard safety cap, independent of the concentration metric: gross motion
 * (e.g. dropping the device) can be transiently concentrated in-band by
 * coincidence. Not empirically calibrated.
 */
#define BROADBAND_RMS_CEILING 4.0f /* m/s^2 RMS */

/* Second activity-gate signal: genuine tremor barely rotates the wrist,
 * while a deliberate hand motion at the same frequency involves much more
 * rotation. Not empirically calibrated.
 */
#define GYRO_RMS_CEILING 5.0f /* rad/s */

static arm_rfft_fast_instance_f32 fft_instance;

static float hpf_alpha;
static float hpf_prev_raw[3];
static float hpf_prev_out[3];

/* Sliding windows: shifted left by one sample per call, new sample
 * appended at the end, always holding the most recent WINDOW_SIZE samples
 * in time order. gyro_window_buf only needs a time-domain RMS, not a
 * spectrum, so it skips the FFT/Hann-window machinery entirely.
 */
static float window_buf[WINDOW_SIZE];
static float gyro_window_buf[WINDOW_SIZE];
static int samples_since_process;
static bool window_primed; /* false until the buffers have been filled once */

static float hann_window[WINDOW_SIZE];
static float fft_input[WINDOW_SIZE];
static float fft_output[WINDOW_SIZE];
static float bin_magnitude[NUM_BINS];

static inline float bin_to_hz(int bin)
{
	return (float)bin * (float)SAMPLE_RATE / (float)WINDOW_SIZE;
}

static inline int hz_to_bin_ceil(float hz)
{
	return (int)ceilf(hz * (float)WINDOW_SIZE / (float)SAMPLE_RATE);
}

static inline int hz_to_bin_floor(float hz)
{
	return (int)floorf(hz * (float)WINDOW_SIZE / (float)SAMPLE_RATE);
}

void tremor_pipeline_init(void)
{
	arm_status status = arm_rfft_fast_init_f32(&fft_instance, WINDOW_SIZE);

	__ASSERT(status == ARM_MATH_SUCCESS, "arm_rfft_fast_init_f32 failed: %d", status);
	ARG_UNUSED(status);

	float dt = 1.0f / (float)SAMPLE_RATE;
	float tau = 1.0f / (2.0f * PI_F * HPF_CUTOFF_HZ);

	hpf_alpha = tau / (tau + dt);

	memset(hpf_prev_raw, 0, sizeof(hpf_prev_raw));
	memset(hpf_prev_out, 0, sizeof(hpf_prev_out));
	memset(window_buf, 0, sizeof(window_buf));
	memset(gyro_window_buf, 0, sizeof(gyro_window_buf));
	samples_since_process = 0;
	window_primed = false;

	/* Hann window, precomputed once: w[n] = 0.5 * (1 - cos(2*pi*n/(N-1))) */
	for (int n = 0; n < WINDOW_SIZE; n++) {
		hann_window[n] = 0.5f * (1.0f - cosf(2.0f * PI_F * n / (WINDOW_SIZE - 1)));
	}
}

static float high_pass(int axis, float raw)
{
	float out = hpf_alpha * (hpf_prev_out[axis] + raw - hpf_prev_raw[axis]);

	hpf_prev_raw[axis] = raw;
	hpf_prev_out[axis] = out;

	return out;
}

/* Parabolic interpolation around the peak power bin for a sub-bin frequency
 * estimate. Standard three-point interpolation on the bin power values.
 */
static float interpolate_peak_freq(int peak_bin)
{
	if (peak_bin <= 0 || peak_bin >= NUM_BINS - 1) {
		return bin_to_hz(peak_bin);
	}

	float y_left = bin_magnitude[peak_bin - 1] * bin_magnitude[peak_bin - 1];
	float y_center = bin_magnitude[peak_bin] * bin_magnitude[peak_bin];
	float y_right = bin_magnitude[peak_bin + 1] * bin_magnitude[peak_bin + 1];
	float denom = y_left - 2.0f * y_center + y_right;

	if (fabsf(denom) < 1e-9f) {
		return bin_to_hz(peak_bin);
	}

	float delta = 0.5f * (y_left - y_right) / denom;

	return bin_to_hz(peak_bin) + delta * (float)SAMPLE_RATE / (float)WINDOW_SIZE;
}

static float band_power_bins(int bin_low, int bin_high)
{
	float power = 0.0f;

	if (bin_low < 0) {
		bin_low = 0;
	}

	for (int k = bin_low; k <= bin_high && k < NUM_BINS; k++) {
		power += bin_magnitude[k] * bin_magnitude[k];
	}

	return power;
}

static inline float band_power_hz(float hz_low, float hz_high)
{
	return band_power_bins(hz_to_bin_ceil(hz_low), hz_to_bin_floor(hz_high));
}

/* Shannon entropy of the normalized power spectrum over [bin_low, bin_high],
 * normalized to [0, 1] by dividing by log(number of bins considered).
 * Informational only -- classification uses relative-power concentration,
 * not this.
 */
static float spectral_entropy(int bin_low, int bin_high)
{
	float total = band_power_bins(bin_low, bin_high);

	if (total < 1e-12f) {
		return 0.0f;
	}

	float entropy = 0.0f;
	int num_bins = 0;

	for (int k = bin_low; k <= bin_high && k < NUM_BINS; k++) {
		float p = (bin_magnitude[k] * bin_magnitude[k]) / total;

		if (p > 1e-12f) {
			entropy -= p * logf(p);
		}
		num_bins++;
	}

	if (num_bins <= 1) {
		return 0.0f;
	}

	return entropy / logf((float)num_bins);
}

static void process_window(struct tremor_window_result *result)
{
	for (int n = 0; n < WINDOW_SIZE; n++) {
		fft_input[n] = window_buf[n] * hann_window[n];
	}

	arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0);

	/* Packed real-FFT output: [0]=DC real, [1]=Nyquist real,
	 * [2k]/[2k+1] = real/imag for bin k, k = 1..N/2-1.
	 */
	bin_magnitude[0] = fabsf(fft_output[0]);
	bin_magnitude[NUM_BINS - 1] = fabsf(fft_output[1]);
	for (int k = 1; k < NUM_BINS - 1; k++) {
		float re = fft_output[2 * k];
		float im = fft_output[2 * k + 1];

		bin_magnitude[k] = sqrtf(re * re + im * im);
	}

	float broadband_power = band_power_bins(1, NUM_BINS - 1); /* skip DC */
	float broadband_rms = sqrtf(broadband_power / (float)(NUM_BINS - 1));

	result->broadband_power = broadband_power;

	float gyro_sum_sq = 0.0f;

	for (int n = 0; n < WINDOW_SIZE; n++) {
		gyro_sum_sq += gyro_window_buf[n] * gyro_window_buf[n];
	}
	result->gyro_rms = sqrtf(gyro_sum_sq / (float)WINDOW_SIZE);

	int tremor_lo = hz_to_bin_ceil(CONFIG_TREMOR_BAND_LOW_HZ);
	int tremor_hi = hz_to_bin_floor(CONFIG_TREMOR_BAND_HIGH_HZ);
	int dysk_lo = hz_to_bin_ceil(CONFIG_DYSKINESIA_BAND_LOW_HZ);
	int dysk_hi = hz_to_bin_floor(CONFIG_DYSKINESIA_BAND_HIGH_HZ);

	result->tremor_band_power = band_power_bins(tremor_lo, tremor_hi);
	result->dyskinesia_band_power = band_power_bins(dysk_lo, dysk_hi);

	/* Peak search and entropy scan the combined tremor+dyskinesia range:
	 * that is the region a real peak in either condition would fall in.
	 */
	int scan_lo = tremor_lo < dysk_lo ? tremor_lo : dysk_lo;
	int scan_hi = tremor_hi > dysk_hi ? tremor_hi : dysk_hi;

	int peak_bin = scan_lo;
	float peak_mag = bin_magnitude[scan_lo];

	for (int k = scan_lo + 1; k <= scan_hi && k < NUM_BINS; k++) {
		if (bin_magnitude[k] > peak_mag) {
			peak_mag = bin_magnitude[k];
			peak_bin = k;
		}
	}

	result->peak_freq_hz = interpolate_peak_freq(peak_bin);
	result->spectral_entropy = spectral_entropy(scan_lo, scan_hi);

	float peak_band_power = band_power_hz(result->peak_freq_hz - PEAK_BAND_HALFWIDTH_HZ,
					       result->peak_freq_hz + PEAK_BAND_HALFWIDTH_HZ);
	float reference_power = band_power_hz(REFERENCE_LOW_HZ, REFERENCE_HIGH_HZ);

	result->relative_power = reference_power > 1e-9f ? peak_band_power / reference_power : 0.0f;

	bool concentrated = result->relative_power >= REL_POWER_THRESHOLD;
	bool gross_motion = broadband_rms > BROADBAND_RMS_CEILING;
	bool excess_rotation = result->gyro_rms > GYRO_RMS_CEILING;

	if (!concentrated || gross_motion || excess_rotation) {
		result->classification = TREMOR_CLASS_ACTIVITY;
	} else if (result->peak_freq_hz >= CONFIG_TREMOR_BAND_LOW_HZ &&
		   result->peak_freq_hz <= CONFIG_TREMOR_BAND_HIGH_HZ) {
		result->classification = TREMOR_CLASS_TREMOR;
	} else if (result->peak_freq_hz >= CONFIG_DYSKINESIA_BAND_LOW_HZ &&
		   result->peak_freq_hz <= CONFIG_DYSKINESIA_BAND_HIGH_HZ) {
		result->classification = TREMOR_CLASS_DYSKINESIA;
	} else {
		/* Concentrated peak, but outside both configured bands (e.g.
		 * physiological tremor above the dyskinesia band).
		 */
		result->classification = TREMOR_CLASS_NONE;
	}

	result->severity = result->relative_power > 1.0f ? 1.0f : result->relative_power;
}

bool tremor_pipeline_add_sample(float ax, float ay, float az, float gx, float gy, float gz,
				 struct tremor_window_result *result)
{
	float fx = high_pass(0, ax);
	float fy = high_pass(1, ay);
	float fz = high_pass(2, az);
	float magnitude = sqrtf(fx * fx + fy * fy + fz * fz);
	float gyro_magnitude = sqrtf(gx * gx + gy * gy + gz * gz);

	memmove(&window_buf[0], &window_buf[1], (WINDOW_SIZE - 1) * sizeof(window_buf[0]));
	window_buf[WINDOW_SIZE - 1] = magnitude;
	memmove(&gyro_window_buf[0], &gyro_window_buf[1],
		(WINDOW_SIZE - 1) * sizeof(gyro_window_buf[0]));
	gyro_window_buf[WINDOW_SIZE - 1] = gyro_magnitude;
	samples_since_process++;

	if (!window_primed) {
		if (samples_since_process < WINDOW_SIZE) {
			return false;
		}
		window_primed = true;
		samples_since_process = 0;
		process_window(result);
		return true;
	}

	if (samples_since_process < HOP_SIZE) {
		return false;
	}

	samples_since_process = 0;
	process_window(result);

	return true;
}
