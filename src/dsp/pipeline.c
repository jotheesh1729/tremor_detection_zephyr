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

/* 50% overlap between successive windows: a window is re-analyzed every
 * HOP_SIZE new samples instead of every WINDOW_SIZE. Cheap on this chip
 * (a few tens of thousands of float copies per second) and meaningfully
 * improves temporal resolution -- see "Parkinson's Disease Tremor
 * Detection in the Wild Using Wearable Accelerometers" (Sensors 2020,
 * PMC7602495), which uses 3 s windows with 2 s overlap (roughly 33% hop).
 * 50% here is the more standard/simpler STFT convention; worth tuning
 * toward their heavier overlap later if smoother episode boundaries turn
 * out to matter more than the extra CPU cost.
 */
#define HOP_SIZE (WINDOW_SIZE / 2)

/*
 * Gravity removal: a one-pole high-pass filter per axis, cutoff well below
 * the tremor band, so gravity's (near-)DC contribution is filtered out on
 * each axis before the orientation-invariant magnitude is computed. This is
 * what lets v1 skip orientation estimation entirely (see project README,
 * "Magnitude vector, not orientation fusion").
 */
#define HPF_CUTOFF_HZ 0.5f

/*
 * Classification is based on spectral concentration, not absolute power:
 * "Continuous Accelerometry-Based Tremor Detection During Daily Living"
 * (Sensors 2026, doi:10.3390/s26051459) computes
 *
 *     P_rel = P(peak_freq +/- PEAK_BAND_HALFWIDTH_HZ) / P(REFERENCE_LOW_HZ..REFERENCE_HIGH_HZ)
 *
 * and classifies a window as tremor when P_rel >= 0.40 (84.8% sensitivity,
 * 96.5% specificity, 90.8% accuracy in their validation). The intuition:
 * a genuine tremor/dyskinesia window has power concentrated around one
 * peak; ordinary movement has power spread across a low-frequency
 * "voluntary movement" band and a high-frequency "physiological tremor"
 * band instead of concentrated at one frequency. This directly replaces
 * what was previously two separately-tuned, unvalidated heuristics (a
 * low-frequency power ratio gate and an absolute power floor) with one
 * metric that has a published threshold, and is amplitude-scale-invariant
 * (robust to grip strength / wearing tightness / individual variation,
 * unlike an absolute power threshold).
 */
#define REFERENCE_LOW_HZ          0.5f
#define REFERENCE_HIGH_HZ         15.0f
#define PEAK_BAND_HALFWIDTH_HZ    0.5f
#define REL_POWER_THRESHOLD       0.40f

/* Secondary hard safety cap, independent of the concentration metric:
 * gross motion (e.g. dropping/picking up the device) can be transiently
 * concentrated in-band by coincidence. Reasoned starting point, not
 * empirically validated -- watch Teleplot's broadband_power against real
 * deliberate gross motion to see if this needs adjusting.
 */
#define BROADBAND_RMS_CEILING 4.0f /* m/s^2 RMS */

/* ponytail: 2.0 was too tight -- a mere 5deg sweep at 5Hz already hits
 * ~1.9 rad/s RMS, so it was rejecting real tremor-scale motion, not just
 * waves. 5.0 gives tremor-scale motion headroom while a real wave
 * (~45deg sweep, ~7 rad/s RMS) still trips it. Still a rough estimate, not
 * measured -- recalibrate against real gyro_rms readings once there's
 * hardware data to tune against.
 */
#define GYRO_RMS_CEILING 5.0f /* rad/s */

static arm_rfft_fast_instance_f32 fft_instance;

static float hpf_alpha;
static float hpf_prev_raw[3];
static float hpf_prev_out[3];

/* Sliding windows: shifted left by one sample per call, new sample
 * appended at the end, so they always hold the most recent WINDOW_SIZE
 * samples in time order. Re-analyzed every HOP_SIZE samples (50% overlap).
 * gyro_window_buf only ever needs a time-domain RMS, not a spectrum, so it
 * skips the FFT/Hann-window machinery entirely.
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
 * Informational only (logged/Teleplotted) -- classification is driven by
 * the relative-power concentration metric below, not this.
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
