#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <stdint.h>

#include "ble_service.h"
#include "dsp/pipeline.h"
#include "episode_fsm.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* accel0 alias points at the board's IMU node (boards/st/disco_l475_iot1.dts).
 * Getting the device through this alias, not a node label, is what makes
 * porting to a board with a different IMU a devicetree change instead of
 * an application code change.
 */
static const struct device *const accel = DEVICE_DT_GET(DT_ALIAS(accel0));

/* Immediate visual feedback while there is no BLE yet: led0 lit for the
 * duration of any window classified as tremor, led1 for dyskinesia. Both
 * aliases are on-board LEDs already defined by the board's own devicetree,
 * so no overlay is needed.
 */
static const struct gpio_dt_spec led_tremor = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led_dyskinesia = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

struct accel_sample {
	float ax;
	float ay;
	float az;
	float gx;
	float gy;
	float gz;
};

/* Ring buffer between the driver's FIFO-watermark trigger callback and the
 * DSP thread. Sized well above one FIFO drain batch (16 samples) to absorb
 * scheduling jitter. Overrun policy: drop the newest sample and log it --
 * this preserves the in-progress window's temporal continuity rather than
 * corrupting it by silently overwriting older, not-yet-consumed samples.
 */
K_MSGQ_DEFINE(sample_queue, sizeof(struct accel_sample), 32, 4);

static void accel_trigger_handler(const struct device *dev, const struct sensor_trigger *trig)
{
	struct sensor_value accel_xyz[3];
	struct sensor_value gyro_xyz[3];
	struct accel_sample sample;

	if (sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel_xyz) < 0) {
		return;
	}
	if (sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro_xyz) < 0) {
		return;
	}

	sample.ax = sensor_value_to_float(&accel_xyz[0]);
	sample.ay = sensor_value_to_float(&accel_xyz[1]);
	sample.az = sensor_value_to_float(&accel_xyz[2]);
	sample.gx = sensor_value_to_float(&gyro_xyz[0]);
	sample.gy = sensor_value_to_float(&gyro_xyz[1]);
	sample.gz = sensor_value_to_float(&gyro_xyz[2]);

	if (k_msgq_put(&sample_queue, &sample, K_NO_WAIT) != 0) {
		LOG_WRN("sample queue full, dropping sample");
	}
}

static const char *classification_str(enum tremor_classification c)
{
	switch (c) {
	case TREMOR_CLASS_TREMOR:
		return "tremor";
	case TREMOR_CLASS_DYSKINESIA:
		return "dyskinesia";
	case TREMOR_CLASS_ACTIVITY:
		return "activity";
	default:
		return "none";
	}
}

static void dsp_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	tremor_pipeline_init();
	episode_fsm_init();

	while (1) {
		struct accel_sample sample;
		struct tremor_window_result result;
		struct episode_record record;

		k_msgq_get(&sample_queue, &sample, K_FOREVER);

		if (!tremor_pipeline_add_sample(sample.ax, sample.ay, sample.az, sample.gx,
						 sample.gy, sample.gz, &result)) {
			continue;
		}

		LOG_INF("window: %s peak=%.2fHz rel_pwr=%.2f tremor_pwr=%.3f dysk_pwr=%.3f "
			"entropy=%.2f gyro_rms=%.2f severity=%.2f",
			classification_str(result.classification), (double)result.peak_freq_hz,
			(double)result.relative_power, (double)result.tremor_band_power,
			(double)result.dyskinesia_band_power, (double)result.spectral_entropy,
			(double)result.gyro_rms, (double)result.severity);

		printk(">peak_freq_hz:%.3f\n", (double)result.peak_freq_hz);
		printk(">relative_power:%.3f\n", (double)result.relative_power);
		printk(">tremor_power:%.4f\n", (double)result.tremor_band_power);
		printk(">dyskinesia_power:%.4f\n", (double)result.dyskinesia_band_power);
		printk(">entropy:%.3f\n", (double)result.spectral_entropy);
		printk(">gyro_rms:%.3f\n", (double)result.gyro_rms);
		printk(">severity:%.3f\n", (double)result.severity);
		/* 0=none 1=tremor 2=dyskinesia 3=activity -- plotted as a step
		 * trace so the actual decision is visible next to the numbers
		 * that fed it, not just in the text log.
		 */
		printk(">classification:%d\n", (int)result.classification);

		gpio_pin_set_dt(&led_tremor, result.classification == TREMOR_CLASS_TREMOR);
		gpio_pin_set_dt(&led_dyskinesia, result.classification == TREMOR_CLASS_DYSKINESIA);
		ble_service_notify_severity(&result);

		if (episode_fsm_update(&result, &record)) {
			LOG_INF("episode closed: type=%s duration=%lldms freq=%.2fHz "
				"mean_severity=%.2f",
				record.type == EPISODE_TYPE_TREMOR ? "tremor" : "dyskinesia",
				record.duration_ms, (double)record.dominant_freq_hz,
				(double)record.mean_severity);
			ble_service_notify_episode(&record);
		}
	}
}

K_THREAD_DEFINE(dsp_tid, 2048, dsp_thread, NULL, NULL, NULL, 5, 0, 0);

int main(void)
{
	LOG_INF("booted");

	if (!device_is_ready(accel)) {
		LOG_ERR("accel device not ready");
		return 0;
	}

	if (!device_is_ready(led_tremor.port) || !device_is_ready(led_dyskinesia.port)) {
		LOG_ERR("LED GPIO not ready");
		return 0;
	}
	gpio_pin_configure_dt(&led_tremor, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led_dyskinesia, GPIO_OUTPUT_INACTIVE);

	static const struct sensor_trigger trig = {
		.type = SENSOR_TRIG_FIFO_WATERMARK,
		.chan = SENSOR_CHAN_ACCEL_XYZ,
	};

	if (sensor_trigger_set(accel, &trig, accel_trigger_handler) < 0) {
		LOG_ERR("failed to set FIFO watermark trigger");
		return 0;
	}

	ble_service_start();

	return 0;
}
