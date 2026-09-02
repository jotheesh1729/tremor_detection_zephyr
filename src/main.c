#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <stdint.h>
#include <string.h>

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

/* Local visual feedback, independent of BLE: led0 lit for the duration of
 * any window classified as tremor, led1 for dyskinesia. Both aliases are
 * on-board LEDs already defined by the board's own devicetree.
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

/* Stats accumulated between BLE reports: periodic radio traffic instead
 * of one notify per window (~1.2s cadence) keeps power draw sane for a
 * battery-powered wearable.
 */
static struct tremor_report pending_report;
static uint32_t report_severity_sum_pct;
static int64_t next_report_at_ms;

static void report_episode(const struct episode_record *record)
{
	uint16_t duration_s = (uint16_t)(record->duration_ms / 1000);
	uint8_t severity_pct = (uint8_t)(record->mean_severity * 100.0f);

	if (record->type == EPISODE_TYPE_TREMOR) {
		pending_report.tremor_episodes++;
		pending_report.tremor_duration_s += duration_s;
	} else {
		pending_report.dyskinesia_episodes++;
		pending_report.dyskinesia_duration_s += duration_s;
	}
	report_severity_sum_pct += severity_pct;
}

static void send_report_if_due(void)
{
	int64_t now = k_uptime_get();
	uint8_t total_episodes;

	if (now < next_report_at_ms) {
		return;
	}
	next_report_at_ms = now + (int64_t)CONFIG_TREMOR_REPORT_INTERVAL_MIN * 60 * 1000;

	total_episodes = pending_report.tremor_episodes + pending_report.dyskinesia_episodes;
	pending_report.mean_severity_pct =
		total_episodes > 0 ? (uint8_t)(report_severity_sum_pct / total_episodes) : 0;

	ble_service_notify_report(&pending_report);

	memset(&pending_report, 0, sizeof(pending_report));
	report_severity_sum_pct = 0;
}

static void dsp_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	tremor_pipeline_init();
	episode_fsm_init();
	next_report_at_ms = k_uptime_get() + (int64_t)CONFIG_TREMOR_REPORT_INTERVAL_MIN * 60 * 1000;

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

		/* ">name:value" lines are picked up by a live serial plotter
		 * during development (Teleplot); classification is 0=none
		 * 1=tremor 2=dyskinesia 3=activity.
		 */
		printk(">peak_freq_hz:%.3f\n", (double)result.peak_freq_hz);
		printk(">relative_power:%.3f\n", (double)result.relative_power);
		printk(">tremor_power:%.4f\n", (double)result.tremor_band_power);
		printk(">dyskinesia_power:%.4f\n", (double)result.dyskinesia_band_power);
		printk(">entropy:%.3f\n", (double)result.spectral_entropy);
		printk(">gyro_rms:%.3f\n", (double)result.gyro_rms);
		printk(">severity:%.3f\n", (double)result.severity);
		printk(">classification:%d\n", (int)result.classification);

		gpio_pin_set_dt(&led_tremor, result.classification == TREMOR_CLASS_TREMOR);
		gpio_pin_set_dt(&led_dyskinesia, result.classification == TREMOR_CLASS_DYSKINESIA);

		if (episode_fsm_update(&result, &record)) {
			LOG_INF("episode closed: type=%s duration=%lldms freq=%.2fHz "
				"mean_severity=%.2f",
				record.type == EPISODE_TYPE_TREMOR ? "tremor" : "dyskinesia",
				record.duration_ms, (double)record.dominant_freq_hz,
				(double)record.mean_severity);
			ble_service_notify_episode(&record);
			report_episode(&record);
		}

		send_report_if_due();
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
