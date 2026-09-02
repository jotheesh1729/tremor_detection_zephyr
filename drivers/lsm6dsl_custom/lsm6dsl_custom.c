#define DT_DRV_COMPAT st_lsm6dsl

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "lsm6dsl_custom.h"

LOG_MODULE_REGISTER(lsm6dsl_custom, CONFIG_SENSOR_LOG_LEVEL);

static int lsm6dsl_custom_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct lsm6dsl_custom_config *cfg = dev->config;
	struct lsm6dsl_custom_data *data = dev->data;
	uint8_t buf[12];
	int ret;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_ACCEL_XYZ &&
	    chan != SENSOR_CHAN_GYRO_XYZ) {
		return -ENOTSUP;
	}

	/* OUTX_L_G through OUTZ_H_XL are contiguous (gyro block, then accel
	 * block) -- one burst read gets both regardless of which was asked
	 * for, so every fetch just reads all of it.
	 */
	ret = i2c_burst_read_dt(&cfg->i2c, LSM6DSL_REG_OUTX_L_G, buf, sizeof(buf));
	if (ret < 0) {
		LOG_ERR("failed to read sensor data: %d", ret);
		return ret;
	}

	data->gx = sys_get_le16(&buf[0]);
	data->gy = sys_get_le16(&buf[2]);
	data->gz = sys_get_le16(&buf[4]);
	data->x = sys_get_le16(&buf[6]);
	data->y = sys_get_le16(&buf[8]);
	data->z = sys_get_le16(&buf[10]);

	return 0;
}

static int lsm6dsl_custom_channel_get(const struct device *dev, enum sensor_channel chan,
				       struct sensor_value *val)
{
	struct lsm6dsl_custom_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		sensor_ug_to_ms2(data->x * LSM6DSL_ACCEL_FS4G_SENSITIVITY_UG, val);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		sensor_ug_to_ms2(data->y * LSM6DSL_ACCEL_FS4G_SENSITIVITY_UG, val);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		sensor_ug_to_ms2(data->z * LSM6DSL_ACCEL_FS4G_SENSITIVITY_UG, val);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		sensor_ug_to_ms2(data->x * LSM6DSL_ACCEL_FS4G_SENSITIVITY_UG, &val[0]);
		sensor_ug_to_ms2(data->y * LSM6DSL_ACCEL_FS4G_SENSITIVITY_UG, &val[1]);
		sensor_ug_to_ms2(data->z * LSM6DSL_ACCEL_FS4G_SENSITIVITY_UG, &val[2]);
		break;
	case SENSOR_CHAN_GYRO_X:
		sensor_10udegrees_to_rad(data->gx * LSM6DSL_GYRO_FS500_SENSITIVITY_10UDEG, val);
		break;
	case SENSOR_CHAN_GYRO_Y:
		sensor_10udegrees_to_rad(data->gy * LSM6DSL_GYRO_FS500_SENSITIVITY_10UDEG, val);
		break;
	case SENSOR_CHAN_GYRO_Z:
		sensor_10udegrees_to_rad(data->gz * LSM6DSL_GYRO_FS500_SENSITIVITY_10UDEG, val);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		sensor_10udegrees_to_rad(data->gx * LSM6DSL_GYRO_FS500_SENSITIVITY_10UDEG, &val[0]);
		sensor_10udegrees_to_rad(data->gy * LSM6DSL_GYRO_FS500_SENSITIVITY_10UDEG, &val[1]);
		sensor_10udegrees_to_rad(data->gz * LSM6DSL_GYRO_FS500_SENSITIVITY_10UDEG, &val[2]);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int lsm6dsl_custom_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
				       sensor_trigger_handler_t handler)
{
	struct lsm6dsl_custom_data *data = dev->data;

	if (trig->type != SENSOR_TRIG_FIFO_WATERMARK) {
		return -ENOTSUP;
	}

	data->trigger = trig;
	data->handler = handler;

	return 0;
}

static DEVICE_API(sensor, lsm6dsl_custom_api) = {
	.sample_fetch = lsm6dsl_custom_sample_fetch,
	.channel_get = lsm6dsl_custom_channel_get,
	.trigger_set = lsm6dsl_custom_trigger_set,
};

/*
 * Drains whatever is currently in the FIFO in one I2C burst read (the
 * FIFO output register auto-increments per read), then walks the drained
 * samples and invokes the app's registered trigger handler once per
 * sample. This runs in the driver's own thread (lsm6dsl_custom_thread),
 * never in interrupt context: i2c_transfer() is blocking and thread-context
 * only in Zephyr, which is why the GPIO ISR below does nothing but signal
 * a semaphore.
 */
static void lsm6dsl_custom_drain_fifo(const struct device *dev)
{
	const struct lsm6dsl_custom_config *cfg = dev->config;
	struct lsm6dsl_custom_data *data = dev->data;
	uint8_t status[2];
	uint8_t buf[LSM6DSL_FIFO_MAX_DRAIN_SAMPLES * 12];
	uint16_t words;
	uint16_t samples;
	int ret;

	ret = i2c_burst_read_dt(&cfg->i2c, LSM6DSL_REG_FIFO_STATUS1, status, sizeof(status));
	if (ret < 0) {
		LOG_ERR("failed to read FIFO status: %d", ret);
		return;
	}

	/* DIFF_FIFO[10:0]: unread 16-bit words in the FIFO. Gyro + accel both
	 * in the FIFO, so 6 words (gyro X/Y/Z, accel X/Y/Z) per sample.
	 */
	words = (((uint16_t)status[1] & 0x07) << 8) | status[0];
	samples = words / 6;

	if (samples == 0) {
		return;
	}
	if (samples > LSM6DSL_FIFO_MAX_DRAIN_SAMPLES) {
		LOG_WRN("FIFO has more samples than the drain buffer, dropping the rest");
		samples = LSM6DSL_FIFO_MAX_DRAIN_SAMPLES;
	}

	ret = i2c_burst_read_dt(&cfg->i2c, LSM6DSL_REG_FIFO_DATA_OUT_L, buf, samples * 12);
	if (ret < 0) {
		LOG_ERR("failed to read FIFO data: %d", ret);
		return;
	}

	for (uint16_t i = 0; i < samples; i++) {
		data->gx = sys_get_le16(&buf[i * 12 + 0]);
		data->gy = sys_get_le16(&buf[i * 12 + 2]);
		data->gz = sys_get_le16(&buf[i * 12 + 4]);
		data->x = sys_get_le16(&buf[i * 12 + 6]);
		data->y = sys_get_le16(&buf[i * 12 + 8]);
		data->z = sys_get_le16(&buf[i * 12 + 10]);

		if (data->handler != NULL) {
			data->handler(dev, data->trigger);
		}
	}
}

static void lsm6dsl_custom_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	const struct lsm6dsl_custom_config *cfg = dev->config;
	struct lsm6dsl_custom_data *data = dev->data;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		k_sem_take(&data->fifo_sem, K_FOREVER);

		/* Keep draining while the pin is still active: a single
		 * drain only pulls up to LSM6DSL_FIFO_MAX_DRAIN_SAMPLES. If
		 * there's a larger backlog than that (e.g. catching up from
		 * the manual kick in init, or the thread having fallen
		 * behind), one drain may not bring the FIFO back below
		 * watermark, and since this is edge-triggered there is no
		 * future edge to rely on if the pin never actually goes
		 * inactive in between. Loop until it does.
		 */
		do {
			lsm6dsl_custom_drain_fifo(dev);
		} while (gpio_pin_get_dt(&cfg->irq_gpio) > 0);
	}
}

/* Runs in interrupt context: do no I2C work here, only signal the thread. */
static void lsm6dsl_custom_gpio_callback(const struct device *port, struct gpio_callback *cb,
					  uint32_t pins)
{
	struct lsm6dsl_custom_data *data =
		CONTAINER_OF(cb, struct lsm6dsl_custom_data, gpio_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	k_sem_give(&data->fifo_sem);
}

static int lsm6dsl_custom_init(const struct device *dev)
{
	const struct lsm6dsl_custom_config *cfg = dev->config;
	struct lsm6dsl_custom_data *data = dev->data;
	uint8_t who_am_i;
	int ret;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	ret = i2c_reg_read_byte_dt(&cfg->i2c, LSM6DSL_REG_WHO_AM_I, &who_am_i);
	if (ret < 0) {
		LOG_ERR("failed to read WHO_AM_I: %d", ret);
		return ret;
	}
	if (who_am_i != LSM6DSL_WHO_AM_I_VAL) {
		LOG_ERR("unexpected WHO_AM_I: 0x%02x", who_am_i);
		return -ENODEV;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, LSM6DSL_REG_CTRL3_C, LSM6DSL_CTRL3_C_SW_RESET);
	if (ret < 0) {
		LOG_ERR("failed to reset device: %d", ret);
		return ret;
	}
	k_msleep(LSM6DSL_RESET_DELAY_MS);

	ret = i2c_reg_write_byte_dt(&cfg->i2c, LSM6DSL_REG_CTRL3_C, LSM6DSL_CTRL3_C_CONFIG_VAL);
	if (ret < 0) {
		LOG_ERR("failed to configure CTRL3_C: %d", ret);
		return ret;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, LSM6DSL_REG_CTRL1_XL,
				     LSM6DSL_ODR_104HZ | LSM6DSL_FS_XL_4G);
	if (ret < 0) {
		LOG_ERR("failed to configure CTRL1_XL: %d", ret);
		return ret;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, LSM6DSL_REG_CTRL2_G,
				     LSM6DSL_ODR_G_104HZ | LSM6DSL_FS_G_500DPS);
	if (ret < 0) {
		LOG_ERR("failed to configure CTRL2_G: %d", ret);
		return ret;
	}

	/*
	 * Arm the GPIO interrupt and start the servicing thread BEFORE
	 * enabling FIFO collection below. The watermark interrupt is
	 * edge-triggered: it only fires on the transition from below- to
	 * at-watermark. If the FIFO were already running when the interrupt
	 * gets armed, a fill that crosses the watermark in that window would
	 * be a missed edge -- and since nothing would ever be draining the
	 * FIFO, the watermark condition just sits there permanently true
	 * with no further edge to catch. Net effect: total silence, no
	 * error, forever. Arming first closes that race.
	 */
	if (!device_is_ready(cfg->irq_gpio.port)) {
		LOG_ERR("IRQ GPIO port not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("failed to configure IRQ GPIO: %d", ret);
		return ret;
	}

	/* Register the callback before arming the interrupt below: an edge
	 * landing between "interrupt armed" and "callback registered" would
	 * fire the port's shared ISR, find no callback in the list yet, and
	 * the pending flag gets cleared with nobody told -- same missed-edge
	 * problem as the FIFO-enable-order issue above, one level deeper.
	 */
	gpio_init_callback(&data->gpio_cb, lsm6dsl_custom_gpio_callback, BIT(cfg->irq_gpio.pin));
	ret = gpio_add_callback(cfg->irq_gpio.port, &data->gpio_cb);
	if (ret < 0) {
		LOG_ERR("failed to add IRQ GPIO callback: %d", ret);
		return ret;
	}

	k_sem_init(&data->fifo_sem, 0, 1);

	/* STM32's EXTI hardware is edge-only -- GPIO_INT_LEVEL_ACTIVE returns
	 * -ENOTSUP on this platform, confirmed on hardware. So this has to
	 * stay edge-triggered, which reopens the missed-edge race described
	 * above at a finer grain: if the watermark condition became true
	 * (FIFO already at/above threshold) at any point before this exact
	 * call arms the interrupt, there is no edge left to catch and the
	 * driver goes silent forever with the FIFO stuck overrunning. Poll
	 * the pin once immediately after arming and manually kick the
	 * semaphore if it's already active, closing that race by hand
	 * instead of relying on hardware level-sensitivity we don't have.
	 */
	ret = gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		LOG_ERR("failed to configure IRQ GPIO interrupt: %d", ret);
		return ret;
	}

	if (gpio_pin_get_dt(&cfg->irq_gpio) > 0) {
		k_sem_give(&data->fifo_sem);
	}

	k_thread_create(&data->thread, data->thread_stack,
			K_KERNEL_STACK_SIZEOF(data->thread_stack), lsm6dsl_custom_thread,
			(void *)dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_LSM6DSL_CUSTOM_THREAD_PRIORITY), 0, K_NO_WAIT);
	k_thread_name_set(&data->thread, "lsm6dsl_custom");

	ret = i2c_reg_write_byte_dt(&cfg->i2c, LSM6DSL_REG_FIFO_CTRL1, LSM6DSL_FIFO_CTRL1_VAL);
	if (ret < 0) {
		LOG_ERR("failed to configure FIFO_CTRL1: %d", ret);
		return ret;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, LSM6DSL_REG_FIFO_CTRL2, LSM6DSL_FIFO_CTRL2_VAL);
	if (ret < 0) {
		LOG_ERR("failed to configure FIFO_CTRL2: %d", ret);
		return ret;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, LSM6DSL_REG_FIFO_CTRL3, LSM6DSL_FIFO_CTRL3_VAL);
	if (ret < 0) {
		LOG_ERR("failed to configure FIFO_CTRL3: %d", ret);
		return ret;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, LSM6DSL_REG_INT1_CTRL, LSM6DSL_INT1_CTRL_FTH);
	if (ret < 0) {
		LOG_ERR("failed to configure INT1_CTRL: %d", ret);
		return ret;
	}

	/* FIFO_MODE goes last: the interrupt is already armed and the
	 * servicing thread already running by the time this actually starts
	 * FIFO collection.
	 */
	ret = i2c_reg_write_byte_dt(&cfg->i2c, LSM6DSL_REG_FIFO_CTRL5, LSM6DSL_FIFO_CTRL5_VAL);
	if (ret < 0) {
		LOG_ERR("failed to configure FIFO_CTRL5: %d", ret);
		return ret;
	}

	return 0;
}

#define LSM6DSL_CUSTOM_INIT(inst)                                                          \
	static struct lsm6dsl_custom_data lsm6dsl_custom_data_##inst;                       \
	static const struct lsm6dsl_custom_config lsm6dsl_custom_config_##inst = {          \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                           \
		.irq_gpio = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),                          \
	};                                                                                    \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, lsm6dsl_custom_init, NULL,                       \
				      &lsm6dsl_custom_data_##inst,                           \
				      &lsm6dsl_custom_config_##inst, POST_KERNEL,            \
				      CONFIG_SENSOR_INIT_PRIORITY, &lsm6dsl_custom_api);

DT_INST_FOREACH_STATUS_OKAY(LSM6DSL_CUSTOM_INIT)
