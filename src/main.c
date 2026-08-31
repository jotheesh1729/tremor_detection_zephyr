#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* LSM6DSL WHO_AM_I: register 0x0F, fixed value 0x6A (datasheet section 9.12). */
#define LSM6DSL_REG_WHO_AM_I 0x0F
#define LSM6DSL_WHO_AM_I_VAL 0x6A

/* lsm6dsl@6a on i2c2 is already declared in the board's own devicetree
 * (boards/st/disco_l475_iot1/disco_l475_iot1.dts). No overlay needed.
 */
static const struct i2c_dt_spec lsm6dsl = I2C_DT_SPEC_GET(DT_NODELABEL(lsm6dsl));

int main(void)
{
	LOG_INF("booted");

	if (!device_is_ready(lsm6dsl.bus)) {
		LOG_ERR("I2C bus not ready");
		return 0;
	}

	uint8_t who_am_i = 0;
	int ret = i2c_reg_read_byte_dt(&lsm6dsl, LSM6DSL_REG_WHO_AM_I, &who_am_i);

	if (ret < 0) {
		LOG_ERR("WHO_AM_I read failed: %d", ret);
	} else if (who_am_i != LSM6DSL_WHO_AM_I_VAL) {
		LOG_ERR("WHO_AM_I mismatch: got 0x%02x, expected 0x%02x", who_am_i, LSM6DSL_WHO_AM_I_VAL);
	} else {
		LOG_INF("LSM6DSL WHO_AM_I OK: 0x%02x", who_am_i);
	}

	while (1) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
