/*
 * Register map and config/data structs for the out-of-tree LSM6DSL driver.
 * Register addresses and encodings are from ST datasheet DocID028475
 * (docs/DS_lsm6dsl.pdf), section 9.
 */
#ifndef ZEPHYR_DRIVERS_SENSOR_LSM6DSL_CUSTOM_LSM6DSL_CUSTOM_H_
#define ZEPHYR_DRIVERS_SENSOR_LSM6DSL_CUSTOM_LSM6DSL_CUSTOM_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#define LSM6DSL_REG_FIFO_CTRL1    0x06
#define LSM6DSL_REG_FIFO_CTRL2    0x07
#define LSM6DSL_REG_FIFO_CTRL3    0x08
#define LSM6DSL_REG_FIFO_CTRL5    0x0A
#define LSM6DSL_REG_INT1_CTRL     0x0D
#define LSM6DSL_REG_WHO_AM_I      0x0F
#define LSM6DSL_REG_CTRL1_XL      0x10
#define LSM6DSL_REG_CTRL2_G       0x11
#define LSM6DSL_REG_CTRL3_C       0x12
#define LSM6DSL_REG_STATUS        0x1E
/* OUTX_L_G (0x22) through OUTZ_H_XL (0x2D) are contiguous: gyro X/Y/Z then
 * accel X/Y/Z, 12 bytes, one burst read for both.
 */
#define LSM6DSL_REG_OUTX_L_G      0x22
#define LSM6DSL_REG_OUTX_L_XL     0x28
#define LSM6DSL_REG_FIFO_STATUS1  0x3A
#define LSM6DSL_REG_FIFO_STATUS2  0x3B
#define LSM6DSL_REG_FIFO_DATA_OUT_L 0x3E

#define LSM6DSL_WHO_AM_I_VAL 0x6A

/* CTRL3_C: SW_RESET is bit 0. Reflashing the MCU does not reset this
 * external I2C sensor -- its state (including the FIFO) persists across
 * MCU resets. Explicitly resetting it at init guarantees a known-clean
 * state every time instead of inheriting whatever was left running.
 */
#define LSM6DSL_CTRL3_C_SW_RESET 0x01
#define LSM6DSL_RESET_DELAY_MS   50 /* datasheet turn-on time: 35 ms typical */

/* CTRL3_C bit layout: BOOT(7) BDU(6) H_LACTIVE(5) PP_OD(4) SIM(3) IF_INC(2)
 * BLE(1) SW_RESET(0). Post-reset config write: BDU=1 (datasheet recommends
 * this for reliable multi-byte reads of FIFO_STATUS1/2 -- without it a
 * burst read can straddle an internal update and return a torn value) and
 * IF_INC=1 (auto-increment -- defaults to 1, but this is a plain byte
 * write, not read-modify-write, so it must be included explicitly or every
 * burst read in this driver, OUTX and FIFO_DATA_OUT alike, breaks).
 */
#define LSM6DSL_CTRL3_C_CONFIG_VAL 0x44

/* CTRL1_XL bit layout: ODR_XL[7:4] | FS_XL[3:2] | LPF1_BW_SEL[1] | BW0_XL[0] */
#define LSM6DSL_ODR_104HZ 0x40 /* ODR_XL = 0100 */
#define LSM6DSL_FS_XL_4G  0x08 /* FS_XL = 10 -> +/-4g */

/* Sensitivity at FS = +/-4g, in micro-g per LSB (datasheet section on
 * mechanical characteristics, LA_So).
 */
#define LSM6DSL_ACCEL_FS4G_SENSITIVITY_UG 122

/* CTRL2_G bit layout: ODR_G[7:4] | FS_G[3:2] | FS_125[1] | 0[0]. FS_G = 01
 * -> +/-500 dps: enough headroom for gyro-magnitude activity gating
 * (rejecting deliberate rotation like a hand wave, not precision angular
 * measurement) without the resolution loss of a wider range.
 */
#define LSM6DSL_ODR_G_104HZ 0x40 /* ODR_G = 0100, same encoding as accel */
#define LSM6DSL_FS_G_500DPS 0x04 /* FS_G = 01 */

/* Sensitivity at FS = +/-500 dps, in "10 micro-degree" units per LSB (the
 * unit Zephyr's sensor_10udegrees_to_rad() helper expects): datasheet
 * G_So = 17.50 mdps/LSB = 17.50 m-degree/s per LSB. Converting mdps to
 * 10u-degree units is a factor of 100 (1 mdps = 1e-3 deg = 100 * 1e-5 deg,
 * and 10u-degree = 1e-5 degree), so 17.50 * 100 = 1750 exactly.
 */
#define LSM6DSL_GYRO_FS500_SENSITIVITY_10UDEG 1750

/* FIFO_CTRL1/2: 11-bit watermark FTH_[10:0], in 16-bit words (not samples).
 * Gyro (first data set) + accel (second data set) both in the FIFO -> 6
 * words per sample (gyro X/Y/Z, accel X/Y/Z). 16 samples = 96 words.
 */
#define LSM6DSL_FIFO_WATERMARK_SAMPLES 16
#define LSM6DSL_FIFO_WATERMARK_WORDS   (LSM6DSL_FIFO_WATERMARK_SAMPLES * 6)
#define LSM6DSL_FIFO_CTRL1_VAL         (LSM6DSL_FIFO_WATERMARK_WORDS & 0xFF)
#define LSM6DSL_FIFO_CTRL2_VAL         ((LSM6DSL_FIFO_WATERMARK_WORDS >> 8) & 0x07)

/* FIFO_CTRL3: DEC_FIFO_XL[2:0] at bits[2:0], DEC_FIFO_GYRO[2:0] at bits[5:3].
 * Both default to 000 ("sensor not in FIFO"), so both must be explicitly
 * set to 001 (no decimation) to appear in the FIFO at all. Datasheet:
 * "Gyro FIFO (first data set)" / "Accelerometer FIFO (second data set)" --
 * a fixed, documented interleave order, gyro block then accel block per
 * sample period, not an undocumented pattern to reverse-engineer.
 */
#define LSM6DSL_FIFO_CTRL3_VAL 0x09

/* FIFO_CTRL5: ODR_FIFO[3:0] at bits[6:3], FIFO_MODE[2:0] at bits[2:0].
 * ODR_FIFO = 0100 (104 Hz, matches CTRL1_XL). FIFO_MODE = 110 (continuous:
 * once full, new samples overwrite the oldest, so the FIFO keeps running
 * without needing a manual reset if the acquisition thread falls behind).
 */
#define LSM6DSL_FIFO_CTRL5_VAL 0x26

/* INT1_CTRL bit order (MSB to LSB): STEP_DETECTOR, SIGN_MOT, FULL_FLAG,
 * FIFO_OVR, FTH, BOOT, DRDY_G, DRDY_XL. INT1_FTH (FIFO threshold /
 * watermark) is bit 3 -- cross-checked against INT2_CTRL, which extracts
 * cleanly from the datasheet PDF and has the identical bit ordering
 * (STEP_DELTA, STEP_COUNT_OV, FULL_FLAG, FIFO_OVR, FTH, DRDY_TEMP, DRDY_G,
 * DRDY_XL). Bit 4 is FIFO_OVR (overrun), a different interrupt.
 */
#define LSM6DSL_INT1_CTRL_FTH 0x08

#define LSM6DSL_FIFO_MAX_DRAIN_SAMPLES 32

struct lsm6dsl_custom_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec irq_gpio;
};

struct lsm6dsl_custom_data {
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t gx;
	int16_t gy;
	int16_t gz;

	struct gpio_callback gpio_cb;
	struct k_sem fifo_sem;
	struct k_thread thread;
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_LSM6DSL_CUSTOM_THREAD_STACK_SIZE);

	sensor_trigger_handler_t handler;
	const struct sensor_trigger *trigger;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_LSM6DSL_CUSTOM_LSM6DSL_CUSTOM_H_ */
