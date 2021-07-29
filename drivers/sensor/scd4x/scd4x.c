/* scd4x.c - Driver for SENSIRION co2 temperature and RH sensor */

/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sensirion_scd4x

#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <sys/byteorder.h>
#include <sys/crc.h>

#include "scd4x.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(sensirion_scd4x, CONFIG_SENSOR_LOG_LEVEL);

#define CRC8_POLYNOMIAL 0x31
#define CRC8_INIT_VAL 0xFF

struct scd4x_data {
	struct device *i2c_dev;
	int32_t co2;
	int32_t temperature;
	int32_t rh;
};

static int sensirion_scd4x_data_available(const struct scd4x_data *drv_data)
{
	uint8_t i2c_data[3];
	uint8_t i2c_cmd[SCD4x_CMD_LEN];
	int status;
	uint8_t crc_res;

	i2c_cmd[0] = SCD4x_CMD_GET_DATA_READY_STATUS_MSB;
	i2c_cmd[1] = SCD4x_CMD_GET_DATA_READY_STATUS_LSB;

	status = i2c_write(drv_data->i2c_dev,
			   i2c_cmd,
			   SCD4x_CMD_LEN,
			   DT_PROP_BY_IDX(DT_DRV_INST(0), reg, 0));
	if (status < 0) {
		LOG_ERR("Unable to send request to check if data is available");
		return status;
	}

	k_sleep(K_MSEC(1)); /* Execution Time Delay */

	status = i2c_read(
		drv_data->i2c_dev, i2c_data, 3, DT_PROP_BY_IDX(DT_DRV_INST(0), reg, 0));
	if (status < 0) {
		LOG_ERR("Unable check if data is available due to I2C failure");
		return status;
	}

	crc_res = crc8((i2c_data + 0), 2, CRC8_POLYNOMIAL, CRC8_INIT_VAL, false);
	if (i2c_data[2] != crc_res) {
		LOG_ERR("CRC not valid during scd4x measurment reading");
		return -EIO;
	}

	/* If 11 LSB are 0 then data is not ready */
	if (((i2c_data[1] & 0xFF) == 0) && ((i2c_data[0] & 0x07) == 0)) {
		return 0;
	}
	return 1;
}

static int sensirion_scd4x_read_meas(struct device *dev,
				     struct scd4x_data *drv_data)
{
	uint8_t i2c_data[9], i2c_cmd[2];
	uint8_t crc_res;
	uint16_t temperature_i2c, rh_i2c;
	int status;
	int i;

	/* Check if data is available */
	status = sensirion_scd4x_data_available(drv_data);
	if (status <= 0) {
		LOG_ERR("Data is not available by scd4x");
		return -EIO;
	}

	i2c_cmd[0] = SCD4x_CMD_READ_MEAS_MSB;
	i2c_cmd[1] = SCD4x_CMD_READ_MEAS_LSB;

	status = i2c_write(drv_data->i2c_dev,
			   i2c_cmd,
			   SCD4x_CMD_LEN,
			   DT_PROP_BY_IDX(DT_DRV_INST(0), reg, 0));
	if (status < 0) {
		LOG_ERR("Unable to send request to read measurments");
		return status;
	}

	k_sleep(K_MSEC(1));

	status = i2c_read(
		drv_data->i2c_dev, i2c_data, 9, DT_PROP_BY_IDX(DT_DRV_INST(0), reg, 0));
	if (status < 0) {
		LOG_ERR("Unable to read measurments due to I2C bus failure");
		return status;
	}

	/* CRC checks for received data */
	for (i = 0; i < 3; i++) {
		crc_res = crc8(
			(i2c_data + 0 + i * 3), 2, CRC8_POLYNOMIAL, CRC8_INIT_VAL, false);
		if (i2c_data[2 + i * 3] != crc_res) {
			LOG_ERR("CRC not valid during scd4x measurment reading");
			return -EIO;
		}
	}

	/* Get CO2 */
	drv_data->co2 = sys_get_be16(i2c_data);

	/* Get Temperature */
	temperature_i2c = sys_get_be16(i2c_data + 3);
	drv_data->temperature = -45 + 175 * temperature_i2c / 65536;

	/* Get RH */
	rh_i2c = sys_get_be16(i2c_data + 6);
	drv_data->rh = 100 * rh_i2c / 65536;

	return 0;
}

static int sensirion_scd4x_sample_fetch(struct device *dev,
					enum sensor_channel chan)
{
	struct scd4x_data *data;
	int status = 0;

	data = dev->driver_data;

	switch (chan) {
	case SENSOR_CHAN_CO2:
	case SENSOR_CHAN_AMBIENT_TEMP:
	case SENSOR_CHAN_HUMIDITY:
	case SENSOR_CHAN_ALL:
		status = sensirion_scd4x_read_meas(dev, data);
		if (status < 0) {
			LOG_ERR("Failed to read data");
		}
		break;
	default:
		return -ENOTSUP;
	}

	return status;
}

static int sensirion_scd4x_attr_set(struct device *dev,
				    enum sensor_channel chan,
				    enum sensor_attribute attr,
				    const struct sensor_value *val)
{
	struct scd4x_data *drv_data = dev->driver_data;
	int status;
	uint8_t i2c_data[5];

	ARG_UNUSED(chan);

	switch (attr) {
	case SENSOR_ATTR_DEEP_SLEEP_MODE:
	case SENSOR_ATTR_LP_MODE:
		i2c_data[0] = SCD4x_CMD_STOP_PERIODIC_MEAS_MSB;
		i2c_data[1] = SCD4x_CMD_STOP_PERIODIC_MEAS_LSB;
		status = i2c_write(drv_data->i2c_dev,
				   i2c_data,
				   SCD4x_CMD_LEN,
				   DT_PROP_BY_IDX(DT_DRV_INST(0), reg, 0));
		break;
	case SENSOR_ATTR_RUN_MODE:
		i2c_data[0] = SCD4x_CMD_START_PERIODIC_MEAS_MSB;
		i2c_data[1] = SCD4x_CMD_START_PERIODIC_MEAS_LSB;
		status = i2c_write(drv_data->i2c_dev,
				   i2c_data,
				   SCD4x_CMD_LEN,
				   DT_PROP_BY_IDX(DT_DRV_INST(0), reg, 0));
		break;
	case SENSOR_ATTR_CALIB_TARGET:
		status =
			sensirion_scd4x_attr_set(dev, chan, SENSOR_ATTR_LP_MODE, val);
		if (status) {
			break;
		}
		/* Duration until sensor will respond again after setting in LP Mode */
		k_sleep(K_MSEC(500));

		i2c_data[0] = SCD4x_CMD_SET_AUTOMATIC_SELF_CALIBRATION_ENABLED_MSB;
		i2c_data[1] = SCD4x_CMD_SET_AUTOMATIC_SELF_CALIBRATION_ENABLED_LSB;
		i2c_data[2] = 0x00;
		if (val->val1 == 0) {
			i2c_data[3] = 0x00;
		} else {
			i2c_data[3] = 0x01;
		}
		i2c_data[4] =
			crc8((i2c_data + 2), 2, CRC8_POLYNOMIAL, CRC8_INIT_VAL, false);
		status = i2c_write(drv_data->i2c_dev,
				   i2c_data,
				   (SCD4x_CMD_LEN + 3),
				   DT_PROP_BY_IDX(DT_DRV_INST(0), reg, 0));
		sensirion_scd4x_attr_set(dev, chan, SENSOR_ATTR_RUN_MODE, val);
		break;
	default:
		status = -ENOTSUP;
	}

	return status;
}

static int sensirion_scd4x_channel_get(struct device *dev,
				       enum sensor_channel chan,
				       struct sensor_value *val)
{
	struct scd4x_data *drv_data = dev->driver_data;

	switch (chan) {
	case SENSOR_CHAN_CO2:
		val->val1 = drv_data->co2;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_AMBIENT_TEMP:
		val->val1 = drv_data->temperature;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_HUMIDITY:
		val->val1 = drv_data->rh;
		val->val2 = 0;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int sensirion_scd4x_issue_raw_cmd(struct device *dev,
					 const uint8_t *cmd,
					 size_t size)
{
	struct scd4x_data *drv_data;
	int status;

	drv_data = dev->driver_data;

	if (size != SCD4x_CMD_LEN) {
		LOG_ERR("Size %d not supported (should be %u)", size, SCD4x_CMD_LEN);
		return -ENOTSUP;
	}

	status = i2c_write(drv_data->i2c_dev,
			   cmd,
			   SCD4x_CMD_LEN,
			   DT_PROP_BY_IDX(DT_DRV_INST(0), reg, 0));

	return status;
}

static const struct sensor_driver_api sensirion_scd4x_driver_api = {
	.sample_fetch = sensirion_scd4x_sample_fetch,
	.channel_get = sensirion_scd4x_channel_get,
	.attr_set = sensirion_scd4x_attr_set,
	.issue_raw_cmd = sensirion_scd4x_issue_raw_cmd,
};

static int sensirion_scd4x_init(struct device *dev)
{
	struct scd4x_data *drv_data;

	drv_data = dev->driver_data;

	drv_data->i2c_dev = device_get_binding(DT_INST_BUS_LABEL(0));
	if (drv_data->i2c_dev == NULL) {
		LOG_DBG("Failed to initialize %s device!", DT_INST_BUS_LABEL(0));
		return -EINVAL;
	}

	return 0;
}

static struct scd4x_data scd4x_driver;

DEVICE_AND_API_INIT(sensirion_scd4x,
		    DT_INST_LABEL(0),
		    sensirion_scd4x_init,
		    &scd4x_driver,
		    NULL,
		    POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY,
		    &sensirion_scd4x_driver_api);
