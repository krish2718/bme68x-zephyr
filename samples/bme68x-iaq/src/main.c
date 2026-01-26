/*
 * ----------------------------------------------------
 *  main.c - Zephyr RTOS Multi-Sensor Application
 * ----------------------------------------------------
 * This application performs a one-time I2C bus scan on startup,
 * then enters a continuous monitoring loop for five environmental
 * sensors.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <stdbool.h> // For the 'bool' type

/* Required for the SGP40's external processing algorithm */
#include "gas_index_algorithm.h"

/* --- I2C Scanner Functions --- */

const char *get_sensor_name(uint8_t addr)
{
	/* Maps known I2C addresses to human-readable names */
	switch (addr) {
	case 0x29: return "TSL2591 - Light Sensor";
	case 0x39: return "APDS9960 - Proximity/Light/Color";
	case 0x40: return "HTU21D - Temp/Humidity";
	case 0x59: return "SGP40 - VOC Gas Sensor";
	case 0x62: return "SCD4x - CO2 Sensor";
	case 0x77: return "BME68x - Temp/Hum/Pres/Gas";
	default: return "Unknown Device";
	}
}

void run_i2c_scanner(void)
{
	const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
	if (!device_is_ready(i2c_dev)) {
		printk("  [ERROR] I2C bus is not ready for scanning.\n");
		return;
	}

	printk("\n--- Performing 1-time I2C Bus Scan ---\n");
	uint8_t found_count = 0;
	for (uint8_t addr = 1; addr <= 127; addr++) {
		struct i2c_msg msg;
		uint8_t dummy_buf;
		msg.buf = &dummy_buf;
		msg.len = 0;
		msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

		if (i2c_transfer(i2c_dev, &msg, 1, addr) == 0) {
			printk("  [OK] Found device at 0x%02X: %s\n", addr, get_sensor_name(addr));
			found_count++;
		}
	}
	printk("--- Scan complete. Found %d devices. ---\n", found_count);
}


/* --- Multi-Sensor Application Logic --- */

/* Device Handles from Devicetree */
const struct device *const tsl2591_dev = DEVICE_DT_GET(DT_NODELABEL(tsl2591_dev));
const struct device *const apds9960_dev = DEVICE_DT_GET(DT_NODELABEL(apds9960_dev));
const struct device *const htu21d_dev = DEVICE_DT_GET(DT_NODELABEL(htu21d_dev));
const struct device *const sgp40_dev = DEVICE_DT_GET(DT_NODELABEL(sgp40_dev));
const struct device *const scd4x_dev = DEVICE_DT_GET(DT_NODELABEL(scd4x_dev));

/* Global state for the SGP40's algorithm */
static GasIndexAlgorithmParams gas_index_params;

void main(void)
{
	printk("\n\n*** Zephyr Multi-Sensor Environmental Node Initializing ***\n");

	/* --- Phase 1: Scan the I2C Bus --- */
	run_i2c_scanner();

	/* --- Phase 2: Check Sensor API readiness --- */
	if (!device_is_ready(tsl2591_dev) || !device_is_ready(apds9960_dev) ||
	    !device_is_ready(htu21d_dev) || !device_is_ready(sgp40_dev) ||
	    !device_is_ready(scd4x_dev)) {
		printk("  [ERROR] One or more sensor drivers failed to initialize. Halting.\n");
		return;
	}
	printk("  [OK] All 5 sensor drivers are ready.\n");

	GasIndexAlgorithm_init(&gas_index_params, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);
	printk("  [OK] SGP40 Gas Index Algorithm initialized.\n");
	
	printk("\n--- Starting Main Measurement Loop ---\n");
	printk("SGP40 sampling at 1Hz | Full report every 10 seconds\n");
	k_sleep(K_SECONDS(2));

	int loop_counter = 0;
	bool sgp40_is_stable = false; /* SGP40 stability flag */

	while (1) {
		/* --- SGP40: Sample every 1 second for algorithm stability --- */
		struct sensor_value raw_voc;
		int32_t voc_index;
		sensor_sample_fetch(sgp40_dev);
		sensor_channel_get(sgp40_dev, SENSOR_CHAN_GAS_RES, &raw_voc);
		GasIndexAlgorithm_process(&gas_index_params, raw_voc.val1, &voc_index);

		/* New, intelligent stability check */
		if (!sgp40_is_stable && voc_index > 0) {
			sgp40_is_stable = true;
		}

		/* --- Full Report: Read all other sensors and print every 10 seconds --- */
		if (loop_counter % 10 == 0) {
			int64_t uptime_s = k_uptime_get() / 1000;

			/* TSL2591 Data */
			struct sensor_value lux, ir;
			sensor_sample_fetch(tsl2591_dev);
			sensor_channel_get(tsl2591_dev, SENSOR_CHAN_LIGHT, &lux);
			sensor_channel_get(tsl2591_dev, SENSOR_CHAN_IR, &ir);

			/* APDS9960 Data */
			struct sensor_value light, prox, r, g, b;
			sensor_sample_fetch(apds9960_dev);
			sensor_channel_get(apds9960_dev, SENSOR_CHAN_LIGHT, &light);
			sensor_channel_get(apds9960_dev, SENSOR_CHAN_PROX, &prox);
			sensor_channel_get(apds9960_dev, SENSOR_CHAN_RED, &r);
			sensor_channel_get(apds9960_dev, SENSOR_CHAN_GREEN, &g);
			sensor_channel_get(apds9960_dev, SENSOR_CHAN_BLUE, &b);

			/* HTU21D Data */
			struct sensor_value htu_temp, htu_hum;
			sensor_sample_fetch(htu21d_dev);
			sensor_channel_get(htu21d_dev, SENSOR_CHAN_AMBIENT_TEMP, &htu_temp);
			sensor_channel_get(htu21d_dev, SENSOR_CHAN_HUMIDITY, &htu_hum);

			/* SCD4x Data */
			struct sensor_value co2, scd_temp, scd_hum;
			sensor_sample_fetch(scd4x_dev);
			sensor_channel_get(scd4x_dev, SENSOR_CHAN_CO2, &co2);
			sensor_channel_get(scd4x_dev, SENSOR_CHAN_AMBIENT_TEMP, &scd_temp);
			sensor_channel_get(scd4x_dev, SENSOR_CHAN_HUMIDITY, &scd_hum);

			/* --- Print the formatted output block --- */
			printk("\n--- Sensor Readout at %lld seconds ---\n", uptime_s);
			printk("  [Environment]\n");
			printk("    SCD4x -> CO2: %4d ppm | Temp: %5.2f C | Humidity: %5.2f %%\n",
				   co2.val1, sensor_value_to_double(&scd_temp), sensor_value_to_double(&scd_hum));
			printk("    HTU21D-> Temp: %5.2f C | Humidity: %5.2f %%\n",
				   sensor_value_to_double(&htu_temp), sensor_value_to_double(&htu_hum));

			printk("  [Air Quality]\n");
			printk("    SGP40 -> VOC Index: %3d (Raw: %5d) [Status: %s]\n",
				   voc_index, raw_voc.val1, (sgp40_is_stable ? "Stable" : "Warming Up"));
			
			printk("  [Light & Proximity]\n");
			printk("    TSL2591-> Ambient Light: %6.2f Lux (IR: %d)\n",
				   sensor_value_to_double(&lux), ir.val1);
			printk("    APDS9960-> Proximity: %3d/255 | Light: %4d | Color (R,G,B): (%d,%d,%d)\n",
				   prox.val1, light.val1, r.val1, g.val1, b.val1);
		}

		loop_counter++;
		k_sleep(K_SECONDS(1)); // Main loop runs every 1 second
	}
}