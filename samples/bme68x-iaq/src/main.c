/*
 * BME68x + BSEC IAQ demo (based on bme68x module sample)
 * Target: NUCLEO-F767ZI
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

#ifdef CONFIG_BME68X_IAQ_SETTINGS
#include <zephyr/settings/settings.h>
#endif

/* This header is provided by the bme68x zephyr module (sensor api wrapper) */
#include <drivers/bme68x_sensor_api.h>

/* Bosch Sensor API device struct */
#include "bme68x.h"

/* IAQ library interface from the module */
#include "bme68x_iaq.h"

LOG_MODULE_REGISTER(bme68x_iaq_demo, LOG_LEVEL_INF);

/* Print helper (avoid float printf requirements by formatting carefully) */
static const char *accuracy2str(enum bme68x_iaq_accuracy a)
{
	switch (a) {
	case BME68X_IAQ_ACCURACY_UNRELIABLE: return "unreliable";
	case BME68X_IAQ_ACCURACY_LOW:        return "low";
	case BME68X_IAQ_ACCURACY_MEDIUM:     return "medium";
	case BME68X_IAQ_ACCURACY_HIGH:       return "high";
	default:                             return "?";
	}
}

static const char *stab2str(enum bme68x_iaq_status s)
{
	switch (s) {
	case BME68X_IAQ_STAB_ONGOING:  return "ongoing";
	case BME68X_IAQ_STAB_FINISHED: return "finished";
	default:                       return "?";
	}
}

#if 0
/* Callback invoked by IAQ run-loop each time BSEC produces outputs */
/* Floating-point print variant */
static void iaq_output_handler_flt(const struct bme68x_iaq_sample *s)
{
	/* Raw signals are already “physical units” as floats from the library */
	LOG_INF("T=%.2f C  H=%.2f %%  P=%.2f hPa  Gas=%.0f ohm | IAQ=%.1f (%s) | CO2eq=%.0f ppm (%s) | VOC=%.2f ppm (%s) | stab=%s run=%s",
		(double)s->temperature,
		(double)s->humidity,
		(double)s->raw_pressure,        /* depends on library: may already be hPa */
		(double)s->raw_gas_res,         /* ohms */
		(double)s->iaq,
		accuracy2str(s->iaq_accuracy),
		(double)s->co2_equivalent,
		accuracy2str(s->co2_accuracy),
		(double)s->voc_equivalent,
		accuracy2str(s->voc_accuracy),
		stab2str(s->stab_status),
		stab2str(s->run_status));
}
#else
/* Integer print variant (fixed-point scaled or rounded as appropriate) */
static void iaq_output_handler_int(const struct bme68x_iaq_sample *s)
{
	LOG_INF("T=%d.%02d C  H=%d.%02d %%  P=%d.%02d hPa  Gas=%ld ohm | IAQ=%d.%01d (%s) | CO2eq=%ld ppm (%s) | VOC=%d.%02d ppm (%s) | stab=%s run=%s",
		(int)s->temperature, (int)((s->temperature - (int)s->temperature) * 100),
		(int)s->humidity, (int)((s->humidity - (int)s->humidity) * 100),
		(int)s->raw_pressure, (int)((s->raw_pressure - (int)s->raw_pressure) * 100),
		(long)s->raw_gas_res,
		(int)s->iaq, (int)((s->iaq - (int)s->iaq) * 10),
		accuracy2str(s->iaq_accuracy),
		(long)s->co2_equivalent,
		accuracy2str(s->co2_accuracy),
		(int)s->voc_equivalent, (int)((s->voc_equivalent - (int)s->voc_equivalent) * 100),
		accuracy2str(s->voc_accuracy),
		stab2str(s->stab_status),
		stab2str(s->run_status));
}
#endif


int main(void)
{
	int ret;

	LOG_INF("*** BME68x + BSEC IAQ Demo (module-based) ***");

	/* Get any compatible BME68x Sensor-API device from DTS */
	const struct device *dev = DEVICE_DT_GET_ONE(bosch_bme68x_sensor_api);
	if (!device_is_ready(dev)) {
		LOG_ERR("BME68x sensor-api device not ready");
		return 0;
	}

	/* Initialize Bosch Sensor API device struct via Zephyr wrapper */
	struct bme68x_dev bme = {0};

	ret = bme68x_sensor_api_init(dev, &bme);
	if (ret) {
		LOG_ERR("bme68x_sensor_api_init failed: %d", ret);
		return 0;
	}

	ret = bme68x_init(&bme);
	if (ret) {
		LOG_ERR("bme68x_init failed: %d", ret);
		return 0;
	}

#ifdef CONFIG_BME68X_IAQ_SETTINGS
	/* Only needed if you enable settings-based state saving */
	ret = settings_subsys_init();
	if (ret) {
		LOG_ERR("settings_subsys_init failed: %d", ret);
		return 0;
	}
#endif

	/*
	 * CRITICAL: This activates IAQ/BSEC scheduling and heater profiles.
	 * If you skip this, heater never runs -> gas resistance stays constant -> IAQ = NA.
	 */
	ret = bme68x_iaq_init();
	if (ret) {
		LOG_ERR("bme68x_iaq_init failed: %d", ret);
		return 0;
	}

	LOG_INF("Init OK. Entering IAQ sample loop...");

	struct bme68x_iaq_sample sample;
#if defined(CONFIG_BME68X_IAQ_SAMPLE_RATE_ULP)
	k_timeout_t period = K_SECONDS(300);
#else
	k_timeout_t period = K_SECONDS(3);
#endif

	while (true) {
		ret = bme68x_iaq_sample(&bme, &sample);
		if (ret == 0) {
			// TODO: Float causes -134 in the lib, use only int for now
			iaq_output_handler_int(&sample);
		} else if (ret != -EAGAIN) {
			LOG_ERR("IAQ sample failed: %d", ret);
			break;
		}
		k_sleep(period);
	}

	return 0;
}
