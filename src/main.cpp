/*******************************************************************************
 * Copyright (c) 2022 Renesas Electronics Corporation
 * All Rights Reserved.
 *
 * This code is proprietary to Renesas, and is license pursuant to the terms and
 * conditions that may be accessed at:
 * https://www.renesas.com/eu/en/document/msc/renesas-software-license-terms-gas-sensor-software
 *
 ******************************************************************************/

/**
 * @file    oaq_2nd_gen.ino
 * @brief   This is an Arduino example for the ZMOD4510 gas sensor using the oaq_2nd_gen library.
 * @version 4.0.0
 * @author Renesas Electronics Corporation
 */


#include <raq.h>
#include <zmod4450_config_raq.h>
#include <zmod4xxx.h>
#include <zmod4xxx_hal.h>
#include <oaq_2nd_gen.h>
// start sequencer defines

#define FIRST_SEQ_STEP      0
#define LAST_SEQ_STEP       1

static raq_params raq_par = {
        .alpha = 0.8,
        .stop_delay = 24,
        .threshold = 1.3,
        .tau = 720,
        .stabilization_samples = 15,
};

raq_results_t raq_results = { .cs_state = OFF, .conc_ratio = 0.0F };
uint8_t state;
char buffer[50];

void error_handle();

zmod4xxx_dev_t dev;

/* Sensor specific variables */
uint8_t zmod4xxx_status;
uint8_t prod_data[ZMOD4450_PROD_DATA_LEN];
//uint8_t adc_result[ZMOD4450_ADC_DATA_LEN] = { 0 };
// float adc_result[ZMOD4450_ADC_DATA_LEN] = { 0 };
float adc_result;
uint8_t track_number[ZMOD4XXX_LEN_TRACKING];
oaq_2nd_gen_handle_t algo_handle;
oaq_2nd_gen_results_t algo_results;
oaq_2nd_gen_inputs_t algo_input;


void setup()
{
    int8_t lib_ret;
    zmod4xxx_err api_ret;

    Serial.begin(115200);

    /**
        Additional delay is required to wait till system is ready.
        It is used for MKRZERO platform.
    */
    delay(2000);

    Serial.println(F("Starting the Sensor!"));
    /****TARGET SPECIFIC FUNCTION ****/
    /*
	* To allow the example running on customer-specific hardware, the init_hardware
	* function must be adapted accordingly. The mandatory funtion pointers *read,
	* *write and *delay require to be passed to "dev" (reference files located
	* in "dependencies/zmod4xxx_api/HAL" directory). For more information, read
	* the Datasheet, section "I2C Interface and Data Transmission Protocol".
    */
    api_ret = init_hardware(&dev);
    if (api_ret) {
        Serial.print(F("Error "));
        Serial.print(api_ret);
        Serial.println(F(" during init hardware, exiting program!\n"));
        error_handle();
    }
     /****TARGET SPECIFIC FUNCTION ****/

    /* Sensor related data */
    dev.i2c_addr =  ZMOD4450_I2C_ADDR;
    dev.pid = ZMOD4450_PID;
    dev.init_conf = &g_zmod4450_raq_sensor_type[ZMOD_INIT];
    dev.meas_conf = &g_zmod4450_raq_sensor_type[MEASURE];
    dev.prod_data = prod_data;

    /* Read product ID and configuration parameters. */
    api_ret = zmod4xxx_read_sensor_info(&dev);
    if (api_ret) {
        Serial.print(F("Error "));
        Serial.print(api_ret);
        Serial.println(
            F(" during reading sensor information, exiting program!\n"));
        error_handle();
    }

    /*
    * Retrieve sensors unique tracking number and individual trimming information.
    * Provide this information when requesting support from Renesas.
    */

    api_ret = zmod4xxx_read_tracking_number(&dev, track_number);
    if (api_ret) {
        Serial.print(F("Error "));
        Serial.print(api_ret);
        Serial.println(F(" during preparation of the sensor, exiting program!\n"));
        error_handle();
    }
    Serial.print(F("Sensor tracking number: x0000"));
    for (uint8_t i = 0; i < sizeof(track_number); i++) {
        Serial.print(track_number[i], HEX);
    }
    Serial.println("");
    Serial.print(F("Sensor trimming data:"));
    for (uint8_t i = 0; i < sizeof(prod_data); i++) {
        Serial.print(prod_data[i]);
    }
    Serial.println("");

    /* Determine calibration parameters and configure measurement. */
    api_ret = zmod4xxx_prepare_sensor(&dev);
    if (api_ret) {
        Serial.print(F("Error "));
        Serial.print(api_ret);
        Serial.println(F(" during preparation of the sensor, exiting program!\n"));
        error_handle();
    }

    /*
    * One-time initialization of the algorithm. Handle passed to calculation
    * function.
    */
    lib_ret = init_oaq_2nd_gen(&algo_handle);
    if (lib_ret) {
        Serial.println(F("Error "));
        Serial.print(lib_ret);
        Serial.println(F(" during initializing algorithm, exiting program!"));
        error_handle();
    }

}

void loop()
{
    int8_t lib_ret;
    zmod4xxx_err api_ret;
    
    /* Start a measurement. */
    api_ret = zmod4xxx_start_measurement(&dev);
    if (api_ret) {
        Serial.print(F("Error "));
        Serial.print(api_ret);
        Serial.println(F(" during starting measurement, exiting program!\n"));
        error_handle();
    }

    /* Perform delay. Required to keep proper measurement timing. */
    dev.delay_ms(ZMOD4450_OAQ2_SAMPLE_TIME);

   /* Wait for initialization finished. */
    do {
        api_ret = zmod4xxx_read_status(&dev, &zmod4xxx_status);
        if (api_ret) {
            Serial.print(F("Error "));
            Serial.print(api_ret);
            Serial.println(F(" during read of sensor status, exiting program!\n"));
            error_handle();
        }
    } while (FIRST_SEQ_STEP != (zmod4xxx_status & STATUS_LAST_SEQ_STEP_MASK));

 
    float r_mox;
    uint8_t zmod44xx_status;
    uint8_t state;
    int8_t ret;

    /* INSTEAD OF POLLING THE INTERRUPT CAN BE USED FOR OTHER HW */
    /* wait until readout result is possible */
    ret = zmod4xxx_read_status(&dev, &zmod44xx_status);
    if(ret) {
        Serial.printf("Error %d, exiting program!\n", ret);
}
    if(LAST_SEQ_STEP != (zmod44xx_status & STATUS_LAST_SEQ_STEP_MASK))
    {
        dev.delay_ms(50);
    }

    /* evaluate and show measurement results */
    ret = zmod4xxx_read_adc_result(&dev, &adc_result);
    if(ret) {
        if(ERROR_ACCESS_CONFLICT == ret) {
            Serial.printf("Error %d, AccessConflict!\n", ret);
        }
        else if(ERROR_POR_EVENT == ret) {
            Serial.printf("Error %d, exiting program!\n", ret);
        }
    }

    /* To work with the algorithms target specific libraries needs to be
     * downloaded from IDT webpage and included into the project */

    /* get raq control signal and Air Quality Change Rate */
    state = calc_raq( adc_result, &raq_par, &raq_results);

    Serial.println("Measurement:");
    int rmox_kOhm = (int)(adc_result / 1000.0);
    sprintf(buffer, "  Rmox  = %5d kOhm\n", rmox_kOhm);
    Serial.println(buffer);
    if (ZMOD4450_OK == state) {
        sprintf(buffer,"  raq: control state %d", raq_results.cs_state);
        Serial.println(buffer);
        sprintf(buffer,"  raq: Air Quality Change Rate %f", raq_results.conc_ratio);
        Serial.println(buffer);
    }
    else {
        Serial.println("  raq: control state: Sensor not stabilized");
        Serial.println("  raq: Air Quality Change Rate: Sensor not stabilized");
    }

    /* INSTEAD OF POLLING THE INTERRUPT CAN BE USED FOR OTHER HW */
    /* waiting for sensor ready */
    while (FIRST_SEQ_STEP != (zmod4xxx_status & STATUS_LAST_SEQ_STEP_MASK)) {
        dev.delay_ms(50);
        api_ret = zmod4xxx_read_status(&dev, &zmod4xxx_status);
        if(api_ret) {
            sprintf(buffer,"Error %d, exiting program!", api_ret);
            Serial.println(buffer);
        }
    }

}

void error_handle()
{
    while (1);
}
