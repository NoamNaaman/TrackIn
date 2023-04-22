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

void error_handle();

zmod4xxx_dev_t dev;

/* Sensor specific variables */
uint8_t zmod4xxx_status;
uint8_t prod_data[ZMOD4450_PROD_DATA_LEN];
//uint8_t adc_result[ZMOD4450_ADC_DATA_LEN] = { 0 };
float adc_result[ZMOD4450_ADC_DATA_LEN] = { 0 };
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

    // /* Check if measurement is running. */
    // if (zmod4xxx_status & STATUS_SEQUENCER_RUNNING_MASK) {
    //     /*
    //     * Check if reset during measurement occured. For more information,
    //     * read the Programming Manual, section "Error Codes".
    //     */
    //     api_ret = zmod4xxx_check_error_event(&dev);
    //     switch (api_ret) {
    //     case ERROR_POR_EVENT:
    //         Serial.print(F("Error "));
    //         Serial.print(api_ret);
    //         Serial.println(F("Measurement completion fault. Unexpected sensor reset.\n"));
    //         break;
    //     case ZMOD4XXX_OK:
    //         Serial.print(F("Error "));
    //         Serial.print(api_ret);
    //         Serial.println(F("Measurement completion fault. Wrong sensor setup.\n"));
    //         break;
    //     default:
    //         Serial.print(F("Error "));
    //         Serial.print(api_ret);
    //         Serial.println(F("Error during reading status register "));
    //         break;
    //     }
    //      error_handle();
    // }

    /* Read sensor ADC output. */
    api_ret = zmod4xxx_read_adc_result(&dev, *adc_result);
    if (api_ret) {
        Serial.print(F("Error "));
        Serial.print(api_ret);
        Serial.println(F("Error %d during read of ADC results, exiting program!\n"));
        error_handle();
    }

    /*
    * Check validity of the ADC results. For more information, read the
    * Programming Manual, section "Error Codes".
    */
    api_ret = zmod4xxx_check_error_event(&dev);
    if (api_ret) {
        Serial.print(F("Error "));
        Serial.print(api_ret);
        Serial.println(F("during reading status register, exiting program!\n"));
        error_handle();
    }


    // algo_input.adc_result = adc_result;
    /*
	* The ambient compensation needs humidity [RH] and temperature [DegC]
    * measurements! Input them here.
	*/
    // algo_input.humidity_pct = 50.0;
    // algo_input.temperature_degc = 20.0;

   /* To work with the algorithms target specific libraries needs to be
     * downloaded from IDT webpage and included into the project */

    /* get raq control signal and Air Quality Change Rate */
    api_ret = calc_raq( *adc_result, &raq_par, &raq_results);

    printf("\n");
    printf("Measurement:\n");
    printf("  Rmox  = %5.0f kOhm\n", (adc_result / 1000.0));
    if (ZMOD4450_OK == api_ret) {
        printf("  raq: control state %d\n", raq_results.cs_state);
        printf("  raq: Air Quality Change Rate %f\n", raq_results.conc_ratio);
    }
    else {
        printf("  raq: control state: Sensor not stabilized\n");
        printf("  raq: Air Quality Change Rate: Sensor not stabilized\n");
    }

    /* INSTEAD OF POLLING THE INTERRUPT CAN BE USED FOR OTHER HW */
    /* waiting for sensor ready */
    while (FIRST_SEQ_STEP != (zmod4xxx_status & STATUS_LAST_SEQ_STEP_MASK)) {
        dev.delay_ms(50);
        api_ret = zmod4xxx_read_status(dev, &zmod4xxx_status);
        if(api_ret) {
            printf("Error %d, exiting program!\n", api_ret);
        }
    }

    /* Calculate algorithm results */
    // lib_ret =  calc_oaq_2nd_gen(&algo_handle, &dev, &algo_input, &algo_results);
    // /* Skip 900 stabilization samples for oaq_2nd_gen algorithm. */
    // if ((lib_ret != OAQ_2ND_GEN_OK) && (lib_ret != OAQ_2ND_GEN_STABILIZATION)) {
    //     Serial.println(F("Error when calculating algorithm, exiting program!"));
    // } else {
    //     Serial.println("*********** Measurements ***********");
    //     for (int i = 0; i < 8; i++) {
    //         Serial.print(" Rmox[");
    //         Serial.print(i);
    //         Serial.print("] = ");
    //         Serial.print(algo_results.rmox[i] / 1e3);
    //         Serial.println(" kOhm");
    //     }
    //     Serial.print(" O3_conc_ppb = ");
    //     Serial.println(algo_results.O3_conc_ppb);
    //     Serial.print(" Fast AQI = ");
    //     Serial.println(algo_results.FAST_AQI);
    //     Serial.print(" EPA AQI = ");
    //     Serial.println(algo_results.EPA_AQI);


    //     if (lib_ret == OAQ_2ND_GEN_STABILIZATION) {
    //         Serial.println(F("Warm-Up!"));
    //     } else {
    //         Serial.println(F("Valid!"));
    //     }
    // }


}

void error_handle()
{
    while (1);
}
