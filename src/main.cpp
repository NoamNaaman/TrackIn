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
 * @file    rel_iaq.ino
 * @brief   This is an Arduino example for the ZMOD4410 gas sensor using the rel_iaq library.
 * @version 1.0.0
 * @author Renesas Electronics Corporation
 */

#include "Arduino.h"
#include "hal_arduino.h"
#include "wire.h"
#include "sys_defs.h"

void error_handle();

int readSVM30_J_RHT(float *temperature, float *humidity);

#define __ZMOD__ 1
#define __SHTC1_IN_ZMOD__ 1

//====== ZMOD4410 module =============
#include "zmod4410_config_iaq2_ulp.h"
#include "zmod4xxx.h"
#include "zmod4xxx_cleaning.h"
#include "rel_iaq.h"

#include "zmod4xxx_hal.h"
#include "iaq_2nd_gen_ulp.h"

zmod4xxx_dev_t dev;

/* Sensor specific variables */
uint8_t zmod4xxx_status;
uint8_t prod_data[ZMOD4410_PROD_DATA_LEN];
uint8_t adc_result[ZMOD4410_ADC_DATA_LEN] = { 0 };
uint8_t track_number[ZMOD4XXX_LEN_TRACKING];
// rel_iaq_handle_t algo_handle;
// rel_iaq_results_t algo_results;
// rel_iaq_inputs_t algo_input;


    iaq_2nd_gen_ulp_handle_t algo_handle;
    iaq_2nd_gen_ulp_results_t algo_results;
    iaq_2nd_gen_ulp_inputs_t algo_input;


//====== SVM30 module ==============

// SVM30-J I2C address
#define SVM30_J_ADDR 0x44

// SVM30-J command codes
#define SVM30_J_RHT_CMD_MSB 0x2C
#define SVM30_J_RHT_CMD_LSB 0x06

// SHTC1 I2C address
#define SHTC1_ADDR 0x70

// SHTC1 command codes
#define SHTC1_RHT_CMD_MSB 0x5C
#define SHTC1_RHT_CMD_LSB 0x24



//======================================================================
uint32_t loop_counter;

void Serial_init()
{
    Serial.begin(115200);
    Serial.println(F("Serial started"));
}

void SHTC1_init(void)
{
    Wire.begin();
}


void ZMOD_setup()
{
    int8_t lib_ret;
    zmod4xxx_err api_ret;

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
    dev.i2c_addr = ZMOD4410_I2C_ADDR;
    dev.pid = ZMOD4410_PID;
    dev.init_conf = &zmod_iaq2_ulp_sensor_cfg[INIT];
    dev.meas_conf = &zmod_iaq2_ulp_sensor_cfg[MEASUREMENT];
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


//////////////////////////////
/*
    * Start the cleaning procedure. Check the Programming Manual on indications
	* of usage. IMPORTANT NOTE: The cleaning procedure can be run only once
	* during the modules lifetime and takes 1 minute (blocking).
    */
    // printf("Starting cleaning procedure. This might take up to 1 min ...\n");
    // lib_ret = zmod4xxx_cleaning_run(&dev);
    // if (ERROR_CLEANING == lib_ret) {
    //     printf("Skipping cleaning procedure. It has already been performed!\n");
    // } else if (lib_ret) {
    //     printf("Error %d during cleaning procedure, exiting program!\n", lib_ret);
    //     return;
    // }

    /* Determine calibration parameters and configure measurement. */
    lib_ret = zmod4xxx_prepare_sensor(&dev);
    if (lib_ret) {
        printf("Error %d during preparation of the sensor, exiting program!\n",
               lib_ret);
        return;
    }

//////////////////////////////



    /*
    * One-time initialization of the algorithm. Handle passed to calculation
    * function.
    */
   lib_ret = init_iaq_2nd_gen_ulp(&algo_handle);
   if (lib_ret) {
        Serial.println(F("Error "));
        Serial.print(lib_ret);
        Serial.println(F(" during initializing algorithm, exiting program!"));
        error_handle();
    }

}

void ZMOD_loop()
{
    int8_t lib_ret;
    zmod4xxx_err api_ret;
    float temperature = 0, humidity = 0;
    
    /* Start a ZMOD4410 measurement. */
    api_ret = zmod4xxx_start_measurement(&dev);
    if (api_ret) {
        Serial.print(F("Error "));
        Serial.print(api_ret);
        Serial.println(F(" during starting measurement, exiting program!\n"));
        error_handle();
    }

    /* Perform delay. Required to keep proper measurement timing. */
    dev.delay_ms(ZMOD4410_IAQ2_ULP_SAMPLE_TIME);

    /* Verify completion of measurement sequence. */
    api_ret = zmod4xxx_read_status(&dev, &zmod4xxx_status);
    if (api_ret) {
        Serial.print(F("Error "));
        Serial.print(api_ret);
        Serial.println(F(" during read of sensor status, exiting program!\n"));
        error_handle();
    }

    /* Check if measurement is running. */
    if (zmod4xxx_status & STATUS_SEQUENCER_RUNNING_MASK) {
        /*
        * Check if reset during measurement occured. For more information,
        * read the Programming Manual, section "Error Codes".
        */
        api_ret = zmod4xxx_check_error_event(&dev);
        switch (api_ret) {
        case ERROR_POR_EVENT:
            Serial.print(F("Error "));
            Serial.print(api_ret);
            Serial.println(F("Measurement completion fault. Unexpected sensor reset.\n"));
            break;
        case ZMOD4XXX_OK:
            Serial.print(F("Error "));
            Serial.print(api_ret);
            Serial.println(F("Measurement completion fault. Wrong sensor setup.\n"));
            break;
        default:
            Serial.print(F("Error "));
            Serial.print(api_ret);
            Serial.println(F("Error during reading status register "));
            break;
        }
         error_handle();
    }

    /* Read sensor ADC output. */
    api_ret = zmod4xxx_read_adc_result(&dev, adc_result);
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


    algo_input.adc_result = adc_result;
    /* Calculate algorithm results */
    //  lib_ret =  calc_rel_iaq(&algo_handle, &dev, &algo_input, &algo_results);
        algo_input.humidity_pct = 30.0;
        algo_input.temperature_degc = 23.0;

#if __SHTC1_IN_ZMOD__ == 1
        readSVM30_J_RHT(&temperature, &humidity);
        algo_input.humidity_pct = temperature;
        algo_input.temperature_degc = humidity;
#endif
        /* Calculate algorithm results. */
        lib_ret = calc_iaq_2nd_gen_ulp(&algo_handle, &dev, &algo_input, &algo_results);
        /* Skip 10 stabilization samples for iaq_2nd_gen_ulp algorithm. */
        printf("*********** Measurements ***********\n");
        Serial.print("\r\n");
        Serial.print(++loop_counter);
        Serial.printf(F(", "));
        Serial.print(temperature);
        Serial.printf(F(", "));
        Serial.print(humidity - 20);
        Serial.printf(F(", "));
        for (int i = 0; i < 13; i++) {
            // Serial.printf(F()" Rmox[%d]=", i);
            // printf("%.3f,", algo_results.rmox[i] / 1e3);
           Serial.print(F(" Rmox["));
           Serial.print(i);
           Serial.print(F("] = "));
            Serial.print(algo_results.rmox[i] / 1e3);
            Serial.print(F(", "));
        }
        Serial.print("\r\n");
        Serial.printf(F("Rcda = ")); Serial.print(algo_results.log_nonlog_rcda[0]/ 1e3); Serial.printf(F(", "));
        Serial.printf(F("EtOH = ")); Serial.print(algo_results.etoh); Serial.printf(F(", "));
        Serial.printf(F("TVOC = ")); Serial.print(algo_results.tvoc); Serial.printf(F(", "));
        Serial.printf(F("IAQ  = ")); Serial.print(algo_results.iaq); Serial.printf(F(", "));
        Serial.printf(F("eCO2 = ")); Serial.print(algo_results.eco2); Serial.printf(F("\r\n\n"));
   /* Skip 100 stabilization samples for rel_iaq algorithm. */
    // if ((lib_ret != REL_IAQ_OK) && (lib_ret != REL_IAQ_STABILIZATION)) 
    // {
    //     Serial.println(F("Error when calculating algorithm, exiting program!"));
    // } 
    // else 
    {
// //        Serial.println(F("*********** Measurements ***********"));
//         Serial.print(++loop_counter);
//         Serial.printf(F(", "));
//         Serial.print(temperature);
//         Serial.printf(F(", "));
//         Serial.print(humidity - 20);
//         Serial.printf(F(", "));
//         for (int i = 0; i < 13; i++) {
// //            Serial.print(F(" Rmox["));
// //            Serial.print(i);
// //            Serial.print(F("] = "));
//             Serial.print(algo_results.rmox[i] / 1e3);
//             Serial.print(F(", "));
//         }
// //        Serial.print(F(" Rel IAQ  = "));
//         Serial.print(algo_results.rel_iaq);
        Serial.print(F(", "));
        switch (lib_ret) {
        case REL_IAQ_STABILIZATION:
            Serial.println(0); // Warm-Up!
            break;
        case REL_IAQ_DAMAGE:
            Serial.println(-1); //Error: Sensor probably damaged. Algorithm results may be incorrect.
            break;
        case REL_IAQ_OK:
            Serial.println(1); // Valid!
            break;
        default: /* other errors */
            Serial.println(-2);// Unexpected Error during algorithm calculation: Exiting Program.
            error_handle();
        }

    }
//    if (lib_ret == REL_IAQ_STABILIZATION) {
//      dev.delay_ms(1000);
//    } else {
//      dev.delay_ms(10000-ZMOD4410_IAQ2_ULP_SAMPLE_TIME);
//    }

}

void error_handle()
{
    while (1);
}




int readSVM30_J_RHT(float *temperature, float *humidity) {
  uint8_t rht_data[6];

  // Send RHT command
  Wire.beginTransmission(SHTC1_ADDR);
  Wire.write(SHTC1_RHT_CMD_MSB);
  Wire.write(SHTC1_RHT_CMD_LSB);
  if (Wire.endTransmission() != 0) {
    return -1;
  }

  // Read RHT data
  Wire.requestFrom(SHTC1_ADDR, 6);
  if (Wire.available() != 6) {
    return -1;
  }

  for (int i = 0; i < 6; i++) {
    rht_data[i] = Wire.read();
  }

  // Decode humidity data
  uint16_t humidity_raw = (rht_data[0] << 8) | rht_data[1];
  *humidity = -6 + 125 * (humidity_raw / 65535.0);

  // Decode temperature data
  uint16_t temperature_raw = (rht_data[3] << 8) | rht_data[4];
  *temperature = -45 + 175 * (temperature_raw / 65535.0);

  return 0;
}

void SHTC1_loop() {
  float temperature, humidity;

  if (readSVM30_J_RHT(&temperature, &humidity) == 0) {
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" C, Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
  } else {
    Serial.println("Error reading SVM30-J data");
  }

  delay(1000); // Wait for a second before reading again
}

void setup()
{
  Serial_init();

#if __ZMOD__ == 1  
  ZMOD_setup();
//   Serial.println("line,temp,humid,Rmox0,Rmox1,Rmox2,Rmox3,Rmox4,Rmox5,Rmox6,Rmox7,Rmox8,Rmox9,Rmox10,Rmox11,Rmox12,RelIAQ");
#else
  SHTC1_init();
#endif
}


void loop()
{
#if __ZMOD__ == 1  
    ZMOD_loop();
#else
    SHTC1_loop();
#endif

}