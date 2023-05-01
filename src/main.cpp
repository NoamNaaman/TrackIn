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


#include <zmod4410_config_rel_iaq.h>
#include <zmod4xxx.h>
#include <zmod4xxx_hal.h>
#include <rel_iaq.h>

// Start SD card defines
#include <SPI.h>
#include <SD.h>

#define SD_CS_PIN PA8
#define BUTTON_PIN PB5
#define LED_PIN PA0
#define LED_SD PA1
#define DATA_SAVE_INTERVAL 10000 // in milliseconds
#define FILE_MAX_SIZE 3600 // in number of data records
#define DATA_REG_SIZE 15 // number of data registers
float data[DATA_REG_SIZE];
File dataFile;
char filename[13];
int fileIndex = 0;
unsigned long lastDataSaveTime = 0;
int dataRecordCount = 0;
bool isSavingData = false;

// Function placed at the end of the file
void error_handle();
void createNewDataFile();
void writeDataRecordToFile();

zmod4xxx_dev_t dev;

/* Sensor specific variables */
uint8_t zmod4xxx_status;
uint8_t prod_data[ZMOD4410_PROD_DATA_LEN];
uint8_t adc_result[ZMOD4410_ADC_DATA_LEN] = { 0 };
uint8_t track_number[ZMOD4XXX_LEN_TRACKING];
rel_iaq_handle_t algo_handle;
rel_iaq_results_t algo_results;
rel_iaq_inputs_t algo_input;


void setup()
{
    pinMode(SD_CS_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    pinMode(LED_SD, OUTPUT);
    Serial.begin(115200);
    Serial.println(F("Serial started"));

    Serial.println("Initializing SD card...");
    // Configure the SPI pins
    SPI.setMOSI(PA12);
    SPI.setMISO(PA11);
    SPI.setSCLK(PA5);
    // Start the SPI interface

    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD card initialization failed!");
        while (1)
        {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(100);
        }
        
        return;
    }

    Serial.println("SD card initialization done.");

    // turn off the LED
    digitalWrite(LED_SD, HIGH);         //Indicated SD card it read
    digitalWrite(LED_PIN, LOW);         // SD card not writing

    

    int8_t lib_ret;
    zmod4xxx_err api_ret;

    /**
        Additional delay is required to wait till system is ready.
        It is used for MKRZERO platform.
    */
    //delay(2000);

    Serial.println(F("Starting the Sensor!"));
    /****TARGET SPECIFIC FUNCTION ****/
    /*
	* To allow the example running on customer-specific hardware, the init_hardware
	* function must be adapted accordingly. The mandatory function pointers *read,
	* *write and *delay require to be passed to "dev" (reference files located
	* in "dependencies/zmod4xxx_api/HAL" directory). For more information, read
	* the Datasheet, section "I2C Interface and Data Transmission Protocol".
    */
    Wire.setSDA(PA10);
    Wire.setSCL(PA9);
    api_ret = init_hardware(&dev);
    if (api_ret) {
        Serial.print(F("Error "));
        Serial.print(api_ret);
        Serial.println(F(" during init hardware, exiting program!\n"));
        error_handle();
    }
     /****TARGET SPECIFIC FUNCTION ****/

    /* Sensor related data */
    dev.i2c_addr =  ZMOD4410_I2C_ADDR;
    dev.pid = ZMOD4410_PID;
    dev.init_conf = &zmod_rel_iaq_sensor_cfg[zmod4xxxINIT];
    dev.meas_conf = &zmod_rel_iaq_sensor_cfg[MEASUREMENT];
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
    lib_ret = init_rel_iaq(&algo_handle);
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
    //dev.delay_ms(ZMOD4410_REL_IAQ_SAMPLE_TIME);
    dev.delay_ms(5000);

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
    lib_ret =  calc_rel_iaq(&algo_handle, &dev, &algo_input, &algo_results);
    /* Skip 100 stabilization samples for rel_iaq algorithm. */
    if ((lib_ret != REL_IAQ_OK) && (lib_ret != REL_IAQ_STABILIZATION)) {
        Serial.println(F("Error when calculating algorithm, exiting program!"));
    } else {
        Serial.println(F("*********** Measurements ***********"));
        for (int i = 0; i < 13; i++) {
            Serial.print(F(" Rmox["));
            Serial.print(i);
            Serial.print(F("] = "));
            data[i] = algo_results.rmox[i] / 1e3;
            //Serial.print(algo_results.rmox[i] / 1e3);
            Serial.print(data[i]);
            Serial.println(F(" kOhm"));
        }
        Serial.print(F(" Rel IAQ  = "));
        data[13] = algo_results.rel_iaq;
        //Serial.println(algo_results.rel_iaq);
        Serial.println(data[13]);
        data[14] = lib_ret;
        switch (lib_ret) {
        case REL_IAQ_STABILIZATION:
            Serial.println(F("Warm-Up!"));
            break;
        case REL_IAQ_DAMAGE:
            Serial.println(F("Error: Sensor probably damaged. Algorithm results may be incorrect."));
            break;
        case REL_IAQ_OK:
            Serial.println(F("Valid!"));
            break;
        default: /* other errors */
            Serial.println(F("Unexpected Error during algorithm calculation: Exiting Program."));
            error_handle();
        }

    }
    
    // check if the button is pressed
    if (digitalRead(BUTTON_PIN) == LOW) {
        // button is pressed, start or stop saving data
        isSavingData = !isSavingData;
        if (isSavingData) {
        createNewDataFile();
        Serial.println("Data save started.");
        digitalWrite(LED_PIN, HIGH);
        } else {
        Serial.println("Data save stopped.");
        digitalWrite(LED_PIN, LOW);
        dataFile.close();
        }
        delay(500); // debounce delay
    }

    // check if it's time to save data
    if (isSavingData && millis() - lastDataSaveTime >= DATA_SAVE_INTERVAL) {
        lastDataSaveTime = millis();
        // write the data to the file
        if (dataRecordCount >= FILE_MAX_SIZE) {
        // maximum file size reached, create a new file
        createNewDataFile();
        }
        writeDataRecordToFile();
    }
}

void error_handle()
{
    while (1);
}

void createNewDataFile() {
  // close the current file (if open)
  if (!dataFile.isDirectory()) {
    dataFile.close();
  }
  // create a new file with a unique name
  // Find the next available file index
  if (!fileIndex){
      do {
        sprintf(filename, "data_%03d.csv", fileIndex);
        fileIndex++;
    } while (SD.exists(filename));
  }
  else{
    fileIndex++;
    sprintf(filename, "data_%03d.csv", fileIndex);
  }

    // Open the file for writing
    dataFile = SD.open(filename, FILE_WRITE);

  if (!dataFile) {
    Serial.println("Error creating data file!");
  } else {
    // write the CSV header
    dataFile.println("Rmox[0],Rmox[1],Rmox[2],Rmox[3],Rmox[4],Rmox[5],Rmox[6],Rmox[7],Rmox[8],Rmox[9],Rmox[10],Rmox[11],Rmox[12],Rel_IAQ,Status");
    dataRecordCount = 0;
    Serial.print("New data file created: ");
    Serial.println(filename);
  }
}

void writeDataRecordToFile() {

  // write the data to the file
  for (int i = 0; i < DATA_REG_SIZE; i++) {
    dataFile.print(data[i], 2);
    if (i < DATA_REG_SIZE - 1) {
      dataFile.print(",");
    }
  }
  dataFile.println();
  dataRecordCount++;
}