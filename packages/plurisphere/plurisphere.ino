#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#if SOFTWARE_SERIAL_AVAILABLE
    #include <SoftwareSerial.h>
#endif

#include "./bluefruit/BluefruitConfig.h"
#include "./plurisphere/Plurisphere.h"



/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE 0
#define MINIMUM_FIRMWARE_VERSION "0.6.6"

#define PLURISPHERE_DEVICE_ID "v2qfjddfb75oau1qgxwil3bvbr9xtcne"
/*=========================================================================*/



// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);



/* Set the delay between fresh samples (not too fast, BLE UART is slow!) */
/* Firware <=0.6.6 should use 500ms, >=0.6.7 can use 200ms */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);



// A small helper
void error(const __FlashStringHelper *err)
{
    Serial.println(err);
    while (1)
        ;
}



/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       ");
    Serial.println(sensor.name);
    Serial.print("Driver Ver:   ");
    Serial.println(sensor.version);
    Serial.print("Unique ID:    ");
    Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    ");
    Serial.print(sensor.max_value);
    Serial.println(" xxx");
    Serial.print("Min Value:    ");
    Serial.print(sensor.min_value);
    Serial.println(" xxx");
    Serial.print("Resolution:   ");
    Serial.print(sensor.resolution);
    Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
    // Get the system status values (mostly for debugging purposes)
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    // Display the results in the Serial Monitor
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
    // Get the four calibration values (0..3)
    // 3 means 'fully calibrated"
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    // The data should be ignored until the system calibration is > 0
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    // Display the individual values
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}

/**************************************************************************/
/*
    Sends sensor calibration status out over BLE UART
*/
/**************************************************************************/
void transmitCalStatus(void)
{
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;

    // Get the four calibration values (0..3)
    // 3 means 'fully calibrated"
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* Prepare the AT command */
    ble.print("AT+BLEUARTTX=");

    // Transmit individual values
    // Note: The values are abbreviated compared to the Serial Monitor
    // to save space since BLE UART is quite slow */
    ble.print(",");
    ble.print(system, DEC);
    ble.print(gyro, DEC);
    ble.print(accel, DEC);
    ble.println(mag, DEC);

    if (!ble.waitForOK())
    {
        Serial.println(F("Failed to send?"));
    }
}

/**************************************************************************/
/*!
    @brief  Initializes the one wire temperature sensor
*/
/**************************************************************************/
void initSensor(void)
{
    if (!bno.begin())
    {
        // There was a problem detecting the BNO055 ... check your connections
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1)
            ;
    }

    delay(1000);

    // Display some basic information on this sensor
    displaySensorDetails();

    // Optional: Display current status
    displaySensorStatus();

    bno.setExtCrystalUse(true);
}



/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
    char device_name_buffer[58];
    const char *device_name = "AT+GAPDEVNAME=plurisphere-";
    const char *device_id = PLURISPHERE_DEVICE_ID;
    strcpy(device_name_buffer, device_name);
    strcat(device_name_buffer, device_id);


    while (!Serial);  // required for Flora & Micro
    delay(500);

    Serial.begin(115200);
    Serial.println(F("Plurisphere"));
    Serial.println(F("---------------------------------------"));

    /* Initialise the module */
    Serial.print(F("Initialising the Bluefruit LE module: "));

    if (!ble.begin(VERBOSE_MODE))
    {
        error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
    }
    Serial.println(F("OK!"));


    Serial.println(F("Setting device name to 'plurisphere-<id>': "));
    if (!ble.sendCommandCheckOK(device_name_buffer))
    {
        error(F("Could not set device name?"));
    }

    //   if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=plurisphere-xyz" )) ) {
    //    error(F("Could not set device name?"));
    //  }


    if (FACTORYRESET_ENABLE)
    {
        /* Perform a factory reset to make sure everything is in a known state */
        Serial.println(F("Performing a factory reset: "));
        if (!ble.factoryReset())
        {
            error(F("Couldn't factory reset"));
        }
    }


    /* Disable command echo from Bluefruit */
    ble.echo(false);


    Serial.println("Requesting Bluefruit info:");
    /* Print Bluefruit information */
    ble.info();


    // This demo only available for firmware from 0.6.6
    if (!ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION))
    {
        error(F("This sketch requires firmware version " MINIMUM_FIRMWARE_VERSION " or higher!"));
    }


    /* Enable HID Service (including Mouse) */
    Serial.println(F("Enable HID Service (including Mouse): "));
    if (!ble.sendCommandCheckOK(F("AT+BleHIDEn=On")))
    {
        error(F("Failed to enable HID (firmware >=0.6.6?)"));
    }


    /* Add or remove service requires a reset */
    Serial.println(F("Performing a SW reset (service changes require a reset): "));
    if (!ble.reset())
    {
        error(F("Could not reset??"));
    }

    Serial.println();
}



/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
    // Check for user input
    char inputs[BUFSIZE+1];

    // Send sensor data out
    if (ble.isConnected())
    {
        // Get Quaternion data (no 'Gimbal Lock' like with Euler angles)
        imu::Quaternion quat = bno.getQuat();

        // Display the full data in Serial Monitor
        Serial.print("qW: ");
        Serial.print(quat.w(), 4);
        Serial.print(" qX: ");
        Serial.print(quat.y(), 4);
        Serial.print(" qY: ");
        Serial.print(quat.x(), 4);
        Serial.print(" qZ: ");
        Serial.print(quat.z(), 4);
        displayCalStatus();
        Serial.println("");

        // Send abbreviated integer data out over BLE UART
        ble.print("AT+BLEUARTTX=");
        ble.print(quat.w(), 4);
        ble.print(",");
        ble.print(quat.y(), 4);
        ble.print(",");
        ble.print(quat.x(), 4);
        ble.print(",");
        ble.println(quat.z(), 4);
        if (! ble.waitForOK() )
        {
            Serial.println(F("Failed to send?"));
        }

        // Optional: Send the calibration data as well
        transmitCalStatus();

        // Send a new line character for the next record
        ble.println("AT+BLEUARTTX=\\r\\n");
        if (! ble.waitForOK() )
        {
            Serial.println(F("Failed to send?"));
        }

        /*
        // Display the buffer size (firmware 0.6.7 and higher only!)
        ble.println("AT+BLEUARTFIFO=TX");
        ble.readline();
        Serial.print("TX FIFO: ");
        Serial.println(ble.buffer);
        */

        // Wait a bit ...
        delay(BNO055_SAMPLERATE_DELAY_MS);
    }

    // Check for incoming characters from Bluefruit
    if (ble.isConnected())
    {
        ble.println("AT+BLEUARTRX");
        ble.readline();
        if (strcmp(ble.buffer, "OK") == 0) {
            // no data
            return;
        }
        // Some data was found, its in the buffer
        Serial.print(F("[Recv] ")); Serial.println(ble.buffer);
        ble.waitForOK();
    }
}
