#ifndef __ATS_03_SETUP_H__
#define __ATS_03_SETUP_H__

#include <Arduino.h>

enum power_supply_e {BATTERY, POWER_LINE};
const char* power_supply_str[] = {"Battery", "Power Line"};

/**
 * \def POWER_SUPPLY 
 * Define the power supply of AgroTechStation (BATTERY/POWER_LINE) 
 */
#define POWER_SUPPLY            POWER_LINE

/**
 * \def SERIAL_DEBUG_ENABLED 
 * Enable or disable the serial debug.
 */
#define SERIAL_DEBUG_ENABLED

/**
 * \def RGB_LED_ENABLED 
 * Enable or disable RGB led.
 */
#define RGB_LED_ENABLED

/**
 * \def SENSOR_DHT_ENABLED 
 * Enable or disable the DHT sensor.
 */
#define SENSOR_DHT_ENABLED

/**
 * \def I2C1_ENABLED 
 * Enable or disable I2C 1 interface.
 */
#define I2C1_ENABLED

#ifdef I2C1_ENABLED
    /**
    * \def SENSOR_LIGHT_ENABLED 
    * Enable or disable the LIGHT sensor.
    */
    #define SENSOR_LIGHT_ENABLED

    /**
    * \def SENSOR_POWER_SUPPLY_ENABLED 
    * Enable or disable the power supply sensor.
    */
    #define SENSOR_POWER_SUPPLY_ENABLED

    /**
    * \def SENSOR_PRESSURE_ENABLED 
    * Enable or disable the pressure sensor.
    */
    #define SENSOR_PRESSURE_ENABLED
#endif

/**
 * \def SENSOR_LEAF_MOISTURE_ENABLED 
 * Enable or disable the leaf moisture sensor.
 */
#define SENSOR_LEAF_MOISTURE_ENABLED

/**
 * \def SENSOR_UV_ENABLED 
 * Enable or disable UV sensor.
 */
#define SENSOR_UV_ENABLED

/**
 * \def ONE_WIRE_ENABLED 
 * Enable or disable one wire interface.
 */
#define ONE_WIRE_ENABLED

#ifdef ONE_WIRE_ENABLED
    /**
    * \def SENSOR_SOIL_TEMP_ENABLED 
    * Enable or disable soil temperature sensor.
    */
    #define SENSOR_SOIL_TEMP_ENABLED    
#endif

/**
 * \def SENSOR_SOIL_MOISTURE_ENABLED 
 * Enable or disable soil moisture sensor.
 */
#define SENSOR_SOIL_MOISTURE_ENABLED

/**
 * \def SENSOR_WIND_SOCK_ENABLED 
 * Enable or disable wind sock sensor.
 */
#define SENSOR_WIND_SOCK_ENABLED

/**
 * \def SENSOR_ANEMOMETER_ENABLED 
 * Enable or disable anemometer sensor.
 */
#define SENSOR_ANEMOMETER_ENABLED

/**
 * \def SENSOR_PLUVIOMETER_ENABLED 
 * Enable or disable pluviometer sensor.
 */
#define SENSOR_PLUVIOMETER_ENABLED

/*******************************************************
 *                 DEVICE PARAMETERS
 *******************************************************/
/**
 * \def MCU_BOARD 
 * MCU device board.
 */
#define MCU_BOARD               "STM32F103C8T6"

/**
 * \def COMM_IF 
 * Communication interface.
 */
#define COMM_IF                 "LoRaWAN - RHF76-052"

/**
 * \def FW_VERSION 
 * Firmware version.
 */
#define FW_VERSION              "0.1.0"

/**
 * \def HW_VERSION 
 * Hardware version.
 */
#define HW_VERSION              "0.1.0"

/**
 * \def DEV_SENSOR_LIST 
 * List of sensors enabled in the station.
 */
#define DEV_SENSOR_LIST         "INA-219 | BMP-085 | DHT22 | GY-30 | UVM-30A | DS18B20 | HD-38 | WINDSOCK | ANEMOMETER | PLUVIOMETER"

/**
 * \def DEV_ACTUATOR_LIST 
 * List of actuators enabled in the station.
 */
#define DEV_ACTUATOR_LIST       "none"


#ifdef SERIAL_DEBUG_ENABLED
    /**
    * \def SERIAL_BAUDRATE 
    * Define the serial baudrate.
    */
    #define SERIAL_BAUDRATE     115200
#endif

#ifdef RGB_LED_ENABLED
    /**
    * \def LED_RGB_TYPE 
    * RGB LED type (anode/cathode).
    */
    #define LED_RGB_TYPE                ANODE
#endif

#ifdef SERIAL_DEBUG_ENABLED    
    /**
    * \def SERIAL_DEBUG 
    * Serial used to DEBUG terminal
    */    
    #define SERIAL_DEBUG    SerialUSB
#endif

/**
 * \def SERIAL_LORA 
 * Serial used to LoRaWAN
 */
#define SERIAL_LORA     Serial2

/*******************************************************
 *                   SYSTEM PINOUT
 *******************************************************/
 /**
  * \def LED_BUILTIN_PIN 
  * LED builtin pin.
  */
#define LED_BUILTIN_PIN                 13

#ifdef RGB_LED_ENABLED
    /**
    * \def LED_RGB_RED_PIN
    * Red RGB LED device pin.
    */
    #define LED_RGB_RED_PIN             8

    /**
    * \def LED_RGB_GREEN_PIN 
    * Green RGB LED device pin.
    */
    #define LED_RGB_GREEN_PIN           9

    /**
    * \def LED_RGB_BLUE_PIN
    * Blue RGB LED device pin.
    */
    #define LED_RGB_BLUE_PIN            10
#endif

#ifdef SENSOR_DHT_ENABLED
    /**
    * \def DHT_PIN 
    * DHT sensor pin.
    */
    #define DHT_PIN                     5

    /**
    * \def DHT_TYPE 
    * DHT sensor type.
    */
    #define DHT_TYPE                    DHT22
#endif

#ifdef I2C1_ENABLED
    /**
    * \def I2C1_SDA_PIN 
    * SDA pin of I2C 1 interface.
    */
    #define I2C1_SDA_PIN                20

    /**
    * \def I2C1_SCL_PIN 
    * SCL pin of I2C 1 interface.
    */
    #define I2C1_SCL_PIN                21
#endif

#ifdef SENSOR_UV_ENABLED
    /**
    * \def SENSOR_UV_PIN 
    * UV sensor pin.
    */
    #define SENSOR_UV_PIN               A4
#endif

#ifdef ONE_WIRE_ENABLED
    /**
    * \def ONE_WIRE_PIN 
    * One Wire interface pin.
    */
    #define ONE_WIRE_PIN                4
#endif

#ifdef SENSOR_SOIL_MOISTURE_ENABLED
    /**
    * \def SENSOR_SOIL_MOISTURE_PIN 
    * Soil moisture sensor pin.
    */
    #define SENSOR_SOIL_MOISTURE_PIN    A2
#endif

#ifdef SENSOR_LEAF_MOISTURE_ENABLED
    /**
    * \def SENSOR_LEAF_MOISTURE_PIN 
    * Leaf moisture sensor pin.
    */
    #define SENSOR_LEAF_MOISTURE_PIN    A7
#endif

#ifdef SENSOR_WIND_SOCK_ENABLED
    /**
    * \def WIND_SOCK_PIN 
    * Wind sock sensor pin.
    */
    #define WIND_SOCK_PIN               A0
#endif

#ifdef SENSOR_ANEMOMETER_ENABLED
    /**
    * \def ANEMOMETER_PIN 
    * Anemometer sensor pin.
    */
    #define ANEMOMETER_PIN              2
#endif

#ifdef SENSOR_PLUVIOMETER_ENABLED
    /**
    * \def PLUVIOMETER_PIN 
    * Pluviometer sensor pin.
    */
    #define PLUVIOMETER_PIN             3
#endif

/*******************************************************
 *                  SYSTEM PARAMETERS
 *******************************************************/
const unsigned long systemPeriod = 1000;                        /**< System run period (in ms). */
const unsigned long samplingPeriod = 1 * systemPeriod;          /**< Sampling period (in ms). */
const unsigned long txPeriod = 5 * samplingPeriod;              /**< Transmission period (in ms). */
const float pi = 3.1415926;                                     /**< PI used in anemometer computation. */
const float anemometer_radius = 0.147;                          /**< Anemometer radius (in m). */
const unsigned long error_reset_period = 60 * systemPeriod;      /**< Error reset period (in ms). */

/*********************************************
 *             TTN PARAMETERS
 ********************************************/
const char* dev_eui = "70B3D57ED0047B03";                       /**< Device EUI for TTN network. */
const char* app_eui = "0000000000000000";                       /**< Application EUI for TTN network. */
const char* app_key = "103CF98370483258CD3B0202ACCD26D8";       /**< Application Key for TTN network. */
const char* apps_key = "7A01D13AA6B829961AB8792F3C09543C";      /**< Application Session Key for TTN network. */
const char* nwks_key = "4C9264F78B746D9FF205C41CD65E46E5";      /**< Network Session Key for TTN network. */ 
const char* dev_addr = "260DE8CD"; 

#endif // #define __ATS_03_SETUP_H__