#ifndef __MAIN_H__
#define __MAIN_H__

#include <Arduino.h>
#include "ats_03_setup.h"
#include <IWatchdog.h>
#include <STM32RTC.h>
#if (POWER_SUPPLY == BATTERY)
    #include <STM32LowPower.h>
#endif
#ifdef RGB_LED_ENABLED
    #include "RGBLed.h"
#endif
#ifdef SENSOR_DHT_ENABLED
    #include <Adafruit_Sensor.h>
    #include <DHT_U.h>
#endif

/*******************************************************
 *                        STRUCTS
 *******************************************************/
struct station_sensor_t {
    #ifdef SENSOR_DHT_ENABLED
        float airTemp = 0.0f;
        uint8_t airTempCount = 0;
        float airHumid = 0.0f;
        uint8_t airHumidCount = 0;
    #endif
    // #ifdef SENSOR_LIGHT_ENABLED
    //     uint32_t light = 0;
    //     uint8_t lightCount = 0;
    // #endif
    // #ifdef SENSOR_UV_ENABLED
    //     uint32_t uvVoltage = 0;
    //     uint8_t uvVoltageCount = 0;
    // #endif
    // #ifdef SENSOR_SOIL_TEMP_ENABLED
    //     float soilTemp = 0.0f;
    //     uint8_t soilTempCount = 0;
    // #endif
    // #ifdef SENSOR_SOIL_MOISTURE_ENABLED
    //     uint16_t soilMoisture = 0;
    //     uint8_t soilMoistureCount = 0;
    // #endif    
    // #ifdef SENSOR_WIND_SOCK_ENABLED
    //     float windDirVoltage = 0.0f;
    //     uint8_t windDirCount = 0;
    // #endif
    // #ifdef SENSOR_ANEMOMETER_ENABLED
    //     float windSpeed = 0.0f;        
    //     uint8_t windSpeedCount = 0;
    //     uint32_t anemometerTurnAround = 0;
    //     uint32_t lastWindSampling = 0;
    // #endif
    // #ifdef SENSOR_PLUVIOMETER_ENABLED
    //     float rainVolume = 0.0f;        
    //     uint16_t pluviometerTurnAround = 0;        
    // #endif
    // #ifdef SENSOR_POWER_SUPPLY_ENABLED
    //     float powerSupply = 0.0f;
    //     uint8_t powerSupplyCount = 0;
    // #endif
    // #ifdef SENSOR_LEAF_MOISTURE_ENABLED
    //     uint16_t leafMoisture = 0;
    //     uint8_t leafMoistureCount = 0;
    // #endif
    // #ifdef SENSOR_PRESSURE_ENABLED
    //     uint32_t pressure = 0;
    //     uint8_t pressureCount = 0;
    //     float devTemp = 0.0f;
    //     uint8_t devTempCount = 0;
    // #endif
};

/*******************************************************
 *                  GLOBAL VARIABLES
 *******************************************************/
station_sensor_t sensorsData;
volatile uint8_t wakeupTimes = 0;
volatile uint8_t samplingTimes = 0;
STM32RTC& rtc = STM32RTC::getInstance();
static byte seconds = 0;
static byte minutes = 0;
static byte hours = 0;
static byte weekDay = 1;
static byte day = 1;
static byte month = 1;
static byte year = 21;
static uint32_t atime = 1;
#ifdef RGB_LED_ENABLED
    RGBLed rgb_led(LED_RGB_TYPE, LED_RGB_RED_PIN, LED_RGB_GREEN_PIN, LED_RGB_BLUE_PIN);  /**< Global variable to access RGB LED device. */
#endif
#ifdef SENSOR_DHT_ENABLED
    DHT_Unified dht(DHT_PIN, DHT_TYPE);                         /**< Global variable to access DHT sensor. */
#endif

/*******************************************************
 *                FUNCTIONS PROTOTYPES
 *******************************************************/
#ifdef SERIAL_DEBUG_ENABLED
    void printInitInfo();
    void printAverageValues();
#endif
void SystemClock_Config(void);
#if (POWER_SUPPLY == POWER_LINE)
    void wakeUpPowerLine(void* data);
#endif
#if (POWER_SUPPLY == BATTERY)
    void wakeUpBattery(void* data);
    void goSleep();
#endif
#ifdef SENSOR_DHT_ENABLED
    uint8_t initSensorDHT();
    uint8_t getDHTTemperature();
    uint8_t getDHTHumidity();
#endif

/*******************************************************
 *             FUNCTIONS IMPLEMENTATIONS
 *******************************************************/

#ifdef SERIAL_DEBUG_ENABLED
/**
 * @fn printInitInfo
 * @brief Print initial information.
 */
void printInitInfo() {
    SERIAL_DEBUG.print(F("\n#############################################\n"));
    SERIAL_DEBUG.print(F("#                   ATS-03                  #\n"));
    SERIAL_DEBUG.print(F("#            AgroTechStation - 03           #\n"));
    SERIAL_DEBUG.print(F("#    Instituto Federal de Santa Catarina    #\n"));
    SERIAL_DEBUG.print(F("#                Campus Lages               #\n"));
    SERIAL_DEBUG.print(F("#         developed by AgroTechLab          #\n"));
    SERIAL_DEBUG.print(F("#############################################\n"));
    SERIAL_DEBUG.print(F("Starting ATS-03..."));
    SERIAL_DEBUG.print(F("\n\tPower supply..............: "));
    SERIAL_DEBUG.print(power_supply_str[POWER_SUPPLY]);
    SERIAL_DEBUG.print(F("\n\tMCU device board..........: "));
    SERIAL_DEBUG.print(MCU_BOARD);
    SERIAL_DEBUG.print(F("\n\tCPU clock frenquency......: "));
    SERIAL_DEBUG.print(F_CPU);
    SERIAL_DEBUG.print(F("\n\tCommunication interface...: "));
    SERIAL_DEBUG.print(COMM_IF);
    SERIAL_DEBUG.print(F("\n\tFirmware version..........: "));
    SERIAL_DEBUG.print(FW_VERSION);
    SERIAL_DEBUG.print(F("\n\tHardware version..........: "));
    SERIAL_DEBUG.print(HW_VERSION);
    SERIAL_DEBUG.print(F("\n\tSensor list...............: "));
    SERIAL_DEBUG.print(DEV_SENSOR_LIST);
    SERIAL_DEBUG.print(F("\n\tActuator list.............: "));
    SERIAL_DEBUG.print(DEV_ACTUATOR_LIST);    
    SERIAL_DEBUG.flush();    
}

void printAverageValues() {
    SERIAL_DEBUG.print(F("\n\tAverage air temperature (in oC): ")); SERIAL_DEBUG.print(sensorsData.airTemp/sensorsData.airTempCount);
    SERIAL_DEBUG.print(F(" (")); SERIAL_DEBUG.print(sensorsData.airTempCount); SERIAL_DEBUG.print(F(" sampling)"));
    SERIAL_DEBUG.print(F("\n\tAverage air humidity (in %): ")); SERIAL_DEBUG.print(sensorsData.airHumid/sensorsData.airHumidCount);
    SERIAL_DEBUG.print(F(" (")); SERIAL_DEBUG.print(sensorsData.airHumidCount); SERIAL_DEBUG.print(F(" sampling)"));
    // SERIAL_DEBUG.print(F("\nAverage light (in lux): ")); SERIAL_DEBUG.print(sensorsData.light/sensorsData.lightCount);
    // SERIAL_DEBUG.print(F(" (")); SERIAL_DEBUG.print(sensorsData.lightCount); SERIAL_DEBUG.print(F(" sampling)"));
    // SERIAL_DEBUG.print(F("\nAverage UV tension (in milliVolts): ")); SERIAL_DEBUG.print(sensorsData.uvVoltage/sensorsData.uvVoltageCount);
    // SERIAL_DEBUG.print(F(" => Index: ")); SERIAL_DEBUG.print(convertMilliVoltsToIndex(sensorsData.uvVoltage/sensorsData.uvVoltageCount));
    // SERIAL_DEBUG.print(F(" (")); SERIAL_DEBUG.print(sensorsData.uvVoltageCount); SERIAL_DEBUG.print(F(" sampling)"));    
    // SERIAL_DEBUG.print(F("\nAverage soil temperature (in oC): ")); SERIAL_DEBUG.print(sensorsData.soilTemp/sensorsData.soilTempCount);
    // SERIAL_DEBUG.print(F(" (")); SERIAL_DEBUG.print(sensorsData.soilTempCount); SERIAL_DEBUG.print(F(" sampling)")); 
    // SERIAL_DEBUG.print(F("\nAverage soil moisture (in %): ")); SERIAL_DEBUG.print(sensorsData.soilMoisture/sensorsData.soilMoistureCount);
    // SERIAL_DEBUG.print(F(" (")); SERIAL_DEBUG.print(sensorsData.soilMoistureCount); SERIAL_DEBUG.print(F(" sampling)"));   
    // SERIAL_DEBUG.print(F("\nAverage leaf moisture (in %): ")); SERIAL_DEBUG.print(sensorsData.leafMoisture/sensorsData.leafMoistureCount);
    // SERIAL_DEBUG.print(F(" (")); SERIAL_DEBUG.print(sensorsData.leafMoistureCount); SERIAL_DEBUG.print(F(" sampling)"));   
    // SERIAL_DEBUG.print(F("\nAverage wind direction (in Volts): ")); SERIAL_DEBUG.print(sensorsData.windDirVoltage/sensorsData.windDirCount);
    // SERIAL_DEBUG.print(F(" => Direction (in degrees): ")); SERIAL_DEBUG.print(convertVoltsToWindDirection(sensorsData.windDirVoltage/sensorsData.windDirCount));
    // SERIAL_DEBUG.print(F(" (")); SERIAL_DEBUG.print(sensorsData.windDirCount); SERIAL_DEBUG.print(F(" sampling)"));   
    // SERIAL_DEBUG.print(F("\nAverage wind speed (in Km/h): ")); SERIAL_DEBUG.print(sensorsData.windSpeed/sensorsData.windSpeedCount);
    // SERIAL_DEBUG.print(F(" (")); SERIAL_DEBUG.print(sensorsData.windSpeedCount); SERIAL_DEBUG.print(F(" sampling)")); 
    // SERIAL_DEBUG.print(F("\nAverage rain volume (in ml): ")); SERIAL_DEBUG.print(sensorsData.rainVolume);    
    // SERIAL_DEBUG.print(F("\nAverage power supply (in Volts): ")); SERIAL_DEBUG.print(sensorsData.powerSupply/sensorsData.powerSupplyCount);
    // SERIAL_DEBUG.print(F(" (")); SERIAL_DEBUG.print(sensorsData.powerSupplyCount); SERIAL_DEBUG.print(F(" sampling)"));    
    // SERIAL_DEBUG.print(F("\nAverage pressure (in hPa): ")); SERIAL_DEBUG.print(sensorsData.pressure/sensorsData.pressureCount);
    // SERIAL_DEBUG.print(F(" (")); SERIAL_DEBUG.print(sensorsData.pressureCount); SERIAL_DEBUG.print(F(" sampling)"));    
    // SERIAL_DEBUG.print(F("\nAverage device temperature (in oC): ")); SERIAL_DEBUG.print(sensorsData.devTemp/sensorsData.devTempCount);
    // SERIAL_DEBUG.print(F(" (")); SERIAL_DEBUG.print(sensorsData.devTempCount); SERIAL_DEBUG.print(F(" sampling)"));            
    SERIAL_DEBUG.flush();
}
#endif // #ifdef SERIAL_DEBUG_ENABLED

#if (POWER_SUPPLY == BATTERY)
void wakeUpBattery(void* data) {        
    
    // Disable interrupts
    noInterrupts();    

    // Reset WDT
    IWatchdog.reload();

    // Increment wake up times
    wakeupTimes++;

    // Enable interrupts
    interrupts();
}

/**
 * @brief Put the station in sleep mode 
 */
void goSleep() {

}
#endif

#if (POWER_SUPPLY == POWER_LINE)
void wakeUpPowerLine(void* data) {        
    
    // Disable interrupts
    noInterrupts();    

    // Reset WDT
    IWatchdog.reload();    

    // Increment wake up times
    wakeupTimes++;

    // Sets next wake up date/time    
    rtc.setAlarmEpoch(rtc.getEpoch() + (wd_timeout/1000) + rtc_offset);

    // Enable interrupts
    interrupts();
}
#endif

void SystemClock_Config(void) {

    // Sets CPU clock config to power line based power supply
    if (POWER_SUPPLY == POWER_LINE) {
        
        RCC_OscInitTypeDef RCC_OscInitStruct = {0};
        RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
        RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

        /** 
         * Initializes the RCC Oscillators according to the specified parameters
         * in the RCC_OscInitTypeDef structure.
         */
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
        RCC_OscInitStruct.HSEState = RCC_HSE_ON;
        RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
        RCC_OscInitStruct.HSIState = RCC_HSI_ON;
        RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
        RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
        if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
            Error_Handler();
        }

        /** 
         * Initializes the CPU, AHB and APB buses clocks
         */
        RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                    |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
        RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
        RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
        RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
        RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
        if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
            Error_Handler();
        }

        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
        PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
            Error_Handler();
        }
    }

    // Sets CPU clock config to battery based power supply
    if (POWER_SUPPLY == BATTERY) {
        
        RCC_OscInitTypeDef RCC_OscInitStruct = {0};
        RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

        /** 
         * Initializes the RCC Oscillators according to the specified parameters
         * in the RCC_OscInitTypeDef structure.
         */
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
        RCC_OscInitStruct.HSIState = RCC_HSI_ON;
        RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
        if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
            Error_Handler();
        }

        /** 
         * Initializes the CPU, AHB and APB buses clocks
         */
        RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
        RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
        RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
        RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
        RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
        if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
            Error_Handler();
        }
    }
}

#ifdef SENSOR_DHT_ENABLED
uint8_t initSensorDHT() {    
    #ifdef SERIAL_DEBUG_ENABLED
        SERIAL_DEBUG.print(F("\n\tInitiating DHT sensor... "));
        SERIAL_DEBUG.flush();
    #endif
    pinMode(DHT_PIN, INPUT);
    dht.begin();
    sensor_t dht_sensor;
    dht.temperature().getSensor(&dht_sensor);
    if (strcmp(dht_sensor.name, "DHT22") == 0) {
        #ifdef SERIAL_DEBUG_ENABLED
            SERIAL_DEBUG.print(F("[OK]"));
            SERIAL_DEBUG.print(F("\n\t\tSensor type...: ")); SERIAL_DEBUG.print(dht_sensor.name);
            SERIAL_DEBUG.print(F("\n\t\tDriver version: ")); SERIAL_DEBUG.print(dht_sensor.version);
            SERIAL_DEBUG.print(F("\n\t\tUnique ID.....: ")); SERIAL_DEBUG.print(dht_sensor.sensor_id);
            SERIAL_DEBUG.print(F("\n\t\tMinimum delay.: ")); SERIAL_DEBUG.print(dht_sensor.min_delay/1000); SERIAL_DEBUG.print(F(" ms"));
            SERIAL_DEBUG.print(F("\n\t\tAir Temperature "));
            SERIAL_DEBUG.print(F("\n\t\t\tMinimum value...: ")); SERIAL_DEBUG.print(dht_sensor.min_value); SERIAL_DEBUG.print(F(" oC"));
            SERIAL_DEBUG.print(F("\n\t\t\tMaximum value...: ")); SERIAL_DEBUG.print(dht_sensor.max_value); SERIAL_DEBUG.print(F(" oC"));
            SERIAL_DEBUG.print(F("\n\t\t\tResolution......:   ")); SERIAL_DEBUG.print(dht_sensor.resolution); SERIAL_DEBUG.print(F(" oC"));
            SERIAL_DEBUG.flush();
        #endif
        dht.humidity().getSensor(&dht_sensor);
        #ifdef SERIAL_DEBUG_ENABLED
            SERIAL_DEBUG.print(F("\n\t\tAir Humidity "));
            SERIAL_DEBUG.print(F("\n\t\t\tMinimum value...:   ")); SERIAL_DEBUG.print(dht_sensor.min_value); SERIAL_DEBUG.print(F(" %"));
            SERIAL_DEBUG.print(F("\n\t\t\tMaximum value...: ")); SERIAL_DEBUG.print(dht_sensor.max_value); SERIAL_DEBUG.print(F(" %"));
            SERIAL_DEBUG.print(F("\n\t\t\tResolution......:   ")); SERIAL_DEBUG.print(dht_sensor.resolution); SERIAL_DEBUG.print(F(" %"));
            SERIAL_DEBUG.flush();
        #endif
        return 0;
    } else {
        #ifdef SERIAL_DEBUG_ENABLED
            SERIAL_DEBUG.print(F("[FAIL]"));
            SERIAL_DEBUG.flush();
        #endif
        return 1;        
    }    
}

uint8_t getDHTTemperature() {
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature) || (event.temperature < -40) || (event.temperature > 80)) {
        #ifdef SERIAL_DEBUG_ENABLED
            SERIAL_DEBUG.print(F("\n\tError reading air temperature!"));
            SERIAL_DEBUG.flush();
        #endif
        return 1;
    }
    else {
        sensorsData.airTemp += event.temperature;
        sensorsData.airTempCount++;
        #ifdef SERIAL_DEBUG_ENABLED
            SERIAL_DEBUG.print(F("\n\tAir temperature (in oC): "));
            SERIAL_DEBUG.print(event.temperature);           
            SERIAL_DEBUG.flush();
        #endif
        return 0;
    }
}

uint8_t getDHTHumidity() {
    sensors_event_t event;
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity) || (event.relative_humidity < 0) || (event.relative_humidity > 100)) {
        #ifdef SERIAL_DEBUG_ENABLED
            SERIAL_DEBUG.print(F("\n\tError reading air humidity!"));
            SERIAL_DEBUG.flush();
        #endif
        return 1;
    }
    else {
        sensorsData.airHumid += event.relative_humidity;
        sensorsData.airHumidCount++;
        #ifdef SERIAL_DEBUG_ENABLED
            SERIAL_DEBUG.print(F("\n\tAir humidity (in %): "));
            SERIAL_DEBUG.print(event.relative_humidity);            
            SERIAL_DEBUG.flush();
        #endif
        return 0;
    }
}
#endif

#endif // #define __MAIN_H__