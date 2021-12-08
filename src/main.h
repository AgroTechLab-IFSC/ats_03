#ifndef __MAIN_H__
#define __MAIN_H__

#include <Arduino.h>
#include <IWatchdog.h>
#include <STM32LowPower.h>
#include <STM32RTC.h>
#include "ats_03_setup.h"

/*******************************************************
 *                  GLOBAL VARIABLES
 *******************************************************/
uint32_t now = 0;
uint32_t lastSamplingPeriod = 0;
STM32RTC& rtc = STM32RTC::getInstance();
static byte seconds = 0;
static byte minutes = 0;
static byte hours = 0;
static byte weekDay = 1;
static byte day = 7;
static byte month = 12;
static byte year = 21;
static uint32_t atime = 1;

/*******************************************************
 *                FUNCTIONS PROTOTYPES
 *******************************************************/
#ifdef SERIAL_DEBUG_ENABLED
    void printInitInfo();
#endif
void alarmMatch(void* data);

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
    SERIAL_DEBUG.print(F("\n\tAwaiting 5 seconds for systems stabilization..."));
    SERIAL_DEBUG.flush();    
}
#endif // #ifdef SERIAL_DEBUG_ENABLED

void alarmMatch(void* data) {
    // uint32_t epoc;
    // uint32_t epoc_ms;
    // uint32_t sec = 1;
    // uint32_t _millis = 1000;
    
    // if (data != NULL) {
    //     _millis = *(uint32_t*)data;
    // }
    // sec = _millis / 1000;
    // epoc = rtc.getEpoch(&epoc_ms);
    // rtc.setAlarmEpoch(epoc + sec, STM32RTC::MATCH_SS, epoc_ms);

    // if (digitalRead(LED_BUILTIN) == HIGH) {
    //     digitalWrite(LED_BUILTIN, LOW);
    // } else {
    //     digitalWrite(LED_BUILTIN, HIGH);
    // }
    IWatchdog.reload();
}

#endif // #define __MAIN_H__