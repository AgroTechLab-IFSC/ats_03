#include "main.h"

void setup() {

  // Start watchdog using window mode with maximum time (aprox. 22 sec.)
  // IMPORTANT: WDT will be used only in running mode, on sleep mode it will be disabled
  IWatchdog.begin(IWDG_TIMEOUT_MAX, IWDG_TIMEOUT_MAX - 1000);  

  // Start RTC and update time/date to (30/03/2009 @ 21h10min)
  rtc.begin();
  rtc.setTime(hours, minutes, seconds);
  rtc.setDate(weekDay, day, month, year);

  // Start Low Power controller and allow wake up from RTC interrupt
  LowPower.begin();  
  LowPower.enableWakeupFrom(&rtc, alarmMatch, &atime);

  // If power supply is power line, start leds builtin and RGB
  // if (POWER_SUPPLY == POWER_LINE) {
    pinMode(LED_BUILTIN, OUTPUT);
    for (int i = 0; i <= 6; i++) {
      if (digitalRead(LED_BUILTIN) == HIGH) {
        digitalWrite(LED_BUILTIN, LOW);
      } else {
        digitalWrite(LED_BUILTIN, HIGH);
      }
      delay(250);
    }    
  // }

  // Check if serial debug is enabled
  #ifdef SERIAL_DEBUG_ENABLED        
    SERIAL_DEBUG.begin(SERIAL_BAUDRATE);    
    printInitInfo();    
  #endif
    
  // Reset WDT
  IWatchdog.reload();

  // Set the 1st wake up to 15 seconds after and put station in sleep mode
  rtc.setAlarmEpoch(rtc.getEpoch() + 15);
  LowPower.deepSleep();
}

void loop() {  
  
  // Reset WDT
  IWatchdog.reload();

  // Get the wake up instant date/time
  now = rtc.getEpoch();

  // // Check power supply mode
  // if (POWER_SUPPLY == POWER_LINE) {

  //   // Toogle LED builtin
  //   if (digitalRead(LED_BUILTIN) == HIGH) {
  //     digitalWrite(LED_BUILTIN, LOW);
  //   } else {
  //     digitalWrite(LED_BUILTIN, HIGH);
  //   }
  //   delay(1000);

  // } else if (POWER_SUPPLY == BATTERY) {        

  //   // Check sampling period
  //   if (now - lastSamplingPeriod >= samplingPeriod) {

  //     // Update last sampling period
  //     lastSamplingPeriod = now;

  //     // Reset WDT
  //     // IWatchdog.reload();

  //     SerialUSB.printf("\n%d - Running in battery mode (5 seconds)...", millis());
  //     SerialUSB.flush();
  //     delay(5000);
  //     // IWatchdog.reload();      

  //     SerialUSB.printf("\n%d - sleeping 5 seconds...", millis());            
  //     SerialUSB.flush();
  //     LowPower.sleep(5000);

  //     SerialUSB.printf("\nWaking up... ");
  //     SerialUSB.flush();
  //     // IWatchdog.reload();
  //   }    
  // } else {
    digitalWrite(LED_BUILTIN, LOW);
    delay(3000);
    digitalWrite(LED_BUILTIN, HIGH);
    IWatchdog.reload();
    rtc.setAlarmEpoch(rtc.getEpoch() + 30);
    LowPower.deepSleep();    
    
    
    // while (true) {}    
  // }  
}