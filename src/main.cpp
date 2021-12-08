#include "main.h"

void setup() {

  // Start watchdog using window mode with maximum time (aprox. 22 sec.)
  IWatchdog.begin(IWDG_TIMEOUT_MAX, IWDG_TIMEOUT_MAX - 1000);

  // if (POWER_SUPPLY == POWER_LINE) {
    // Start LED builtin and blink it 3 times
    pinMode(LED_BUILTIN, OUTPUT);
    for (int i = 0; i <= 6; i++) {
      if (digitalRead(LED_BUILTIN) == HIGH) {
        digitalWrite(LED_BUILTIN, LOW);
      } else {
        digitalWrite(LED_BUILTIN, HIGH);
      }
      delay(500);
    }    

  // } else if (POWER_SUPPLY == BATTERY) {
    // Start RTC time/date
    rtc.begin();
    rtc.setTime(hours, minutes, seconds);
    rtc.setDate(weekDay, day, month, year);

    // Start Low Power controller and allow wake up from RTC interrupt
    LowPower.begin();  
    LowPower.enableWakeupFrom(&rtc, alarmMatch, &atime);        
  // }

  

  // Check if serial debug is enabled
  // #ifdef SERIAL_DEBUG_ENABLED        
    SERIAL_DEBUG.begin(SERIAL_BAUDRATE);    
    printInitInfo();    
  // #endif
    
  // Reset WDT
  IWatchdog.reload();
}

void loop() {  

  // LowPower.deepSleep(1000);
  // // Get time
  // now = millis();

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
    rtc.setAlarmEpoch(rtc.getEpoch() + 20);
    LowPower.deepSleep();    
    
    
    // while (true) {}    
  // }  
}