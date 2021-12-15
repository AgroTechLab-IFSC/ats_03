#include "main.h"

void setup() {
  uint8_t setupStatus;

  // Sets CPU clock config according to POWER SUPPLY
  SystemClock_Config();  

  // Starts watchdog timer using window mode with maximum time (aprox. 22 sec.)  
  IWatchdog.begin(IWDG_TIMEOUT_MAX, IWDG_TIMEOUT_MAX - 1000);

  // Blink builtin and RGB led
  if (POWER_SUPPLY == POWER_LINE) {
    pinMode(LED_BUILTIN_PIN, OUTPUT);
    #ifdef RGB_LED_ENABLED
      rgb_led.setup();
    #endif

    // Blinks builtin and RGB leds
    digitalWrite(LED_BUILTIN, HIGH);
    #ifdef RGB_LED_ENABLED    
      rgb_led.off();
    #endif    
    for (int i = 0; i <= 10; i++) {
      if (digitalRead(LED_BUILTIN) == HIGH) {
        digitalWrite(LED_BUILTIN_PIN, LOW);
        #ifdef RGB_LED_ENABLED        
          rgb_led.on(Color(200,20,255));
        #endif
      } else {
        digitalWrite(LED_BUILTIN, HIGH);
        #ifdef RGB_LED_ENABLED
          rgb_led.off();
        #endif
      }
      delay(200);
    }    
  }
  
  // Checks if serial debug is enabled
  #ifdef SERIAL_DEBUG_ENABLED                
    printInitInfo();    
  #endif

  // Starts RTC and updates time/date to (01/01/2021 @ 00h00min)
  #ifdef SERIAL_DEBUG_ENABLED                
    SERIAL_DEBUG.print(F("\nStarting RTC..."));
    SERIAL_DEBUG.flush();
  #endif
  rtc.begin();
  rtc.setTime(hours, minutes, seconds);
  rtc.setDate(weekDay, day, month, year);

  // Attach wakeup interrupt based on power supply
  if (POWER_SUPPLY == POWER_LINE) {    
    #ifdef SERIAL_DEBUG_ENABLED                
      SERIAL_DEBUG.print(F("\n\tCreating wake up interrupt..."));
      SERIAL_DEBUG.flush();
    #endif
    rtc.attachInterrupt(wakeUpPowerLine);    
  } else if (POWER_SUPPLY == BATTERY) {
    LowPower.begin();  
    LowPower.enableWakeupFrom(&rtc, wakeUpBattery, &atime);
  }
  
  // Starts and check air temperature and humidity (DHT-22) sensor
  #ifdef SENSOR_DHT_ENABLED
    #ifdef SERIAL_DEBUG_ENABLED                
      SERIAL_DEBUG.print(F("\n\tStarting DHT sensor..."));
      SERIAL_DEBUG.flush();
    #endif
    setupStatus = initSensorDHT();
    if (setupStatus != 0) {            
      // Put RGB LED in error mode
      #ifdef RGB_LED_ENABLED        
        rgb_led.on(Color(255,0,0));
      #endif
    }
  #endif  

  // Power off LEDs if power supply is POWER LINE
  if (POWER_SUPPLY == POWER_LINE) {
    digitalWrite(LED_BUILTIN, HIGH);
    #ifdef RGB_LED_ENABLED
      rgb_led.off();
    #endif
  }  

  // Resets WDT
  #ifdef SERIAL_DEBUG_ENABLED
    SERIAL_DEBUG.print(F("\nReseting WDT..."));
    SERIAL_DEBUG.flush();
  #endif
  IWatchdog.reload();  

  // Sets the 1st wake up
  #ifdef SERIAL_DEBUG_ENABLED
    SERIAL_DEBUG.print(F("\nSetting the 1st wake up time..."));
    SERIAL_DEBUG.flush();
  #endif
  rtc.setAlarmEpoch(rtc.getEpoch() + (wd_timeout/1000) + rtc_offset);

  // LowPower.sleep();
  // LowPower.deepSleep();
  // LowPower.shutdown();
  
}

void loop() {  
    
  // Checks if its only a WDT wake up or its time to assessment
  if (wakeupTimes >= samplingPeriod_WDT) {
    
    // Updates wake up times counter
    wakeupTimes = 0;

    // Updates sampling times counter
    samplingTimes++;

    // Power on LEDs if power supply is POWER LINE
    if (POWER_SUPPLY == POWER_LINE) {
      digitalWrite(LED_BUILTIN, LOW);
      #ifdef RGB_LED_ENABLED
        rgb_led.on(Color(0,255,0));
      #endif
    }

    #ifdef SERIAL_DEBUG_ENABLED        
      SERIAL_DEBUG.print(F("\nStaring assessment at "));
      SERIAL_DEBUG.print(millis());
      SERIAL_DEBUG.flush();
    #endif    

    // Gets air temperature and humidity sensor values
    #ifdef SENSOR_DHT_ENABLED
      if (getDHTTemperature() != 0) {
        // Power on RGB LED in error mode
        #ifdef RGB_LED_ENABLED
          rgb_led.on(Color(255,0,0));
        #endif
      }
      
      if (getDHTHumidity() != 0) {
        // Power on RGB LED in error mode
        #ifdef RGB_LED_ENABLED          
          rgb_led.on(Color(255,0,0));
        #endif
      }
    #endif

    // Simulates time to assessment
    delay(500);

    // Power off LEDs if power supply is POWER LINE
    if (POWER_SUPPLY == POWER_LINE) {
      digitalWrite(LED_BUILTIN, HIGH);
      #ifdef RGB_LED_ENABLED
        rgb_led.off();
      #endif
    }

    // Resets WDT
    IWatchdog.reload();
  }  

  // Check if its time to transmit
  if (samplingTimes >= txPeriod_samplingPeriod) {
    
    // Update sampling times counter
    samplingTimes = 0;

    // Power on LEDs if power supply is POWER LINE
    if (POWER_SUPPLY == POWER_LINE) {
      digitalWrite(LED_BUILTIN, LOW);
      #ifdef RGB_LED_ENABLED
        rgb_led.on(Color(0,0,255));
      #endif
    }

    #ifdef SERIAL_DEBUG_ENABLED        
      SERIAL_DEBUG.print(F("\nStaring transmission..."));
      SERIAL_DEBUG.flush();
    #endif    

    // Print average values
    #ifdef SERIAL_DEBUG_ENABLED
      printAverageValues();                       
    #endif

    // Simulate time to assessment
    delay(500);

    // Power off LEDs if power supply is POWER LINE
    if (POWER_SUPPLY == POWER_LINE) {
      digitalWrite(LED_BUILTIN, HIGH);
      #ifdef RGB_LED_ENABLED
        rgb_led.off();
      #endif
    }

    // Reset WDT
    IWatchdog.reload();
  }  

  // Sets next wake up date/time
  //rtc.setAlarmEpoch();  
  //rtc.setAlarmEpoch(now + (wd_timeout/1000));  
  // // LowPower.deepSleep();              
  // LowPower.sleep();              
}