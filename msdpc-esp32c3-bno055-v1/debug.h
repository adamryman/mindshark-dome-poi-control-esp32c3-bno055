#ifndef DEBUG_H
#define DEBUG_H

#ifdef DEBUG
  #include <Arduino.h>  // Ensure Arduino functions are available
  
  #define PRINTLN(x)  Serial.println(x)
  #define PRINT(x)    Serial.print(x)
  #define PRINTF(fmt, ...)  Serial.printf(fmt, ##__VA_ARGS__)

  void SETUP_SERIAL() {
    Serial.begin(115200);
    delay(10);
    Serial.println("Starting up...");
  }
#else
  #define PRINTLN(x)  // Do nothing
  #define PRINT(x)    // Do nothing
  #define PRINTF(fmt, ...)  // Do nothing

  void SETUP_SERIAL() {
    // Do nothing
  }
#endif

#endif // DEBUG_H