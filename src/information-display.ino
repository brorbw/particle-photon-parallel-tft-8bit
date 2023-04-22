#include "driver.h"

// Driver tft;

void setup() {
  Serial.begin();
  Serial.println("Setting up pins");
  // tft.readID();
  Serial.println("Setting up done");
}

void loop() {
  delay(500);
  Serial.println("Setting up down");
}
