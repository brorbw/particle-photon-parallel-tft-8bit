#include "driver.h"

TFT_Driver tft;

int id;
void setup() {
  Serial.begin();
  Serial.println("Setting up pins");
  // id = tft.readID();
}

void loop() {
  delay(500);
  // Serial.println(id);
  // tft.writeCmdWord(0);
  Serial.println(String(tft.read16bits(), HEX));
  Serial.println(String(tft.read16bits(), HEX));
}
