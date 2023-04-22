#include "driver.h"

TFT_Driver tft;

int id;
void setup() {
  Serial.begin();
  Serial.println("Setting up pins");
  id = tft.readID();
  Serial.println("Setting up done");
}

void loop() {
  delay(500);
  Serial.println(id);
}
