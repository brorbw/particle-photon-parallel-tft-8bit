#include "driver.h"

#define TFT_RD  D4
#define TFT_WR  D3
#define TFT_RS  D2
#define TFT_CS  D1
#define TFT_RST D1

// data pins
// Port data |A7 |A6 |A5 |A4 |A3 |A2 |A1 |A0 |
// Pin stm32 |PA7|PA6|PA5|PA4|PA3|PA2|PA1|PA0|
#define TFT_D0 A0
#define TFT_D1 A1
#define TFT_D2 A2
#define TFT_D3 A3
#define TFT_D4 A4
#define TFT_D5 A5
#define TFT_D6 A6
#define TFT_D7 A7

#define RD_ACTIVE  digitalWrite(TFT_RD, LOW)
#define RD_IDLE    digitalWrite(TFT_RD, HIGH)
#define WR_ACTIVE  digitalWrite(TFT_WR, LOW)
#define WR_IDLE    digitalWrite(TFT_WR, HIGH)
#define CD_COMMAND digitalWrite(TFT_RS, LOW)
#define CD_DATA    digitalWrite(TFT_RS, HIGH)
#define CS_ACTIVE  digitalWrite(TFT_CS, LOW)
#define CS_IDLE    digitalWrite(TFT_CS, HIGH)
#define RST_ACTIVE digitalWrite(TFT_RST, LOW)
#define RST_IDLE   digitalWrite(TFT_RST, HIGH)

#define RD_STROBE      {RD_ACTIVE; RD_IDLE;} // Not use
#define WR_STROBE      {WR_ACTIVE; WR_IDLE;} // Not use

void setupControlPins() {
  pinMode(TFT_RD, OUTPUT);
  pinMode(TFT_WR, OUTPUT);
  pinMode(TFT_RS, OUTPUT);
  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_RST, OUTPUT);
}

void setDatapinsAsInput(){
  pinMode(TFT_D0, INPUT_PULLDOWN);
  pinMode(TFT_D1, INPUT_PULLDOWN);
  pinMode(TFT_D2, INPUT_PULLDOWN);
  pinMode(TFT_D3, INPUT_PULLDOWN);
  pinMode(TFT_D4, INPUT_PULLDOWN);
  pinMode(TFT_D5, INPUT_PULLDOWN);
  pinMode(TFT_D6, INPUT_PULLDOWN);
  pinMode(TFT_D7, INPUT_PULLDOWN);
}
void setDatapinsAsOutput(){
  pinMode(TFT_D0, OUTPUT);
  pinMode(TFT_D1, OUTPUT);
  pinMode(TFT_D2, OUTPUT);
  pinMode(TFT_D3, OUTPUT);
  pinMode(TFT_D4, OUTPUT);
  pinMode(TFT_D5, OUTPUT);
  pinMode(TFT_D6, OUTPUT);
  pinMode(TFT_D7, OUTPUT);
}

TFT_Driver::TFT_Driver(void) {
  setupControlPins();
  CS_IDLE; // Disable CS
	// CD_DATA; // Enable Command
  WR_IDLE; // Disable WR
  RD_IDLE; // Disable RD

  // toggle RST low to reset
  RST_IDLE;   // Set Reset HIGH
  delay(5);
  RST_ACTIVE; // Set Reset LOW
  delay(20);
  RST_IDLE;   // Set Reset HIGH
  delay(150);
}


uint16_t TFT_Driver::read16bits(void)
{
  uint16_t ret;
  uint8_t lo;
  uint8_t hi;
  CS_ACTIVE;
  CD_DATA;
  hi = read8();
  //all MIPI_DCS_REV1 style params are 8-bit
  lo = read8();
  CS_IDLE;
  return (hi << 8) | lo;
}

uint8_t TFT_Driver::read8(void){
  setDatapinsAsInput();
  RD_ACTIVE;
  delay(5);

  uint8_t ret = 0x0000;
  if(digitalRead(TFT_D0)){
    ret |= 1 << 0;
  } else {
    ret |= 0 << 0;
  }
  if(digitalRead(TFT_D1)){
    ret |= 1 << 1;
  } else {
    ret |= 0 << 1;
  }
  if(digitalRead(TFT_D2)){
    ret |= 1 << 2;
  } else {
    ret |= 0 << 2;
  }
  if(digitalRead(TFT_D3)){
    ret |= 1 << 3;
  } else {
    ret |= 0 << 3;
  }
  if(digitalRead(TFT_D4)){
    ret |= 1 << 4;
  } else {
    ret |= 0 << 4;
  }
  if(digitalRead(TFT_D5)){
    ret |= 1 << 5;
  } else {
    ret |= 0 << 5;
  }
  if(digitalRead(TFT_D6)){
    ret |= 1 << 6;
  } else {
    ret |= 0 << 6;
  }
  if(digitalRead(TFT_D7)){
    ret |= 1 << 7;
  } else {
    ret |= 0 << 7;
  }

  RD_IDLE;
  delay(5);
  return ret;
}
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
void TFT_Driver::write8(uint8_t c) {
  setDatapinsAsOutput();

  digitalWrite(TFT_D0, (CHECK_BIT(c, 0)) ? HIGH : LOW);
  digitalWrite(TFT_D1, (CHECK_BIT(c, 1)) ? HIGH : LOW);
  digitalWrite(TFT_D2, (CHECK_BIT(c, 2)) ? HIGH : LOW);
  digitalWrite(TFT_D3, (CHECK_BIT(c, 3)) ? HIGH : LOW);
  digitalWrite(TFT_D4, (CHECK_BIT(c, 4)) ? HIGH : LOW);
  digitalWrite(TFT_D5, (CHECK_BIT(c, 5)) ? HIGH : LOW);
  digitalWrite(TFT_D6, (CHECK_BIT(c, 6)) ? HIGH : LOW);
  digitalWrite(TFT_D7, (CHECK_BIT(c, 7)) ? HIGH : LOW);
  WR_STROBE;
  //delayMicroseconds(50); //used to observe patterns
}

uint16_t TFT_Driver::readReg16(uint16_t reg)
{
  uint16_t ret;
  uint8_t lo;
  writeCmdWord(reg);
  ret = read16bits();
  setDatapinsAsOutput();
  return ret;
}

void TFT_Driver::writeCmdWord(uint16_t c) {
  CD_COMMAND;
  CS_ACTIVE;
  write8(c >> 8);
  write8(c & 0xff);
  CS_IDLE;
}

uint16_t TFT_Driver::readID(void)
{
    uint16_t ret;
    ret = readReg16(0);           //forces a reset() if called before begin()
    Serial.println("readReg16(0)=0x" + String(ret,HEX));
    return ret;
}
