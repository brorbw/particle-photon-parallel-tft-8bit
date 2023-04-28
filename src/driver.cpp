#include "driver.h"

#define TFT_RD  D4
#define TFT_WR  D3
#define TFT_RS  D2
#define TFT_CS  D1
#define TFT_RST D0

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
#define WR_STROBE      {WR_ACTIVE; WR_IDLE;}

void setupControlPins() {
  pinMode(TFT_RD, OUTPUT);
  pinMode(TFT_WR, OUTPUT);
  pinMode(TFT_RS, OUTPUT);
  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_RST, OUTPUT);
}

void TFT_Driver::setReadDataBus(){
  pinMode(TFT_D0, INPUT);
  pinMode(TFT_D1, INPUT);
  pinMode(TFT_D2, INPUT);
  pinMode(TFT_D3, INPUT);
  pinMode(TFT_D4, INPUT);
  pinMode(TFT_D5, INPUT);
  pinMode(TFT_D6, INPUT);
  pinMode(TFT_D7, INPUT);
}
void TFT_Driver::setWriteDataBus(){
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

uint8_t TFT_Driver::read8(void){
  setReadDataBus();
  RD_ACTIVE;
  delay(5);
  uint8_t ret = 0x0000;
  if(digitalRead(TFT_D0)){
    ret |= 1 << 0;
  }
  if(digitalRead(TFT_D1)){
    ret |= 1 << 1;
  }
  if(digitalRead(TFT_D2)){
    ret |= 1 << 2;
  }
  if(digitalRead(TFT_D3)){
    ret |= 1 << 3;
  }
  if(digitalRead(TFT_D4)){
    ret |= 1 << 4;
  }
  if(digitalRead(TFT_D5)){
    ret |= 1 << 5;
  }
  if(digitalRead(TFT_D6)){
    ret |= 1 << 6;
  }
  if(digitalRead(TFT_D7)){
    ret |= 1 << 7;
  }
  RD_IDLE;
  delay(5);
  return ret;
}

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

void TFT_Driver::write8(uint8_t c) {
  setWriteDataBus();

  digitalWrite(TFT_D0, (CHECK_BIT(c, 0)) ? HIGH : LOW);
  digitalWrite(TFT_D1, (CHECK_BIT(c, 1)) ? HIGH : LOW);
  digitalWrite(TFT_D2, (CHECK_BIT(c, 2)) ? HIGH : LOW);
  digitalWrite(TFT_D3, (CHECK_BIT(c, 3)) ? HIGH : LOW);
  digitalWrite(TFT_D4, (CHECK_BIT(c, 4)) ? HIGH : LOW);
  digitalWrite(TFT_D5, (CHECK_BIT(c, 5)) ? HIGH : LOW);
  digitalWrite(TFT_D6, (CHECK_BIT(c, 6)) ? HIGH : LOW);
  digitalWrite(TFT_D7, (CHECK_BIT(c, 7)) ? HIGH : LOW);
  WR_STROBE;
  delayMicroseconds(50); //used to observe patterns
}

uint16_t TFT_Driver::readID(void)
{
    uint16_t ret;
    ret = readReg16(0);           //forces a reset() if called before begin()
    Serial.println("readReg16(0)=0x" + String(ret,HEX));
    return ret;
}

uint16_t TFT_Driver::readReg16(uint16_t reg)
{
  uint16_t ret;
  writeCmdWord(reg);
  //    READ_16(ret);
  ret = read16bits();
  return ret;
}

uint16_t TFT_Driver::read16bits(void)
{
  uint8_t lo;
  uint8_t hi;
  CS_ACTIVE;
  CD_DATA;
  WR_IDLE;
  RD_ACTIVE;
  hi = read8();

  CS_IDLE;
  CS_ACTIVE;

  lo = read8();
  RD_IDLE;
  CS_IDLE;
  return (hi << 8) | lo;
}

void TFT_Driver::writeCmdWord(uint16_t c) {
  CD_COMMAND;
  CS_ACTIVE;
  write8(c >> 8);
  write8(c & 0xff);
  CS_IDLE;
}

uint8_t TFT_Driver::readcommand8(uint8_t c) {
  writeCmdByte(c);
  CS_ACTIVE;
  CD_DATA;
  uint8_t data = read8();
  CS_IDLE;
  return data;
}

void TFT_Driver::writeCmdByte(uint8_t c) {
  CD_COMMAND;
  CS_ACTIVE;
  write8(c);
  CS_IDLE;
}
