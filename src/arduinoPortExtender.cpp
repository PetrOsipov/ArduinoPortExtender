/*
  ArduinoPortExtender.h - ArduinoPortExtender library. This library allows to use an Arduino board (STM32 or ESP board would probably also work) as a Port Extender.
  Project page: https://github.com/PetrOsipov/ArduinoPortExtender
  Copyright (c) 2020 Petr Osipov.  All right reserved.
  Released under MIT License
  Version 1.0
*/

#include "arduinoPortExtender.h"

ArduinoPortExtender::ArduinoPortExtender(uint8_t addrClient, uint8_t addrMaster = 0x0c){
  client = addrClient;
  masterAddress = addrMaster;
  Wire.begin(addrMaster);

}



ArduinoPortExtender::~ArduinoPortExtender(){
 
}


void ArduinoPortExtender::receiveEvent(){
  
  uint8_t rbuf[5];
  int ctr = 0;
  while(Wire.available())    // slave may send less than requested
  { 
 
    rbuf[ctr] = Wire.read(); // receive a byte  
    ctr++;
  }
        
      
      
}

uint8_t ArduinoPortExtender::pinMode(uint8_t pin, uint8_t mode){
  uint8_t buf[4];
  buf[0] = PIN_MODE;
  buf[1] = mode;
  buf[2] = pin;
  fillChecksum((uint8_t *)buf, 4);
  Wire.beginTransmission(client);
  Wire.write(buf, 4);
  return Wire.endTransmission();
}

uint8_t ArduinoPortExtender::digitalRead(uint8_t pin){
  uint8_t buf[5];
  buf[0] = PIN_READ;
  buf[1] = PIN_DIGITAL;
  buf[2] = pin;
  buf[3] = masterAddress;
  fillChecksum((uint8_t *)buf, 5);
  
  Wire.beginTransmission(client);
  Wire.write(buf, 5);
  int trerr = Wire.endTransmission();

  if (trerr!=0) return 0xFF; //ERROR
  int rcvcount = Wire.requestFrom(client, 5);
  if (rcvcount<2) return 0xFF; //ERROR
  uint8_t rbuf[5];
  
  int ctr = 0;
  while(Wire.available())    // slave may send less than requested
  { 
    rbuf[ctr] = Wire.read(); // receive a byte as character
    ctr++;
  }
  // we got a correct message? If decoded ok, then return the value, else error.
  uint8_t ret=0xFF;
  if (ctr == 5 && rbuf[0]==PIN_READ && rbuf[1]==PIN_DIGITAL && rbuf[2]==pin && rbuf[4]==calculateChecksum(rbuf,5)){
     ret = rbuf[3];
  }
  return ret;
}

// read data from pin (0-1023, only pins 14-17 (A0-A3) on UNO )
uint16_t ArduinoPortExtender::analogRead(uint8_t pin){
  uint8_t buf[5];
  buf[0] = PIN_READ;
  buf[1] = PIN_ANALOG;
  buf[2] = pin;
  buf[3] = masterAddress;
  fillChecksum((uint8_t *) buf, 5);
  Wire.beginTransmission(client);
  Wire.write(buf, 5);
  int trerr = Wire.endTransmission();
  if (trerr!=0) return 0xFFFF; //ERROR

  
  int rcvcount = Wire.requestFrom(client, 6);
  if (rcvcount<2) return 0xFFFF; //ERROR
  uint8_t rbuf[6];

  int ctr = 0;
  while(Wire.available())    // slave may send less than requested
  { 
    rbuf[ctr] = Wire.read(); // receive a byte as character
    ctr++;
  }
  // we got a correct message?  If decoded ok, then return the value, else error.
  uint16_t ret=0xFFFF;
  if (ctr == 6 && rbuf[0]==PIN_READ && rbuf[1]==PIN_ANALOG && rbuf[2]==pin && rbuf[5]==calculateChecksum(rbuf,6)){
     uint16_t hbyte = rbuf[3];
     uint16_t lbyte = rbuf[4];
     ret = 0x0000 | (hbyte << 8) | lbyte;
  }
  return ret;
}

// write data to pin
uint8_t ArduinoPortExtender::digitalWrite(uint8_t pin, uint8_t val){
  uint8_t buf[5];
  buf[0] = PIN_WRITE;
  buf[1] = PIN_DIGITAL;
  buf[2] = pin;
  buf[3] = val;
  fillChecksum((uint8_t *)buf, 5);
  Wire.beginTransmission(client);
  Wire.write(buf, 5);
  return Wire.endTransmission();
}
uint8_t ArduinoPortExtender::analogWrite(uint8_t pin, uint16_t val){
  uint8_t buf[6];
  buf[0] = PIN_WRITE;
  buf[1] = PIN_ANALOG;
  buf[2] = pin;
  buf[3] = highByte(val);
  buf[4] = lowByte(val);
  fillChecksum((uint8_t *)buf, 6);
  Wire.beginTransmission(client);
  Wire.write(buf, 6);
  return Wire.endTransmission();
}

// take all values except last one, and fill the last with checksum.
// calculation 0xFF XOR B0 XOR B1 XOR... BN
void ArduinoPortExtender:: fillChecksum(uint8_t* buffer, uint8_t bufsize){
  if (bufsize>=2){
    buffer[bufsize-1] = calculateChecksum(buffer, bufsize);
  }
}

// take all values except last one, and fill the last with checksum.
// calculation 0xFF XOR B0 XOR B1 XOR... BN
uint8_t ArduinoPortExtender:: calculateChecksum(uint8_t* buffer, uint8_t bufsize){
  uint8_t res = 0xFF;
  if (bufsize>=2){
    for (int i = 0; i<=bufsize-2; i++){
      res ^= buffer[i];
      i++;
    }
  }
  return res;
}

// validate checksum. Calculate it, and see if it matches.
// calculation 0xFF XOR B0 XOR B1 XOR... BN
bool ArduinoPortExtender::validateChecksum(uint8_t* buffer, uint8_t bufsize){
  if (bufsize>=2){
    return (buffer[bufsize-1] == calculateChecksum(buffer, bufsize));
  }
  return false;
}

