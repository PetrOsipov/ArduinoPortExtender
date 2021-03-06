
/*
  ArduinoPortExtender.h - ArduinoPortExtender library. This library allows to use an Arduino board (STM32 or ESP board would probably also work) as a Port Extender.
  Project page: https://github.com/PetrOsipov/ArduinoPortExtender
  Copyright (c) 2020 Petr Osipov.  All right reserved.
  Released under MIT License
  Version 1.0
*/

#ifndef ARDUINOPORTEXTENDER_H
#define ARDUINOPORTEXTENDER_H

#include <Wire.h>
#include <Arduino.h>


// Command actions
#define PIN_WRITE 0x01
#define PIN_READ 0x02
#define PIN_MODE 0x04
#define PIN_SERVO 0x08

// Modes for Read/Write ops
#define PIN_DIGITAL 0x01
#define PIN_ANALOG 0x02

#define PIN_INPUT 0x01
#define PIN_INPUT_PULLUP 0x02
#define PIN_OUTPUT 0x04

// Config for Arduino Uno
struct  {
  uint8_t P_D0 =0;
  uint8_t P_D1 =1;
  uint8_t P_D2 =2;
  uint8_t P_D3 =3;
  uint8_t P_D4 =4;
  uint8_t P_D5 =5;
  uint8_t P_D6 =6;
  uint8_t P_D7 =7;
  uint8_t P_D8 =8;
  uint8_t P_D9 =9;
  uint8_t P_D10 =10;
  uint8_t P_D11 =11;
  uint8_t P_D12 =12;
  uint8_t P_D13 =13;
  
  uint8_t P_A0 =14;
  uint8_t P_A1 =15;
  uint8_t P_A2 =16;
  uint8_t P_A3 =17;
  // Note - no 18 or 19, as I2C
} PinsUno;

// Config vor Arduino MEGA
struct  {
  uint8_t P_D0 =0;
  uint8_t P_D1 =1;
  uint8_t P_D2 =2;
  uint8_t P_D3 =3;
  uint8_t P_D4 =4;
  uint8_t P_D5 =5;
  uint8_t P_D6 =6;
  uint8_t P_D7 =7;
  uint8_t P_D8 =8;
  uint8_t P_D9 =9;
  uint8_t P_D10 =10;
  uint8_t P_D11 =11;
  uint8_t P_D12 =12;
  uint8_t P_D13 =13;
  uint8_t P_D14 =14;
  uint8_t P_D15 =15;
  uint8_t P_D16 =16;
  uint8_t P_D17 =17;
  uint8_t P_D18 =18;
  uint8_t P_D19 =19;
  // Note - no 20 or 21, as I2C
  uint8_t P_D22 =22;
  uint8_t P_D23 =23;
  uint8_t P_D24 =24;
  uint8_t P_D25 =25;
  uint8_t P_D26 =26;
  uint8_t P_D27 =27;
  uint8_t P_D28 =28;
  uint8_t P_D29 =29;
  uint8_t P_D30 =30;
  uint8_t P_D31 =31;
  uint8_t P_D32 =32;
  uint8_t P_D33 =33;
  uint8_t P_D34 =34;
  uint8_t P_D35 =35;
  uint8_t P_D36 =36;
  uint8_t P_D37 =37;
  uint8_t P_D38 =38;
  uint8_t P_D39 =39;
  
  uint8_t P_D40 =40;
  uint8_t P_D41 =41;
  uint8_t P_D42 =42;
  uint8_t P_D43 =43;
  uint8_t P_D44 =44;
  uint8_t P_D45 =45;
  uint8_t P_D46 =46;
  uint8_t P_D47 =47;
  uint8_t P_D48 =48;
  uint8_t P_D49 =49;
  
  uint8_t P_D50 =50;
  uint8_t P_D51 =51;
  uint8_t P_D52 =52;
  uint8_t P_D53 =53;
  
  uint8_t P_A0 =54;
  uint8_t P_A1 =55;
  uint8_t P_A2 =56;
  uint8_t P_A3 =57;
  uint8_t P_A4 =58;
  uint8_t P_A5 =59;
  uint8_t P_A6 =60;
  uint8_t P_A7 =61;
  uint8_t P_A8 =62;
  uint8_t P_A9 =63;
  uint8_t P_A10 =64;
  uint8_t P_A11 =65;
  uint8_t P_A12 =66;
  uint8_t P_A13 =67;
  uint8_t P_A14 =68;
  uint8_t P_A15 =69;
   
} PinsMEGA;

class ArduinoPortExtender {
public:
  ArduinoPortExtender(uint8_t addr,  uint8_t addrMaster );
  virtual ~ArduinoPortExtender();

  // takes PIN_INPUT, PIN_INPUT_PULLUP, PIN_OUTPUT. Use provided structs for the boards to avoid using illegal pins.
  // Also - avoid using pins D0 or D1, as it is the serial port and debug info is going out there, except if you compiled Arduino sketch with nodebug
  // It does not guarantee that the command arrived ungarbled and was decoded right by the client, 0 code only means that command was received by client and acknowledged.
  // returns 0 if ok, 1, 2, 3 or 4 if transmission fails (see https://www.arduino.cc/en/Reference/WireEndTransmission codes)
  virtual uint8_t pinMode(uint8_t pin, uint8_t mode);

  // read data from pin. (0 or 1, pins 0 to 17 (D0-D13, A0-A3 on UNO)) 
  // returns 0xFF in error case. Does not retry to send once more. 
  virtual uint8_t digitalRead(uint8_t pin);


  // read data from pin (0-1023, only pins 14-17 (A0-A3) on UNO )
  // returns 0xFFFF in error case. Does not retry to send once more.
  virtual uint16_t analogRead(uint8_t pin);

  // write data to pin. It does not guarantee that the command arrived ungarbled and was decoded right by the client, 0 code only means that command was received by client and acknowledged.
  // returns 0 if ok, 1, 2, 3 or 4 if transmission fails (see https://www.arduino.cc/en/Reference/WireEndTransmission codes
  virtual uint8_t digitalWrite(uint8_t pin, uint8_t val);

  // write data to pin. It does not guarantee that the command arrived ungarbled and was decoded right by the client, 0 code only means that command was received by client and acknowledged.
  // returns 0 if ok, 1, 2, 3 or 4 if transmission fails (see https://www.arduino.cc/en/Reference/WireEndTransmission codes
  virtual uint8_t analogWrite(uint8_t pin, uint16_t val);

  // write servo angle to pin. Value is between 0 and 180. It does not guarantee that the command arrived ungarbled and was decoded right by the client, 0 code only means that command was received by client and acknowledged.
  // Restrictions as per Servo.h https://www.arduino.cc/en/reference/servo to PWM output happen. 
  // returns 0 if ok, 1, 2, 3 or 4 if transmission fails (see https://www.arduino.cc/en/Reference/WireEndTransmission codes
  virtual uint8_t servoWrite(uint8_t pin, uint8_t val);

  
private:
  void fillChecksum(uint8_t* buffer, uint8_t size);
  uint8_t calculateChecksum(uint8_t* buffer, uint8_t size);
  bool validateChecksum(uint8_t* buffer, uint8_t size);
  uint8_t masterAddress;
  uint8_t client;
  static void receiveEvent();
};



#endif /* ARDUINOPORTEXTENDER_H */
