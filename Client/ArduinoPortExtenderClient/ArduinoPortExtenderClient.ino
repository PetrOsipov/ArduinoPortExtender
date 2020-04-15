


/*
  ArduinoPortExtenderClient.ino - Sketch for client board for ArduinoPortExtender project
  Project page: https://github.com/PetrOsipov/ArduinoPortExtender
  Copyright (c) 2020 Petr Osipov.  All right reserved.
  Released under MIT License
  Version 1.0
*/



#include <Wire.h>
#include <LinkedList.h>
#include <Servo.h>

#define CLIENT_I2C_ADDRESS 0x22

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

#define TRANSMISSION_ERROR 0xFF

// For requests, first a receiveEvent is done and placed here (only PIN_DIGITAL/ANALOG and pin number stored); When the request is then done - the pin is actually read and reported back.
uint8_t read_cache[6];
uint8_t cacheLength;
uint8_t master;

LinkedList<Servo*> servoList;


 
void setup()
{
  Wire.begin(CLIENT_I2C_ADDRESS);                // join i2c bus with address 
  Wire.onReceive(receiveEvent);    // register event
  Wire.onRequest(requestResponse);
  Serial.begin(115200);           // start serial for output
  resetCache(); // make cache to show error if unexpected request comes. 
}

void loop()
{
  delay(10);
}

uint8_t readAll(uint8_t* buf, uint8_t max = 6){
  int ctr = 0;
  while (Wire.available()){
    buf[ctr] = Wire.read();
    ctr++;
    if (ctr>=max){
      return ctr;
    }
  }
  return ctr;
}

bool initServo(uint8_t pin){
  if (!servoList.get(pin)){
    Servo* srv = new Servo();
    //srv->attach(pin);
    servoList.add(pin, srv);
  }
  return true;
}

void setServo(uint8_t pin, uint8_t value){
  Servo* s = servoList.get(pin);
  if (!s){
    bool worked = initServo(pin);
    if (!worked){
      return;
    }
    else
    {
      s = servoList.get(pin);
      if (!s){
        return;
      }
    }
  }
  s->attach(pin);
  delay(15);
  s->write(value);
  delay(15);
  //s->detach();
}

void processWrite(uint8_t* buf, uint8_t length){
  //Serial.print("Write command - length is ");
  //Serial.println(length);
  if (length>=4)
  {
    if (buf[1]==PIN_DIGITAL){
      
      digitalWrite(buf[2], buf[3]);
      
    } else if (buf[1]==PIN_SERVO){
      Serial.print("Set servo pin ");
      Serial.print(buf[2]);
      Serial.print(" to ");
      Serial.println(buf[3]);
      //digitalWrite(buf[2], buf[3]);
      setServo(buf[2], buf[3]);
    } 
    else 
    if (buf[1]==PIN_ANALOG){
      uint16_t val = 0x0000 | (buf[3] << 8) | buf[4];
      analogWrite(buf[2], val);
    } 
  }
}

// When we get the read command we can only return data in a following request. We thus store the params here. 
void processRead(uint8_t* buf, uint8_t length){
  
  if (length > 2) {
   
    if (buf[1] == PIN_ANALOG){
      uint16_t val = analogRead(buf[2]);
      
      
      
      read_cache[0] = PIN_READ;
      read_cache[1] = PIN_ANALOG;
      read_cache[2] = buf[2];
      read_cache[3] = highByte(val);
      read_cache[4] = lowByte(val);
      fillChecksum(read_cache,6);
      
      
      master = buf[3];
      cacheLength = 6;
      
    } else
    if (buf[1] == PIN_DIGITAL){
      uint8_t val = digitalRead(buf[2]);
      
      read_cache[0] = PIN_READ;
      read_cache[1] = PIN_DIGITAL;
      read_cache[2] = buf[2];
      read_cache[3] = val;
      fillChecksum(read_cache,5);
      master = buf[3];
      cacheLength = 5;
      
    }
  }
}

// If mode set is requested, call this
void processMode(uint8_t* buf, uint8_t length){
  if (length==4)
  {
    if (buf[1]==PIN_INPUT){
      pinMode(buf[2], INPUT);
    } else 
    if (buf[1]==PIN_OUTPUT){
      pinMode(buf[2], OUTPUT);
    } else
    if (buf[1]==PIN_INPUT_PULLUP){
      pinMode(buf[2], INPUT_PULLUP);
    } else
    if (buf[1]==PIN_SERVO){
      initServo(buf[2]);
    }
  }
}

// take all values except last one, and fill the last with checksum.
// calculation 0xFF XOR B0 XOR B1 XOR... BN
void fillChecksum(uint8_t* buffer, uint8_t bufsize){
  if (bufsize>=2){
    buffer[bufsize-1] = calculateChecksum(buffer, bufsize);
  }
}

// take all values except last one, and fill the last with checksum.
// calculation 0xFF XOR B0 XOR B1 XOR... BN
uint8_t calculateChecksum(uint8_t* buffer, uint8_t bufsize){
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
bool validateChecksum(uint8_t* buffer, uint8_t bufsize){
  if (bufsize>=2){
    return (buffer[bufsize-1] == calculateChecksum(buffer, bufsize));
  }
  return false;
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  
  if (Wire.available()){
    uint8_t buf[6];
    uint8_t length = readAll((uint8_t*) buf);

        
    if (!validateChecksum((uint8_t*) buf, length)){
      return;
    }
    uint8_t command = buf[0];
    if (command == PIN_WRITE){
      
      processWrite((uint8_t*) buf, length);
    }
    else if (command == PIN_READ){
      processRead((uint8_t*) buf, length);
    }
    else if (command == PIN_MODE){
      processMode((uint8_t*) buf, length);
    }
    
    
  }
}

// If a response is requested (in AnalogWrite and DigitalWrite, send out the prepared cache)
void requestResponse(){
  //read_cache is set either to a correct response or to an error. In any case, always right to be send back.
  
  Wire.write(read_cache, cacheLength);
  //as the answer is set right, reset it all to error message
  resetCache();
}

// Normally, the cache is set to an error message, so if no request came garbled or unexpected due to comm troubles, error message is sent back. 
void resetCache(){
  cacheLength = 3;
  read_cache[0] = PIN_READ;
  read_cache[1] = TRANSMISSION_ERROR;
  fillChecksum(read_cache,3);
}

