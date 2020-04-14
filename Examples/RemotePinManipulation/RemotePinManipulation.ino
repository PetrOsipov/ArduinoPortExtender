
/*
  ArduinoPortExtenderClient.ino - Example Master for ArduinoPortExtender project
  Project page: https://github.com/PetrOsipov/ArduinoPortExtender
  Copyright (c) 2020 Petr Osipov.  All right reserved.
  Released under MIT License
  Version 1.0
*/


#include "arduinoPortExtender.h"


ArduinoPortExtender* extender;
int increment = 15;
int value = 0;

  
void setup()
{
  extender = new ArduinoPortExtender(0x22, 0x0c);
  extender->pinMode(PinsUno.P_D13, PIN_OUTPUT); 
  extender->pinMode(PinsUno.P_D3, PIN_OUTPUT);
  extender->pinMode(PinsUno.P_D4, PIN_INPUT);
  extender->pinMode(PinsUno.P_A0, PIN_INPUT);
  Serial.begin(74880);
}

void loop()
{

  extender->digitalWrite(PinsUno.P_D13, (value<128));
  value +=increment;
  if (value >240 || value < 15) increment = -increment;
  
  extender->analogWrite(PinsUno.P_D3, value);
  delay(100);

  if (Serial){
    Serial.print("Digital Read D4 - ");
    Serial.println(extender->digitalRead(PinsUno.P_D4));
    Serial.print("Analog Read A0 - ");
    Serial.println(extender->analogRead(PinsUno.P_A0));
  }

  
  
  
}
