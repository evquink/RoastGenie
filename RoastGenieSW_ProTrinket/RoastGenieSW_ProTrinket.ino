//BeanMassProbe

/*
The MIT License (MIT)

Copyright (c) [2014] [Evan Graham]

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/



#include <SPI.h>
#include "Adafruit_MAX31855.h"

#define         RATE        2                                    //rate = # of times/second a temperature reading is taken

#define         DO          5
#define         CS          4
#define         CLK         3


Adafruit_MAX31855 thermocouple(CLK, CS, DO);

int t_delay = int(1000/RATE);                                     // this is the number of thousandths of a second between temp readings
int TmaxLoopCount=0;                                              // # of measurements since Tmax was set
int TminLoopCount=0;                                              // # of measurements since Tmin was set

double Tmin;                                                      // best estimate of minumum temperature (this is bean mass temp)
double Tmax;                                                      // best estimate of maximum temperature (this is environmental temp)
double Tnow;                                                      // current thermocouple reading (t=0)
double Tlast;                                                     // last thermocouple reading (t=-1)
double T_dif_now;                                                 // temp difference between most recent readings(t = 0 and = -1)
double T_dif_last;                                                // temp difference between previous two readings (t = -1 and t = -2)

char    buffer[81]             = "";                              //a place to put fomatted data for printing 

long interval = 500;                                              // interval at which to read thermocouple (milliseocnds)
unsigned long currentMillis = millis(); 
long previousMillis = 0;                                          // will store last time thermocouple was read
int inByte = 0;                                                   // incoming serial byte

void setup() {
  Serial.begin(9600);
  // wait for MAX chip to stabilize
  delay(500);
  Tmin = thermocouple.readFarenheit();
  Tlast = Tnow = Tmax = Tmin;
  T_dif_now = 0;
  T_dif_last = 0;
}

void loop() {
  currentMillis = millis();
  if(currentMillis - previousMillis > interval) {
     previousMillis = currentMillis;   
     Tnow = thermocouple.readFarenheit();
     TminLoopCount ++;
     TmaxLoopCount ++;
     T_dif_now = Tnow - Tlast;
    
     if (T_dif_now >= 0.0 && T_dif_last < 0.0 && TminLoopCount > 1){    // this is a local minimum
       Tmin = Tlast;                                                    // best estimate of environmental temp
       TminLoopCount = 0;                                               // reset loop counter
     }
   
     if (T_dif_now <= 0.0 && T_dif_last > 0.0 && TmaxLoopCount > 1){    // this is a local maximum
       Tmax = Tlast;                                                    // best estimate of bean mass temp
       TmaxLoopCount = 0;                                               // reset loop counter
     }
  
     Tlast = Tnow;
     T_dif_last = T_dif_now;
  }
  
  if (Serial.available() > 0) {                                        // send temp when character is received
    inByte = Serial.read();
    if(inByte == '?'){
      sprintf(buffer,"%03d, %03d, %03d",int(Tnow),int(Tmin),int(Tmax));
      Serial.println(buffer);
    }
  }

}
