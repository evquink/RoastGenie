const char fileName[] = "RoastGenie_V18";
// Bugs

// debug code needs to be removed

// Version 17.2: Cleanup
// enable interrupt controlling heater _ONLY_ during roast state
// add ability to shutdown raspberry pi
//
// Version 18 for New RoastGenie Hardware
// adds led around user button
// replaces 2 x 16 lcd with 4 x 20 lcd
// Issues:
// make coffee display less "flashy" when running the encoder
// address issue -- arduino does not behave well when raspi link is lost mid process
// compress code
// set up led to come on when active (starting w/green weight entry when not connected)



// Libraries
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/programspace.h>
#include <Wire.h>
#include <LiquidTWI2.h>
#include <Encoder.h>
#include <Bounce2.h>
#include <PID_v1.h>
#include <Time.h>
#include <max6675.h>
#include <SoftwareSerial.h>
#include "Adafruit_Thermal.h"

// Pins
#define zeroCross       2
#define encB            3
#define select          4
#define drumSpd         5
#define fanSpd          6                    //was pin 11 in V10
#define drumPos         7
#define tCs1            8                    //was tCs0 in previous versions
#define gate            9
#define userButton      10                   //2nd button pin control panel
#define tD0             11                   //was pin 6 in V10
#define tClk            12
#define tCs0            13                   //was tCs1 in previous versions
#define encA            14                   //AKA A0
#define printer_RX_Pin  15                   // This is the green wire
#define printer_TX_Pin  16                   // This is the yellow wire
#define led             17                   // This is the led around the user button
#define i2dat           18                   //AKA A4
#define i2clk           19                   //AKA A5  

// Constants
#define PULSE           4                    //trigger pusle width for AC control

//Settings
#define T_outSet0      510                   //initial setpoint for outlet temperature
#define T_outSetMin    0
#define T_outSetMax    600
#define T_outCooled    130                   // outlet temperature that must be reached before cooling cycle is automaticlly stopped
#define T_postCrack    490                   // temperature to set back to once first crack has been reached
#define fanSpdPctMin   0                     //minimum allowable fan speed
#define drumSpdPctMin  0                     //minimum allowable drum speed
#define tDrumPeriod    20                    //the number of seconds required for the drum to complete ~2 revolutions.  Used to determine if drum is stopped
#define tShutDown      1500                  //the maximum allowable roast time before forced shutdown
#define tDispDuration  3                     //time (seconds) before display reverts to normal mode



// Flags and flow control variables
boolean   selectState          = HIGH;       //debounced state of the select button
boolean   selectStateFlag      = LOW;        //flag to ensure select button press is relased before next detection
boolean   lastSelectState      = HIGH;
boolean   userButtonState      = HIGH;       //debounced stat of the user button
boolean   userButtonStateFlag  = LOW;         //flag to ensure user button press is released before next detection
boolean   drumPosState         = HIGH;       //debounced state of the drum positon switch
boolean   drumPosLast          = HIGH;       //this is the last state of the drum position switch
boolean   drumState            = LOW;        //this is the state of the roaster drum LOW = stopped, HIGH = turning
boolean   lastRotation         = LOW;        //flag to store signal to turn drum off on next revolution
boolean   lastUserButtonState  = 1;
boolean   subStateToggleLock   = 0;
boolean   subStateChange       = 0;          // high when substate changes
boolean   newSecond            = 0;
boolean   connectedFlag;
int       pgmState             = 0;          //init = 0, load = 5, waiting = 10, roasting = 15, cooling = 20, done = 25
int       subState             = 0;          //tracks sub states within a state
boolean   ctChanged            = LOW;        //keeps track of when coffee type changes during selection process
boolean   userButtonEnabled     = LOW;        //enables / disables user button

//Serial Communication Variables
#define DATABUFFERSIZE           80
char    dataBuffer[DATABUFFERSIZE+1];        //Add 1 for NULL terminator
byte    dataBufferIndex        = 0;          //place to store incoming serial messages
char    startChar              = '{';        //starting character for valid serial message
char    endChar                = '}';        //ending character for valid serial message
boolean storeString            = false;      //flag to indicate valid serial message is coming in
boolean serReady               = 0;          //1 if a valid message is waiting 1 loop only
boolean serBusy                = 0;          //1 if waiting on a reply from the raspi
char    serMsg;                              //place to store command byte of serial message
long    serCnt                 = 0;          //count of loops since serial message was sent - used for timeout
long    serCntLst              = 0;          //used to store loops to get serial communication for debug output
boolean serErr                 = 0;          //place to store serial communication error messages
long    serTO                  = 250;        //how many program loops to wait for a serial response **Make as small as possible**
char    buffer[81]             = "";         //a place to put fomatted data for printing to the lcd

// Counters
int       loopCount            = 0;          //zero for the first loop through a new mode
int       encValueNow          = 0;          //Place to store the value of the parameter
int       encValueOld          = 0;
int       i;                                 //a loop counter
int       j;                                 //a loop counter
int       n;                                 //a loop counter

// Heater, fan, and motor control
double    htrPwrI              = 0;          // Desired heater power (% of max) based on inlet temp
double    htrPwrO              = 0;          // Desired heater power (% of max) based on inlet temp
int       htrPwrPct;                         // The actual heater power desired in % of max available
int       htrPwrCnt;                         // The actual heater power desired in clock counts
int       fanSpdPct            = 100;        //set point for fan speed - percent of max
int       fanSpdCnt;
int       drumSpdPct           = 100;        //set point for drum speed - percent of max
int       drumSpdCnt;


    //Time Variables (tXxxx)
long      tLast;                             //used for once-per-second events    
long      tRoast0              = 0;          //time roasting started
long      tRoast               = 0;          //length of time in roast state
long      tCool0               = 0;          //time cooling started
int       tCool                = 0;          //length of time in cooling state
long      t1stCrack0           = 0;          //time at which first crack occured
int       t1stCrack            = 0;          //time elapsed past 1st crack
long      tInit0               = 0;          //time initialization started
int       tCurrent             = 0;          //time elapsed in current mode
long      tDrumPosLast         = 0;          //the last time the drum position sensor was closed
long      tDispReset           = 0;          //time at which display should reset

    //Temperature Variables T_xxxx, dT_xxxx  
double    T_inSet              = 700;        // Setpoint for the inlet temperature in degrees F
double    T_in                 = 68;         // current inlet temperature
double    T_out                = 68;         // current outlet temperature
double    T_outSet             = T_outSet0;  // Setpoint for the outlet temperature in degrees F

  //Coffee Data
int      coffeeType            = 0;          // index of the type of coffee being roasted
String   coffeeOrigin;                       // origin of selected coffee
String   coffeeFarm;                         // farm from which selected coffee comes
String   dateTime;                           // date and time of roast
int      greenWeight           = 250;        // weight of charge before roasting in grams
int      roastedWeight;                      // weight of charge after roasting in grams
int      invCnt;                             // number of different coffees in the inventory 
double   massLost;

  //PID Constants
double   kpI = 2.5;          // proportional control constant
double   kiI = 0.1;          // integral control constant
double   kdI = 0;            // differential control constant  
double   kpO = 0.9;          // proportional control constant
double   kiO = 0.1;          // integral control constant
double   kdO = 0;            // differential control constant

// This table linearized the relationship between power desired and actual power output
int fcnIn[] = {0,10,20,30,40,50,60,70,80,90,100}; // Desired heater power in percent of maximum available
int fcnOut[] = {500,395,359,331,306,280,254,222,189,155,100};  // Desired heater power in timer count (corresponding to htrpwrPct)

// Object Declarations
MAX6675     tcIn(tClk, tCs0, tD0);
MAX6675     tcOut(tClk, tCs1, tD0);
LiquidTWI2  lcd(0);                        //lcd is called "lcd"
Bounce      selectDb      = Bounce();      //debounce the select button on the encoder
Bounce      drumPosDb     = Bounce();      //debounce the drum position switch 
Bounce      userButtonDb  = Bounce();      //debounce the user button (2nd button on control panel)
Adafruit_Thermal printer(printer_RX_Pin, printer_TX_Pin);
Encoder     myEnc (encB,encA);             //rotary encoder is called "myEnc"
PID         outPID(&T_out, &htrPwrO, &T_outSet, kpO,kiO,kdO, DIRECT);
PID         inPID(&T_in, &htrPwrI, &T_inSet, kpI, kiI, kdI, DIRECT);


void setup(){
  Serial.begin(115200);           //setup serial port for communiction to rasPi
  lcd.setMCPType(LTI_TYPE_MCP23008); 
  lcd.begin(20, 4);               
  printer.begin();                //Setup the thermal printer
  pinMode(select,INPUT);          //This is the select button on the encoder, set the associated pin as an input
  digitalWrite(select,HIGH);      //Set the pull-up resistor on the encoder select button
  selectDb.attach(select);        //Setup for debouncing
  selectDb.interval(5);           //Setup for debouncing
  pinMode(zeroCross, INPUT);      //zero cross detect
  digitalWrite(zeroCross, HIGH);  //enable pull-up resistor
  pinMode(gate, OUTPUT);          //triac gate control
  pinMode(drumSpd, OUTPUT);       //pwm output for drum
  pinMode(fanSpd, OUTPUT);        //pwm output for fan
  pinMode(drumPos, INPUT);        //drum position switch
  digitalWrite(drumPos, HIGH);    //enable pull-up resistor
  drumPosDb.attach(drumPos);      //Setup for debouncing
  drumPosDb.interval(5);          //Setup for debouncing
  pinMode(userButton, INPUT);     //2nd pushbutton on control panel
  digitalWrite(userButton, HIGH); //enable pull-up resistor
  userButtonDb.attach(userButton);//Setup for debouncing
  userButtonDb.interval(5);       //Setup for debouncing
  pinMode (led,OUTPUT);           //LED on user button
  digitalWrite(led, HIGH);        //Turn LED off for starters
  myEnc.write(encValueNow);
  outPID.SetMode(AUTOMATIC);      //initialize PID loop for outlet temp
  inPID.SetMode(AUTOMATIC);       //initialize PID loop for inlet temp
  tLast = now();

  // set up Timer1 
  //(see ATMEGA 328 data sheet pg 134 for more details)
  OCR1A = 505;      //initialize the comparator
  TIMSK1 = 0x03;    //enable comparator A and overflow interrupts
  TCCR1A = 0x00;    //TCCR1A and TCCR1B are control registers 
  TCCR1B = 0x00;    //that configure the behavior of the timer:
                    // *normal operation
                    // *do not control output pins directly
                    // * clock source / 256  
                    

}  

void loop(){
  // do this stuff every single loop
  selectDb.update();                //check to see if the select button has been pressed once per loop
  selectState = selectDb.read();
  drumPosDb.update();               //check to see if the drum positon switch has been activiated once per loop 
  drumPosState = drumPosDb.read();
  userButtonDb.update();            //check to see if the user button has been pressed
  userButtonState = userButtonDb.read();
  subStateChange = 0;               
  outPID.Compute();                 // do PID magic - frequency of calc is controlled by PID code, call on every loop
  inPID.Compute();                  // do PID magic
  getSerialString();                // check for serial messages
  if(userButtonEnabled){
      digitalWrite(led, LOW);        //Turn LED on when button is enabled
  }
  else{
      digitalWrite(led, HIGH);        //Turn LED off when button is disabled
  } 
  
  // This is the temp measurement and control portion, thermal stuff is slow so we only do this once per second.
  // Temp measurement and control is only active in "roasting" and "cooling" modes.
  if (now() > tLast){  //only read thermocouples once per second 
    tLast = now();
    newSecond=1;
    
    ///////////////////////////////////////////////////////////////
    //  PUT DEBUGGING MESSAGES HERE for Once Per Second Updating //
    //////////////////////////////////////////////////////////////  
        
    T_in = tcIn.readFarenheit();    // read inlet thermocouple
    T_out = tcOut.readFarenheit();  // read outlet thermocouple
    htrPwrPct = 100 * min(htrPwrO/255, htrPwrI/255);   //htrPweO and htrPwrI come from PID calcs
    htrPwrCnt = multiMap(htrPwrPct,fcnIn,fcnOut,11);   //determine htrPwrCnt from multiMap interpolation routine
    if(pgmState == 300 || pgmState == 400){ //send roasting data to serial port during roasting and cooling states
      sprintf(buffer,"?D,%05d,%03d,%03d,%03d,%03d,%03d,%05d\n",int(now()-tRoast0),int(T_in),int(T_out),int(T_outSet),int(htrPwrPct),pgmState,t1stCrack);
      Serial.print(buffer);
    }
  }  
  // BOTTOM OF Once-per-second LOOP
  
  //***************************************************************************************
  if (pgmState == 0){  //************************ INITIALIZE ******************************
    if (loopCount == 0){
      loopCount = 1;
      tRoast0=0;
      tRoast=0;
      t1stCrack0=0;
      t1stCrack=0;
      tInit0=now();
      analogWrite(fanSpd, 0);       //fan off (blue LED)
      analogWrite(drumSpd,0);       //drum off (green LED)
    }
    else {
      // *** PROGRAM CODE HERE *** //
      if (1){ // *** Condition to move to next state ***
        pgmState = 100;
        loopCount = 0;
      }
    }      
  }
  //***************************************************************************************  
  if (pgmState == 100){  //**************************** CONNECT ***************************
    if (loopCount == 0){ 
      loopCount = 1;
      connectedFlag = 0;
      lcd.clear();
      lcd.setCursor(3,1);
      lcd.print(fileName);
      lcd.setCursor(3,2);
      lcd.print(F("Connecting...   "));
      lcd.setCursor(3,3);
      lcd.print(F("Free SRAM:"));
      lcd.setCursor(14,3);
      lcd.print(freeRam());
    }
    else {                
      if(serReady){
        if(serMsg == 'c'){
          serReady = 0;
          lcd.setCursor(0,1);
          lcd.print(F("Success...      "));
          connectedFlag = 1;
          pgmState = 110;
          loopCount = 0;
        }
      }
      else if(!serBusy & !connectedFlag){
        if(newSecond){
          Serial.println("?C"); 
          serBusy = 1;
        }
      }
      if (userButtonState == LOW && !userButtonStateFlag){ // failed to connect - move to state 200 on button press
        userButtonStateFlag = HIGH;
        lcd.clear();    
        pgmState = 125;                                 
        loopCount = 0;
      }
    }      
  }
  
  //***************************************************************************************   
  if (pgmState == 105){  //************************* TRIGGER INVENTORY UPDATE *************
    
    if (loopCount == 0){
      loopCount = 1;
    }
    else {
      if(serReady){
        if(serMsg == 'u'){
          serReady=0;
          pgmState = 110;
          loopCount = 0;
        }
      }
      else if(!serBusy){
        Serial.println("?U");
        serBusy = 1;
      }  
    }      
  }  
  
  
  //***************************************************************************************   
  if (pgmState == 110){  //************************* GET COFFEE QTY ***********************
    
    if (loopCount == 0){
      loopCount = 1;
    }
    else {
      if(serReady){
        if(serMsg == 'n'){
          serReady=0;
          invCnt = atoi(dataBuffer);
          pgmState = 120;
          loopCount = 0;
        }
      }
      else if(!serBusy){
        Serial.println("?N");
        serBusy = 1;
      }  
    }      
  }
  //*************************************************************************************** 
  if (pgmState == 120){ //************************** CHOOSE COFFEE ************************ 
    if (loopCount == 0){
      ctChanged = 1;            
      loopCount = 1;
      lcd.clear();
      lcd.setCursor(3,1);
      lcd.print(F("Select Coffee"));
      lcd.setCursor(2,3);
      lcd.print(char(0x7f));
      lcd.setCursor(5,3);
      lcd.print(char(0x7e));      

    }
    else {
      if(serReady){
        int curPos1;
        if(serMsg == 'i'){
          serReady=0;
          if(ctChanged){
            lcd.setCursor(0,0);
            lcd.print(F("                    "));
            lcd.setCursor(0,1);
            lcd.print(F("                    "));
            lcd.setCursor(int((20-strlen(dataBuffer))/2),1);
            lcd.print(dataBuffer);
            Serial.print("?O,");            //now get the origin
            Serial.println(coffeeType);
            serBusy = 1;
          }

        }
        if(serMsg == 'o'){
          serReady=0;
          lcd.setCursor(0,0);
          lcd.print(F("                    "));
          lcd.setCursor(int((20-strlen(dataBuffer))/2),0);
          lcd.print(dataBuffer);

        }
      }
      if(int delta = checkEnc()){        // do this only when encoder changes 
        ctChanged = 1;
        coffeeType = coffeeType + delta;
        lcd.setCursor(14,3);
        lcd.print(F("SELECT"));
        lcd.setCursor(2,3);
        lcd.print(char(0x7f));
        lcd.setCursor(5,3);
        lcd.print(char(0x7e));       
        
        
        
        
        userButtonEnabled = 1;            //enable userButton
        if (coffeeType > invCnt -1){
          coffeeType = invCnt -1;
          ctChanged = 0;                  //went past end of list -- coffee type did not change
          lcd.setCursor(5,3);
          lcd.print(" ");

        }
        if (coffeeType < 0){
          coffeeType = 0;
          ctChanged = 0;                  //went past end of list -- coffee type did not change
          lcd.setCursor(2,3);
          lcd.print(" ");
        }
        Serial.print("?I,");
        Serial.println(coffeeType);
        serBusy = 1;
      }     
      if (userButtonState == LOW && !userButtonStateFlag){ // detect button press to move to next state
        userButtonStateFlag = HIGH;    
        pgmState = 125;
        loopCount = 0;
      }
    }      
  }
  
  //*************************************************************************************** 
  if (pgmState == 125){ //******************** ENTER GREEN WEIGHT ************************* 
    if (loopCount == 0){
      loopCount = 1;
      lcd.clear();
      lcd.setCursor(1,0);
      lcd.print(F("Enter Green Weight"));
      lcd.setCursor(8,2);
      sprintf(buffer,"%03d g",greenWeight);
      lcd.print(buffer);
      lcd.setCursor(15,3);
      lcd.print(F("START"));
      lcd.setCursor(2,3);
      lcd.print(char(0x7f));
      lcd.setCursor(5,3);
      lcd.print(char(0x7e));
      userButtonEnabled = HIGH;  
    }
    
    if(int delta = checkEnc()){  // do this only when encoder changes
      greenWeight = greenWeight + delta;
      lcd.setCursor(8,2);
      sprintf(buffer,"%03d",greenWeight);
      lcd.print(buffer);
    }     
    
    if (userButtonState == LOW && !userButtonStateFlag){ // detect button press to move to next state
        userButtonStateFlag = HIGH;    //there is a problem with this that needs to be fixed
        if(connectedFlag){
            pgmState = 130;
        }
        else{
            pgmState = 200;
        }
        loopCount = 0;
        roastedWeight = int(greenWeight * 0.85);
        lcd.clear();        
    }
  }      
  
  
  
  //*************************************************************************************** 
  if (pgmState == 130){ //************************ GET ORIGIN *****************************  
    
    if (loopCount == 0){
      loopCount = 1;
      Serial.print("?O,");
      Serial.println(coffeeType);                // NEED TO ADD Ability to handle no connect state troughout 
      serBusy = 1;
    }
    
    else {
      if(serReady){
        if(serMsg == 'o'){
          serReady=0;
          coffeeOrigin = (dataBuffer);
          pgmState = 140;
          loopCount = 0;
        }
      }
    
      //if (1){ // *** Condition to move to next state ***
      //  pgmState = 140;
      //  loopCount = 0;
      //}
    }      
  }
  
    //*************************************************************************************** 
  if (pgmState == 140){  //*********************** GET FARM *********************************  
    if (loopCount == 0){
      loopCount = 1;
      Serial.print("?F,");
      Serial.println(coffeeType);                // NEED TO ADD Ability to handle no connect state troughout 
      serBusy = 1;
    }
    else {
      if(serReady){
        if(serMsg == 'f'){
          serReady=0;
          coffeeFarm = (dataBuffer);
          pgmState = 150;
          loopCount = 0;
        }
      }
    }      
  }
  
  //*************************************************************************************** 
  if (pgmState == 150){  //*********************** GET TIME / DATE ************************
    if (loopCount == 0){
      loopCount = 1;
      Serial.println("?T");
      serBusy = 1;
    }
    else {
      if(serReady){
        if(serMsg == 't'){
          serReady=0;
          dateTime = (dataBuffer);
          pgmState = 200;
          loopCount = 0;
        }
      }
    }      
  }
  
  //*************************************************************************************** 
  if (pgmState == 200){  //****************** WAITING *************************************
    if (loopCount == 0){
      loopCount = 1;
      // **** INIT CODE HERE *** //
    }
    else {
      // *** PROGRAM CODE HERE *** //
      if (1){ // *** Condition to move to next state ***
        pgmState = 300;
        loopCount = 0;
      }
    }      
  }
  
  //*************************************************************************************** 
  if (pgmState == 300){  //****************** ROASTING *************************************
    if (loopCount == 0){
                                                         // start interrupt for heater control
      attachInterrupt(0,zeroCrossingInterrupt, RISING);  //IRQ0 is pin 2. Call zeroCrossingInterrupt on rising signal
      loopCount = 1;
      sprintf(buffer,"?R,%d,%d",int(coffeeType),int(greenWeight));
      Serial.println(buffer); 
      serBusy = 1;
      drumState = HIGH;              //Drum _should_ be running.  Set running flag
      tDrumPosLast = now();          //start timing for drum rotation detection now
      tRoast0 = now();
      t1stCrack0=0;
      t1stCrack=0;
      subState = 0;                  //reset substate to the normal roasting state
      tRoast = now() - tRoast0;
//      setDisplay(pgmState);          //call subroutine to set up display for roasting

      lcd.clear();
      lcd.print(F("100%  *ROAST*  0000s"));
      lcd.setCursor(1,2);
      sprintf(buffer,"Inlet   000/000 %lcF",char(0xdf));
      lcd.print(buffer);
      lcd.setCursor(1,3);
      sprintf(buffer,"Outlet  000/000 %lcF",char(0xdf));
      lcd.print(buffer);


      analogWrite(fanSpd, 255);      //fan on 
      analogWrite(drumSpd,255);      //drum on
      lastUserButtonState = userButtonState;  //this may be unnecessary now
      lastSelectState = selectState; 
      userButtonEnabled= HIGH;  
    }
    else {
      if(serReady){
        if(serMsg == 'r'){
          serReady = 0;
        }
      }
      
      if (now() > tDispReset){//If display timer times out, switch back to subState 0 (normal roasting mode)
         subState =0;
      }      
      
       //Watch for button press
      if (selectState == LOW && subStateToggleLock == 0){                            //button is pressed down
         if (subState==0){                     // During roasting, this is the normal state
           subState = 1;
           subStateToggleLock = 1;
           subStateChange = 1;                // substate has changed, set flag
           userButtonEnabled = 0;
        }
         else if (subState==1){               // This sub-state is used for adjusting the fan speed
           subState = 2;
           subStateToggleLock = 1;
           subStateChange = 1;                // substate has changed, set flag
           userButtonEnabled = 0;
        }
        else if (subState==2){              // This sub-state is used for adjusting the drum speed
          subState = 0;
          subStateToggleLock = 1;
          subStateChange = 1;                // substate has changed, set flag
          userButtonEnabled = 1;
        }
     }     
      
      if (selectState == HIGH){                             //button is no longer pressed down
        subStateToggleLock = 0;
      }
      
      if (t1stCrack0 > 0){           //do this after first crack has happened
        if (t1stCrack < 1){
           T_outSet = T_postCrack;     //set temperature back to post crack level
        }
        t1stCrack = now() - t1stCrack0;
        tCurrent = t1stCrack;
       
        sprintf(buffer,"%03d%%  *CRACK*  %04ds",int(htrPwrPct),tCurrent); 
      }
      else{                         //do this before first crack
        tRoast = now() - tRoast0;
        tCurrent = tRoast;
        sprintf(buffer,"%03d%%  *ROAST*  %04ds",int(htrPwrPct),tCurrent);
      }
      
      if(subStateChange){
        tDispReset = now() + tDispDuration;
        lcd.clear();
      }
      
      if(subStateChange || newSecond){
        if(subState == 0){  // normal display of information during roasting mode
          
          lcd.setCursor(0,0);
          lcd.print(buffer);
          
          sprintf(buffer," Inlet   %03d/%03d %lcF",int(T_in),int(T_inSet),char(0xdf));
          lcd.setCursor(0,2);
          lcd.print(buffer);
          sprintf(buffer," Outlet  %03d/%03d %lcF",int(T_out),int(T_outSet),char(0xdf));
          lcd.setCursor(0,3);
          lcd.print(buffer);          
          
        }
        if(subState == 1){  // display while setting fan speed
          
          lcd.setCursor(3,0);
          lcd.print(F("Set Fan Speed"));
          sprintf(buffer,"%03d%%",fanSpdPct);
          lcd.setCursor(8,2);
          lcd.print(buffer);
          lcd.setCursor(2,3);
          lcd.print(char(0x7f));
          lcd.setCursor(5,3);
          lcd.print(char(0x7e));            
        }   
        if(subState == 2){  // display while setting drum speed
          
          lcd.setCursor(3,0);
          lcd.print(F("Set Drum Speed"));
          sprintf(buffer,"%03d%%",drumSpdPct);
          lcd.setCursor(8,2);
          lcd.print(buffer);
          lcd.setCursor(2,3);
          lcd.print(char(0x7f));
          lcd.setCursor(5,3);
          lcd.print(char(0x7e));           

        }
      }      
      OCR1A = htrPwrCnt;
      fanSpdCnt = (int)((fanSpdPct/100.0)*255);
      drumSpdCnt = (int)((drumSpdPct/100.0)*255);
      analogWrite(fanSpd, fanSpdCnt);       //set fan speed (blue LED)
      analogWrite(drumSpd,drumSpdCnt);       //set drum speed (green LED)
      
      if(int delta = checkEnc()){  // do this only when encoder changes
        if(subState == 0){
          T_outSet += delta*5;
          if (T_outSet > T_outSetMax){
            T_outSet = T_outSetMax;
          }
          if (T_outSet < T_outSetMin){
            T_outSet = T_outSetMin;
          }
          lcd.setCursor(13,3);
          sprintf(buffer,"%03d",int(T_outSet)); 
          lcd.print(buffer);
        }
        
        if(subState == 1){
          fanSpdPct += delta;
          if (fanSpdPct > 100){
            fanSpdPct = 100;
          }
          if (fanSpdPct < fanSpdPctMin){
            fanSpdPct = fanSpdPctMin;
          }
          lcd.setCursor(8,2);
          sprintf(buffer,"%03d",int(fanSpdPct)); 
          lcd.print(buffer);
          tDispReset = now() + tDispDuration;
        }
        
        if(subState == 2){
          drumSpdPct += delta;
          if (drumSpdPct > 100){
            drumSpdPct = 100;
          }
          if (drumSpdPct < drumSpdPctMin){
            drumSpdPct = drumSpdPctMin;
          }          
          lcd.setCursor(8,2);
          sprintf(buffer,"%03d",int(drumSpdPct)); 
          lcd.print(buffer);
          tDispReset = now() + tDispDuration;
        }
      }
      
      if (userButtonState == LOW && !userButtonStateFlag && userButtonEnabled){   
        userButtonStateFlag = HIGH;
        if (t1stCrack0 == 0){     //if first crack has not occured yet, mark 1st crack time
          t1stCrack0 = now();
          lcd.setCursor(6,0);
          lcd.print(F("*CRACK*"));

        }
        else{
          pgmState = 400; // move to next state
          loopCount = 0;
          detachInterrupt(0); //disable heater when not in roasting mode
        }
      }      
    }      
  }  
  
  //*************************************************************************************** 
  if (pgmState == 400){  //****************** COOLING *************************************
    if (loopCount == 0){
      loopCount = 1;
      tCool0=now();
      tCool = now()-tCool0;
      lcd.clear();
      
      sprintf(buffer,"000%%  *COOL*  %04ds",tCool); 
      lcd.setCursor(0,0);
      lcd.print(buffer);
      sprintf(buffer,"Inlet %03d %1cF",int(T_in),char(0xdf));
      lcd.setCursor(5,2);
      lcd.print(buffer);
      sprintf(buffer,"Outlet %03d %1cF",int(T_out),char(0xdf));
      lcd.setCursor(4,3);
      lcd.print(buffer);         
      //OCR1A = 505;                    //obsolete now that we have disabled interrupt outside of roasting mode
      analogWrite(fanSpd, 255);       //fan on (blue LED)
      analogWrite(drumSpd,255);       //drum on (green LED)
      userButtonEnabled = 0;  
    }
    else {
      tCool = now() - tCool0;
      lcd.setCursor(15,0);
      sprintf(buffer,"%04ds",tCool);
      lcd.print(buffer);
      lcd.setCursor(0,1);
      if (lastRotation){
        //sprintf(buffer,"O %03d%1cF  I*%03d%1cF",int(T_out),char(0xdf),int(T_in),char(0xdf));
      }
      else{
        //sprintf(buffer,"O %03d%1cF  I %03d%1cF",int(T_out),char(0xdf),int(T_in),char(0xdf));
      }
      sprintf(buffer,"%03d",int(T_in));
      lcd.setCursor(11,2);
      lcd.print(buffer);
      sprintf(buffer,"%03d",int(T_out));
      lcd.setCursor(11,3);      
      lcd.print(buffer);
      sprintf(buffer,"%04d",int(tCool));
      lcd.setCursor(15,0);      
      lcd.print(buffer);      
      
      if (T_out < T_outCooled && lastRotation == LOW){     //signal end of cooling and do last rotation
        lastRotation = HIGH;
      }
      
      if (selectState == LOW && lastRotation == LOW && !selectStateFlag){   //signal end of cooling and do last rotation
        selectStateFlag == HIGH;                                            //what the hell does this do?
        lastRotation = HIGH;
      }
      
      if (lastRotation == HIGH){          //do this when preparing to stop the drum
        if (drumPosState == LOW){         //if drum is in final position
          lastRotation = LOW;
          analogWrite(drumSpd,0);         //drum off (green LED) 
          pgmState = 500;              // move to next state
          loopCount = 0;
          userButtonEnabled=1;
        }
      }
    }      
  }
  
  //*************************************************************************************** 
  if (pgmState == 500){  //****************** DONE *************************************
    if (loopCount == 0){
      //OCR1A = 505;                  //heater off //obsolete now that we have disabled interrupt outside of roasting mode
      analogWrite(fanSpd, 0);       //fan on (blue LED)
      analogWrite(drumSpd,0);       //drum off (green LED)
      loopCount = 1;
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("Enter Roasted Weight"));
      lcd.setCursor(8,2);
      sprintf(buffer,"%03d g",roastedWeight);
      lcd.print(buffer);
      lcd.setCursor(15,3);
      lcd.print(F("PRINT"));
      lcd.setCursor(2,3);
      lcd.print(char(0x7f));
      lcd.setCursor(5,3);
      lcd.print(char(0x7e));        

    }
    else {
      if(int delta = checkEnc()){  // do this only when encoder changes
        roastedWeight = roastedWeight + delta;
        lcd.setCursor(8,2);
        sprintf(buffer,"%03d",roastedWeight);
        lcd.print(buffer);
      } 
                
      if (userButtonState == LOW && !userButtonStateFlag && userButtonEnabled){
        userButtonStateFlag = HIGH;
        pgmState = 510; // move to next state
        loopCount = 0;
        massLost = (double)100.0 * (greenWeight-roastedWeight)/greenWeight;
        
      }
    }      
  }
  //*************************************************************************************** 
  if (pgmState == 510){  //****************** DONE *************************************
    if (loopCount == 0){
      //OCR1A = 505;                  //heater off//obsolete now that we have disabled interrupt outside of roasting mode
      analogWrite(fanSpd, 0);       //fan on (blue LED)
      analogWrite(drumSpd,0);       //drum off (green LED)
      loopCount = 1;
      lcd.clear();
      lcd.setCursor(0,0);
      sprintf(buffer,"Roast Time: %03ds",int(t1stCrack));
      lcd.print(buffer);
      lcd.setCursor(0,1);
      sprintf(buffer,"Crack Time: %03ds",int(tRoast));
      lcd.print(buffer);
      lcd.setCursor(0,2);
      massLost = (double)100.0 * (greenWeight-roastedWeight)/greenWeight;
      sprintf(buffer,"%03d",int(10*massLost));
      sprintf(buffer,"Mass Lost: %c%c.%c%%",buffer[0],buffer[1],buffer[2]);
      lcd.print(buffer);
      lcd.setCursor(15,3);
      lcd.print(F("ROAST"));
      lcd.setCursor(2,3);
      lcd.print(char(0x7f));
      lcd.setCursor(5,3);
      sprintf(buffer,"?X,%4d,%4d,%4d,%4d",int(tRoast),int(t1stCrack),int(tCool),int(roastedWeight));
      Serial.println(buffer);       
      lcd.print(char(0x7e));
      subState = 0;            //substate is used to select between ROAST and SHUTDOWN
      serBusy = 1;
      
      lcd.setBacklight(0);
      userButtonEnabled = 0;
      digitalWrite(led,HIGH);  //have to manually turn off led becase of printer requirements
//      term.setBacklight(0);
      printReceipt();
      lcd.setBacklight(100);
      userButtonEnabled = 1;
//      term.setBacklight(100);
      
      
    }
    else{
      
//        if (now() > tDispReset){//If display timer times out, switch back to subState 0 (normal roasting mode)
//         if(subState==1){
//            subState =0;
//            subStateChange = 1;
//          }
//        }
      
        if(int delta = checkEnc()){  // do this only when encoder changes
          if(delta > 0){
            subState = 1;
            lcd.setCursor(11,3);
            lcd.print(F("SHUT DOWN"));
          }
          if(delta < 0){
            subState = 0;
            lcd.setCursor(11,3);
            lcd.print(F("    ROAST"));
          }
        } 
      
        //Watch for user button press
        if (userButtonState == LOW && !userButtonStateFlag && userButtonEnabled){
          userButtonStateFlag = HIGH;
          if(subState == 1){           //Shut down
            Serial.println("?Q");      //Send shutdown message to RasPi
            lcd.clear();
            lcd.setCursor(3,1);
            lcd.print(F("Shutting Down"));
            lcd.setCursor(3,2);
            lcd.print(F("Please Wait..."));
            delay(17000);
            lcd.clear();
            lcd.setCursor(4,1);
            lcd.print(F("OK to power"));
            lcd.setCursor(8,2);
            lcd.print(F("down"));            
          }
          
          if(subState == 0){           //Do next roast
            pgmState = 0; // move to next state
            T_outSet = T_outSet0;      // reset outlet temperature setpoint to initial value
            loopCount = 0;
            userButtonEnabled=1;
          }
          
        }            
          
          
//        }
       
     if(serReady){
        if(serMsg == 'x'){
          serReady = 0;
        }
      }
      
//      if (userButtonState == LOW && !userButtonStateFlag){
//        userButtonStateFlag = HIGH;
//        pgmState = 0; // move to next state
//        T_outSet = T_outSet0;      // reset outlet temperature setpoint to initial value
//        loopCount = 0;
//      }
      
    }
  }



  // ** Mandatory Resets Below **
  if (selectStateFlag && selectState){
    selectStateFlag = LOW;
  }
  if (userButtonStateFlag && userButtonState){
    userButtonStateFlag = LOW;
  }
  serReady = 0;
  newSecond = 0;
}  // ** BOTTOM OF MAIN LOOP **   

////////////////// Subroutines and Such /////////////////////////////////

//Interrupt Service Routines
void zeroCrossingInterrupt(){            //zero cross detect   
  TCCR1B=0x04;                           //start timer with divide by 256 input
  TCNT1 = 0;                             //reset timer - count from zero
}

ISR(TIMER1_COMPA_vect){                 //comparator match
  digitalWrite(gate,HIGH);              //set triac gate to high
  TCNT1 = 65536-PULSE;                  //trigger pulse width
}

ISR(TIMER1_OVF_vect){                   //timer1 overflow
  digitalWrite(gate,LOW);               //turn off triac gate
  TCCR1B = 0x00;                        //disable timer to stop unintended triggers...
}

int checkEnc(){
  int delta = 0;
  encValueNow=myEnc.read();                      //read the encoder register
  if(encValueOld/4 != encValueNow/4){            //check to see if it has been moved
    delta = (encValueNow/4 - encValueOld/4);     //determine how much it moved 
    encValueOld = encValueNow;
  }
  return delta;
}

int multiMap(int val, int* _in, int* _out, uint8_t size)
{
  // see http://arduino.cc/playground/Main/MultiMap
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= _in[0]) return _out[0];
  if (val >= _in[size-1]) return _out[size-1];

  // search right interval
  uint8_t pos = 1;  // _in[0] allready tested
  while(val > _in[pos]) pos++;

  // this will handle all exact "points" in the _in array
  if (val == _in[pos]) return _out[pos];

  // interpolate in the right segment for the rest
  return map(val, _in[pos-1], _in[pos], _out[pos-1], _out[pos]);
}


int freeRam () 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}


//incoming message format used here is  {x:Data_Follows_Here}
//x is the message indicator (always lower case)
//: is a required character separator to make messages more "human readable"
//Data_Follows_Here can be any ascii data
//Serial communication scheme based on blog post by jhaskell
//http://jhaskellsblog.blogspot.com/2011/05/serial-comm-fundamentals-on-arduino.html 

boolean getSerialString(){
  while(Serial.available()>0){
    char incomingbyte = Serial.read();
    serBusy = 1;
    if(storeString){
        if(dataBufferIndex==DATABUFFERSIZE){                //don't write past buffer - abort this msg
            dataBufferIndex = 0;
            storeString = false;
            serBusy = 0;
            break;
        }
        if(incomingbyte==endChar){                          //received complete string - finished
            storeString = false;
            serBusy = 0;
            serReady = 1;
            serCntLst = serCnt;
            serCnt = 0;
            return true;
            
        }
        else{                                               //have another byte to process
            if(dataBufferIndex==0){                         //the first char after the start char is the command byte (serMsg)
              serMsg = incomingbyte;
            }
            if(dataBufferIndex>1){                          //throw out the ":" that comes after the command byte
              dataBuffer[dataBufferIndex-2] = incomingbyte; //fill dataBuffer from 0
              dataBuffer[dataBufferIndex-1] = 0;            //assume this is last byte and null terminate the C string
            }
            dataBufferIndex++;
        }
    }
    
    if(incomingbyte==startChar){                      //received start character - valid message to follow
        dataBufferIndex = 0;                          //start storing message at beginning of dataBuffer
        storeString = true;                           //flag that valid message is coming
    }
  } 
  if(serBusy & serCnt < serTO){                      //increment timeout counter
    serCnt ++;
  }
  if(serCnt >= serTO){                              //abort on timeout
    serCnt = 0;
    serErr = 1;
    serBusy = 0;
    dataBufferIndex = 0;
    storeString = false;
  }
  return false;
}

void printReceipt(){
      printer.wake();       // MUST call wake() before printing again, even if reset
      printer.justify('C');
      printer.doubleHeightOn();
      printer.setSize('L');
      printer.println(coffeeOrigin);
      printer.doubleHeightOff();
      printer.setSize('S');
      printer.println(coffeeFarm);
      printer.feed(1);
      printer.println(dateTime);
      printer.feed(1);
      printer.justify('L');
      sprintf(buffer,"%3d g",roastedWeight);
      printer.println(buffer);
      sprintf(buffer,"Rost Time: %4d seconds",int(tRoast));
      printer.println(buffer);
      sprintf(buffer,"1st Crack: %4d seconds",int(t1stCrack));
      printer.println(buffer);
      sprintf(buffer,"%03d",int(10*massLost));
      sprintf(buffer,"Mass Lost:    %c%c.%c%%",buffer[0],buffer[1],buffer[2]);
      printer.println(buffer);
      printer.feed(5);
      printer.sleep();      // Tell printer to sleep   
}
//////////////////////////////// FOLLOWING IS THE COMPATIBLE RASPI SCRIPT ///////////////////////////////////////////////////////
/*



*/
