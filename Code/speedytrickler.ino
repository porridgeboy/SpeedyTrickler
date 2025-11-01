


#include <SerialBT.h>



#include <MobaTools.h>
const byte stepPin = 8;
const byte dirPin = 7;
const byte stepPin2 = 3;
const byte dirPin2 = 2;

MoToStepper stepper1( 6400, STEPDIR );  // big wheel
MoToStepper stepper2( 1600, STEPDIR );  // small wheel
int smallspeed = 450;
int bigspeed = 450;


unsigned long previousMillis = 0; // for interval asking value from balance 
const long interval = 200;        // interval to ask value
const byte numChars = 32;         
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;
float dataNumber = 0;             // new for this version
float stablevalue = 0;             // stable value from scale, updates always when it changes
float f_stablevalue = 0;            // stable value before final throw

float chargemin = 999;                  // value for ED min
float chargemax = 0;                    // value for ED max

const byte numCharsBT = 32;
char receivedCharsBT[numChars];   // an array to store the received data from bluetooth
boolean newDataBT = false;        
float dataNumberBT = 0;             // data number from bluetooth
int   datastatemachine = 0;         // data from datanumberBT converted to int for statemachine

int state =0;                   // state machine for controlling trickler

byte auto_man =0 ;              // tricker auto or manual mode
byte dispense =0 ;              // if 1, starts dispensing 


float targetvalue = 0;          // charge weight

float bigwheelslot = 0.072;         //big wheel amount of powder per slot
float smallwheelslot = 0.0011;         // same as big wheel but for smaller one
int adjustsmallwheel = 0;


int rotationssmall = 0;         //small disk rotations
int rotationsbig = 0;           // big disk rotations

void setup() {

 SerialBT.setName("RUUTIANNOSTELIJA");
  SerialBT.begin();
  Serial1.begin(9600);

   stepper1.attach( stepPin, dirPin );
  stepper1.setSpeed( bigspeed );              // 30 rev/min (if stepsPerRev is set correctly)
  stepper1.setRampLen(20);                 
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);

   stepper2.attach( stepPin2, dirPin2 );
  stepper2.setSpeed(smallspeed );              // 30 rev/min (if stepsPerRev is set correctly)
  stepper2.setRampLen(20);                 
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);

}

void loop() {

  
    askvalue();
    recvWithEndMarker();
    showNewNumber();
    recvWithEndMarkerBT();
    bluetoothdata();
    statemachine();

 }


void recvWithEndMarkerBT() {
    static byte ndx = 0;
     char endMarker = '!';
    char rc;
    
    if (SerialBT.available() > 0) {
        rc = SerialBT.read();

        if (rc != endMarker) {
            receivedCharsBT[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedCharsBT[ndx] = '\0'; // terminate the string
            ndx = 0;
            newDataBT = true;
        }
    }
 }


void bluetoothdata () {
    if (newDataBT == true) {
                   
        dataNumberBT = atof(receivedCharsBT);   
        datastatemachine = (int)dataNumberBT; 
        SerialBT.println(dataNumberBT);   
        SerialBT.println(datastatemachine);  
        newDataBT = false;
        switch (datastatemachine) {
  case 0 ... 8:
  
  if (dataNumberBT > 0.4 && dataNumberBT < 5){

  targetvalue = dataNumberBT;
  SerialBT.print("Charge weight now = ");
   SerialBT.println(dataNumberBT , 3);
  }
  else{
    SerialBT.println("Charge weight too low or high");
  }

  break;
  case 100:
  Serial1.print("!t\r\n");

  break;
  case 101:
    if (auto_man !=0){
      auto_man = 0;
      SerialBT.println("manual mode activated");
         }
         else {
          auto_man = 1;
      SerialBT.println("automatic mode activated");
         }


  break;
  case 102:
  dispense = 1;
 SerialBT.println("Dispensing...");

  break;
  case 103:
  stepper2.setSpeed( smallspeed * 0.5 ); 
 stepper2.move(3200);
  stepper2.setSpeed( smallspeed ); 

  break;

  case 104:
 stepper1.move(6400*2);

  break;
  }
    }
}

void statemachine() {

if (dispense == 1 && stablevalue < 0.2 && stablevalue > -0.2 && state == 0 )

{
state = 1;
}


switch (state) {

   case 1:

     Serial1.print("!t\r\n");
      if (stablevalue == 0)
      {
        state = 2;
      }
    
  break;

  case 2:
 rotationsbig = round((targetvalue / bigwheelslot)-0.5);
 rotationssmall = round((targetvalue - (rotationsbig*bigwheelslot))/ smallwheelslot);
 
stepper2.move(40*rotationssmall);
 stepper1.move(256*rotationsbig);
  

 state = 3;
  break;

  case 3:

if (stablevalue >= targetvalue - 0.002 && stablevalue <= targetvalue + 0.002 )

  { state = 5}

  else {

  if (stablevalue < targetvalue*1.001)

  {

 if (stablevalue > targetvalue*0.8) {

  rotationssmall = round((targetvalue- stablevalue) / smallwheelslot);
 
  stepper2.move(40*(rotationssmall-adjustsmallwheel));

  f_stablevalue = stablevalue;


  state = 4;
   }

   }

   else {
  state = 0;
  SerialBT.println("OVERCHARGE !!");
  
 }
  }

  break;

  case 4:

  if (stablevalue > f_stablevalue + 0.006)

    {state = 5};

  break;


  case 5:



  if (stablevalue >= targetvalue - 0.002 && stablevalue <= targetvalue + 0.002 )

  {
    

    if(stablevalue > chargemax)
    {
      chargemax = stablevalue;
    }

    else if(stablevalue < chargemin) {
      chargemin = stablevalue;
    }

    SerialBT.print("Dispensed: ");
    SerialBT.println(stablevalue);
    SerialBT.print("min: ");
    SerialBT.println(chargemin);
    SerialBT.print("max: ");
    SerialBT.println(chargemax);
  }
  else if (stablevalue < targetvalue - 0.002)
  {
    stepper2.setSpeed( smallspeed * 0.5 ); 
    rotationssmall = round((targetvalue - stablevalue) / smallwheelslot);
    stepper2.move(40*(rotationssmall - adjustsmallwheel));
    stepper2.setSpeed( smallspeed); 
    state = 6;
  }

  break;

  case 6:

  if (stablevalue >= targetvalue - 0.002 && stablevalue <= targetvalue + 0.002 )

 {
    

    if(stablevalue > chargemax)
    {
      chargemax = stablevalue;
    }

    else if(stablevalue < chargemin) {
      chargemin = stablevalue;
    }

    SerialBT.print("Dispensed: ");
    SerialBT.println(stablevalue);
    SerialBT.print("min: ");
    SerialBT.println(chargemin);
    SerialBT.print("max: ");
    SerialBT.println(chargemax);

    state = 0;
    
  }

  else {
    SerialBT.println("OVERCHARGE !!");
    state = 0;
  }

  break;
}


}





void askvalue() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
      Serial1.print("!p\r\n");


    }
  }



void recvWithEndMarker() {
    static byte ndx = 0;
     char endMarker = '\n';
    char rc;
    
    if (Serial1.available() > 0) {
        rc = Serial1.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
 }



void showNewNumber() {
    if (newData == true) {
        dataNumber = 0; 
        
                  
      if (strchr(receivedChars, 'g')) {

        if (strchr(receivedChars, '-')) {
          receivedChars[0] = ' ';
      dataNumber = atof(receivedChars)*-1;

    }
   
    else {
      dataNumber = atof(receivedChars);
    }

        if(dataNumber != stablevalue) {
          stablevalue = dataNumber;
        // SerialBT.print("Data as Number ... ");    // new for this version
        SerialBT.println(stablevalue, 3);     // new for this version
              //  SerialBT.println(receivedChars);     // new for this version

        }
    }   
        newData = false;
 }
 }


