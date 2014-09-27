              
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

#define frontstop = 100            // Right most encoder boundary
#define backstop = 3600            // Left most encoder boundary


// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);


const int encoder1PinA = 2;        // X-AXIS  encoder 1 on pins 2 and 4
const int encoder1PinB = 4;
volatile int encoder1Pos = 0;

const int encoder2PinA = 3;        // Y-AXIS  encoder 2 on pins 3 and 5
const int encoder2PinB = 5;
volatile int encoder2Pos = 0;

boolean CarriageDir = 0;           // Carriage Direction '0' is Right to left
byte spd = 220;                    // Carriage speed from 0-255
int newpos = 0;                    // Taget position for carriage
int posrchd = 1;                   // Flag for target reached

int Pos1, Pos2;


void setup() {
  Serial.begin(115200);
  Serial.println("Linear Encoder Test  04-29-2014");

  AFMS.begin();  // Set up Motors
  
  myMotor->run(BACKWARD);        // Bring carriage to home position. 
  myMotor->setSpeed(spd); 
  delay(100); 
  myMotor->run(FORWARD);        // Bring carriage to home position. 
  myMotor->setSpeed(0); 
  
  attachInterrupt(0, doEncoder1, CHANGE);  // encoder pin on interrupt 0 (pin 2)
  // attachInterrupt(1, doEncoder2, CHANGE);  // encoder pin on interrupt 1 (pin 3)
  
  randomSeed(analogRead(0));
}

void loop() {
static int oldPos1, oldPos2;
uint8_t oldSREG = SREG;

uint8_t i;

  cli();
  Pos1 = encoder1Pos;  
  Pos2 = encoder2Pos;
  SREG = oldSREG;
  
  if(Pos1 != oldPos1){
     Serial.print("Encoder 1=");
     Serial.println(Pos1,DEC);
     oldPos1 = Pos1;
  }
  if(Pos2 != oldPos2){
     Serial.print("Encoder 2=");
     Serial.println(Pos2,DEC);
     oldPos2 = Pos2;
  }  
  

  //sweep_carriage();
  
  if(posrchd) {                           // If target has been reached clear flag, and get new target
    newpos =  random(200,3500);
    posrchd = 0;
  }    
    
  posrchd = go_to_target(newpos);
  
}


/***************************************************************************************
The following code was taken from   http://forum.arduino.cc/index.php?topic=41615.20;wap2
to utilize the fast port based encoder logic.  Thank you Lefty!
please go there for a full explanation of how this works.  I have truncated the comments 
here for brevity.
***************************************************************************************/
void doEncoder1() {                                  // ************** X- AXIS ****************
    if (PIND & 0x04) {                              // test for a low-to-high interrupt on channel A, 
        if ( !(PIND & 0x10)) {                      // check channel B for which way encoder turned, 
           encoder1Pos = ++encoder1Pos;               // CW rotation
           PORTD = PIND | 0x40;                     // set direction output pin to 1 = forward, 
          }
        else {
           encoder1Pos = --encoder1Pos;               // CCW rotation
           PORTD =PIND & 0xBF;                      // Set direction output pin to 0 = reverse, 
          }
    }
    else {                                          // it was a high-to-low interrupt on channel A
        if (PIND & 0x10) {                          // check channel B for which way encoder turned, 
           encoder1Pos = ++encoder1Pos;               // CW rotation
           PORTD = PIND | 0x40;                     // Set direction output pin to 1 = forward, 
           }
        else {
           encoder1Pos = --encoder1Pos;               // CCW rotation
           PORTD =PIND & 0xBF;                      // Set direction output pin to 0 = reverse, 
           }
         }
    PORTD = PIND | 0x80;                            //  digitalWrite(encoderstep, HIGH);   generate step pulse high
    PORTD = PIND | 0x80;                            //  digitalWrite(encoderstep, HIGH);   add a small delay
    PORTD = PIND & 0x7F;                            //  digitalWrite(encoderstep, LOW);    reset step pulse
}                                                   // End of interrupt code for encoder #1
                                                   
void doEncoder2(){                                  // ************** X- AXIS ****************
  if (PIND & 0x08) {                                // test for a low-to-high interrupt on channel A, 
     if (!(PIND & 0x20)) {                          // check channel B for which way encoder turned, 
      encoder2Pos = ++encoder2Pos;                  // CW rotation
      PORTB = PINB | 0x01;                          // Set direction output pin to 1 = forward, 
     }
     else {
      encoder2Pos = --encoder2Pos;                  // CCW rotation
      PORTD =PIND & 0xFE;                           // Set direction output pin to 0 = reverse, 
     }
  }
  else {                                            // it was a high-to-low interrupt on channel A
     if (PIND & 0x20) {                             // check channel B for which way encoder turned, 
      encoder2Pos = ++encoder2Pos;                  // CW rotation
      PORTB = PINB | 0x01;                          // Set direction output pin to 1 = forward, 
      }
     else {
      encoder2Pos = --encoder2Pos;                  // CCW rotation
      PORTB =PINB & 0xFE;                           // Set direction output pin to 0 = reverse, 
     }
  }
  PORTB = PINB | 0x02;                              // digitalWrite(encoder2step, HIGH);   generate step pulse high
  PORTB = PINB | 0x02;                              // digitalWrite(encoder2step, HIGH);   used to add a small delay
  PORTB = PINB & 0xFD;                              // digitalWrite(encoder2step, LOW);    reset step pulse
}                                                   // End of interrupt code for encoder #2



/***************************************************************************************
go_to_target() determines the distance and direction from current position to target 
position, then maps speed to decellerate close to the target so as not to overshoot.
***************************************************************************************/


int go_to_target(int target)
{
  int temp = 0;
  int delta = abs(Pos1-target);                   // Distance to target
  spd = map(delta,3600,0,255,150);                // Decellerate as you get closer
  if(target < 3600 && target > 100) {
     if(Pos1 < target) {
       myMotor->run(FORWARD);
       myMotor->setSpeed(spd); 
       temp = 0;
     } else if(Pos1 > target) {
       myMotor->run(BACKWARD);
       myMotor->setSpeed(spd); 
       temp = 0;
     }  else temp =1;
  }
  return temp;
}


/***************************************************************************************
sweep_carriage() is just a test routine to track back and forth testing overshoot. 
I will likely remove it soon.
***************************************************************************************/

void sweep_carriage()
{
    if(CarriageDir == 0) {              // Carriage Moving Right to Left
    if (Pos1 < 3600) {      
      myMotor->run(FORWARD);
      if (Pos1 > 3400) { 
       myMotor->setSpeed(spd-80); 
      } else myMotor->setSpeed(spd);  
    } else {      
      myMotor->setSpeed(0);  
      CarriageDir = !CarriageDir;  
    }
  } else {                              // Carriage Moving Left to Right
    if (Pos1 > 100) {
      myMotor->run(BACKWARD);
      if (Pos1 < 300) { 
       myMotor->setSpeed(spd-80); 
      } else myMotor->setSpeed(spd);  
     } else {      
      myMotor->setSpeed(0);  
      CarriageDir = !CarriageDir;  
    }
  }
}

