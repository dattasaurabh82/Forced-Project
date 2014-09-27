#define frontstop = 100            // Right most encoder boundary
#define backstop = 2000            // Left most encoder boundary


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

const int motorPinA = 12;
const int motorPinB = 13;
const int spdPin = 5;              //PWM pin for spd control fropm Pro mini's PWM pin 5 to L293D's enable pin. 


void setup() {
  Serial.begin(115200);
  Serial.println("Linear Encoder Test");




  attachInterrupt(0, doEncoder1, CHANGE);  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(1, doEncoder2, CHANGE);  // encoder pin on interrupt 1 (pin 3)



  randomSeed(analogRead(0));

  pinMode(motorPinA, OUTPUT);
  pinMode(motorPinB, OUTPUT);
  pinMode(spdPin, OUTPUT);

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


  if(posrchd) {                           // If target has been reached clear flag, and get new target
    newpos =  random(200,1800);
    posrchd = 0;
  }    

  posrchd = go_to_target(newpos);

}




/***************************************************************************************
 * reading encoder by direct port manipulation for faster results
 * 
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
 * go_to_target() determines the distance and direction from current position to target 
 * position, then maps speed to decellerate close to the target so as not to overshoot.
 ***************************************************************************************/


int go_to_target(int target)
{
  int temp = 0;
  int delta = abs(Pos1-target);                   // Distance to target
  spd = map(delta,3600,0,255,150);                // Decellerate as you get closer
  if(target < 3600 && target > 100) {
    if(Pos1 < target) {
      runForward();
      //setting Speed; 
      analogWrite(spdPin, spd);
      temp = 0;
    } 
    else if(Pos1 > target) {
      runBackward();
      //setting Speed;
      analogWrite(spdPin, spd);
      temp = 0;
    }  
    else temp =1;
  }
  return temp;
}


/***************************************************************************************
 * Motor functions
 ****************************************************************************************/

void runForward(){
  digitalWrite(motorPinA, HIGH);
  digitalWrite(motorPinB, LOW);

}

void runBackward(){
  digitalWrite(motorPinA, LOW);
  digitalWrite(motorPinB, HIGH);

}



