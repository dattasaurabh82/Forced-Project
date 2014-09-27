#include <EEPROM.h>

uint8_t recordButtonPin = 4;
uint8_t playButtonPin = 12;

const int MotorPinA = 10;
const int MotorPinB = 11;
const int enablePin = 5;

#define SAMPLE_DELAY 25 

#define encoderI 2
#define encoderQ 3 // Only using one interrupt in this 

volatile int count = 0;
volatile int OldPos = 255; 

void setup(){
  Serial.begin(115200);

  //setup counter top zero everytime it starts
  count=0;

  /*********************************************
   * ENCODER SETUP
   ********************************************/
  pinMode(encoderI, INPUT);
  pinMode(encoderQ, INPUT); 
  attachInterrupt(0, handleEncoder, CHANGE);


  /********************************************
   * BUTTON PIN SETUP
   ********************************************/
  pinMode(recordButtonPin, INPUT);
  digitalWrite(recordButtonPin, HIGH);
  pinMode(playButtonPin, INPUT);
  digitalWrite(playButtonPin, HIGH);


  /********************************************
   * MOTOR PINS SETUP
   ********************************************/
  pinMode(MotorPinA, OUTPUT);
  pinMode(MotorPinB, OUTPUT);
  pinMode(enablePin, OUTPUT);

  analogWrite(MotorPinA, 0);
  analogWrite(MotorPinB, 0);
  digitalWrite(enablePin, HIGH);


  Serial.println("Linear Encoder recorder/player");
}

void loop(){
  /*********************************************************
   * DEBUGGING RAW PRINTHEAD POSITION VALUE FROM INTERRUPTS
   *******************************************************/
  // Serial.println(count);
  // delay(10);


  //* WHEN "RECORD" BUTTION IS PRESSED
  if (! digitalRead(recordButtonPin)) {
    delay(10);
    // wait for released
    while (! digitalRead(recordButtonPin));
    delay(20);
    // OK released!
    recordEncoder(); //CALLING THE RECORD FUNCTION TO START WRITING ENCODER POSITION VALUE TO THE EEPROM
  }

  //* WHEN "RECORD" BUTTION IS PRESSED
  if (! digitalRead(playButtonPin)) {
    delay(10);
    // wait for released
    while (! digitalRead(playButtonPin));
    delay(20);
    // OK released!
    spitEncoder(); //CALLING THE PLAY FUNCTION TO START READING ENCODER POSITION VALUE FROM THE EEPROM
    //AND MOVE THE MOTOR TO THOSE POSITIONS
  }
}


/********************************************************
 * FUNCTION FOR COUNTING THE POSITIONS FROM ENCODERS
 *******************************************************/
void handleEncoder(){
  if(digitalRead(encoderI) == digitalRead(encoderQ))// IF BOTH ENCODER VALUES MATCH, POSITION REACHED - INCREMENT
  { 
    count++;
  }
  else{  // ELSE DECREMENT THE COUNTER
    count--;
  }
}



void recordEncoder(){
  uint16_t addr = 0;
  Serial.println("Recording Position");


  while (digitalRead(recordButtonPin)){
    // Serial.print("Actual count: ");
    //Serial.println(count);

    volatile int constrainedCount = constrain(count, 0, 3600);
    volatile int mappedCount = map(constrainedCount, 0, 3000, 0, 255);

    Serial.print("mapped count: ");
    Serial.println(mappedCount);

    EEPROM.write(addr, mappedCount);
    addr++;
    if (addr == 512) break;
    delay(SAMPLE_DELAY);
  }
  if (addr != 512) EEPROM.write(addr, 255);

  //digitalWrite(ledPin, LOW);

  Serial.println("Done Recording");
  delay(250);
  // return();
}



void spitEncoder(){
  uint16_t addr = 0;
  Serial.println("Spitting what you just recorded");

  while (digitalRead(playButtonPin)){
    volatile int x = EEPROM.read(addr);
    //volatile int Pos = map(x, 0, 255, 0, 3600);
    volatile int Pos = x;

    volatile int spd = OldPos - Pos;
    volatile int _spd = min(abs(spd), 255);
    if(_spd < 0){
      //MOVE MOTOR IN ONE DIRECTION WITH A TORQUE
      analogWrite(MotorPinA, _spd);
      analogWrite(MotorPinB, 0);
    }
    else{
      //MOVE MOTOR IN ANOTHER DIRECTION WITH A TORQUE
      analogWrite(MotorPinA, 0);
      analogWrite(MotorPinB, _spd);
    }

    OldPos = Pos;

    if (x == 255) break;
    Serial.println(Pos);
    delay(SAMPLE_DELAY);
    //delay(150);
    addr++;
    if (addr == 512) break;
  }
  //if (addr != 512) EEPROM.read(addr);
  Serial.println("Done Spitting");
  delay(250);
  //return();
}








