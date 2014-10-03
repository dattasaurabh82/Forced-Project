#include <EEPROM.h>

uint8_t recordButtonPin = 4;
uint8_t playButtonPin = 7;

#define SAMPLE_DELAY 25 

#define encoderI 2
#define encoderQ 3 // Only using one interrupt in this 

uint16_t count;

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
    Serial.print("Actual count: ");
    Serial.println(count);
  
   //Break and bit shift to left
    uint8_t low = count & 0xFF;
    uint8_t high = (count >> 8) & 0xFF;
  //write bothe the values to the eeprom
    EEPROM.write(addr, low);
    EEPROM.write(addr+1, high);
    addr+=2;
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
    //read bothe the vlues from the eeprom
    uint8_t low = EEPROM.read(addr);
    uint8_t high = EEPROM.read(addr+1);
    //combine them
    uint16_t output = (((low << 0)&0xFF) + ((high << 8)&0xFFFF));

    if (output == 255) break;
    Serial.println(output);
    delay(SAMPLE_DELAY);
    //delay(150);
    addr+=2;
    if (addr == 512) break;
  }
  //if (addr != 512) EEPROM.read(addr);
  Serial.println("Done Spitting");
  delay(250);
  //return();
}








