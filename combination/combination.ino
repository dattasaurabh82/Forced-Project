#include <EEPROM.h>

uint8_t recordButtonPin = 4;
uint8_t playButtonPin = 12;
//uint8_t ledPin = 13;

#define SAMPLE_DELAY 25 

// Interrupt information
// 0 on pin 2
// 1 on pin 3
#define encoderI 2
#define encoderQ 3 // Only using one interrupt in this 

volatile int count;

void setup(){
  Serial.begin(115200);

  count=0;

  pinMode(encoderI, INPUT);
  pinMode(encoderQ, INPUT); 
  attachInterrupt(0, handleEncoder, CHANGE);


  pinMode(recordButtonPin, INPUT);
  digitalWrite(recordButtonPin, HIGH);
  pinMode(playButtonPin, INPUT);
  digitalWrite(playButtonPin, HIGH);
  //pinMode(ledPin, OUTPUT);

  Serial.println("Linear Encoder recorder/player");


}

void loop(){
  // Serial.println(count);
  // delay(10);

  if (! digitalRead(recordButtonPin)) {
    delay(10);
    // wait for released
    while (! digitalRead(recordButtonPin));
    delay(20);
    // OK released!
    recordEncoder();
  }

  if (! digitalRead(playButtonPin)) {
    delay(10);
    // wait for released
    while (! digitalRead(playButtonPin));
    delay(20);
    // OK released!
    spitEncoder();
  }
}




void handleEncoder(){
  if(digitalRead(encoderI) == digitalRead(encoderQ))
  { 
    count++;
  }
  else{
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
    volatile int mappedCount = map(constrainedCount, 0, 4000, 0, 255);

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

   //if (x == 512) break;
    if (x == 255) break;
    Serial.println(x);
    delay(SAMPLE_DELAY);
    addr++;
    if (addr == 512) break;
  }
  //if (addr != 512) EEPROM.read(addr);
  Serial.println("Done Spitting");
  delay(250);
  //return();
}
