// Interrupt information
#define encoderI 2 // 0 on pin 2
#define encoderQ 3 // 1 on pin 3
// Only use one interrupt in this example
uint16_t count; //counter

const int sensorPin = A0;

//uint16_t Position = 0;
int Position = 0;
int newPosition = 1022;

int spd, _spd;


const int MotorPinA = 10;
const int MotorPinB = 11;


void setup()
{
  Serial.begin(115200);
  count=0;
  pinMode(encoderI, INPUT);
  pinMode(encoderQ, INPUT); 
  attachInterrupt(0, handleEncoder, CHANGE);

  pinMode(sensorPin, INPUT);

  pinMode(MotorPinA, OUTPUT);
  pinMode(MotorPinB, OUTPUT);

  analogWrite(MotorPinA, 0);
  analogWrite(MotorPinB, 0);
}

void loop()
{
  int countn = constrain(count, 0, 3600);
  Position = map(countn, 0, 3600, 0, 1024);
  // Serial.println(Position);
  // delay(10);
  newPosition = analogRead(sensorPin);

  spd = newPosition - Position;
  _spd = (min(abs(spd), 255));

  if(spd < 0){
    
    
    analogWrite(MotorPinA, _spd);
    analogWrite(MotorPinB, 0);
  }
  else{
    
    analogWrite(MotorPinA, 0);
    analogWrite(MotorPinB, _spd);
  }
}

void handleEncoder()
{
  if(digitalRead(encoderI) == digitalRead(encoderQ))
  { 
    count++;
  }
  else
  { 
    count--;
  }

}






