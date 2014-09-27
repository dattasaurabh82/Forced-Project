
const int motorPinA = 10;
const int motorPinB = 11;

void setup(){
  pinMode(motorPinA, OUTPUT);
  pinMode(motorPinB, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
}

void loop(){
  for(int i = 0; i<= 255; i=i+50){
    delay(500);
    analogWrite(motorPinA, i);
    analogWrite(motorPinB, 0);
  }
  delay(500);
  for(int j = 0; j<= 255; j=j+50){
    delay(500);
    analogWrite(motorPinA, 0);
    analogWrite(motorPinB, j);
  }
   delay(500);
}


