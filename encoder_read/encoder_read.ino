// Interrupt information
// 0 on pin 2
// 1 on pin 3

#define encoderI 2
#define encoderQ 3 // Only use one interrupt in this example

uint16_t count;

void setup()
{
Serial.begin(115200);
count=0;
pinMode(encoderI, INPUT);
pinMode(encoderQ, INPUT); attachInterrupt(0, handleEncoder, CHANGE);

}

void loop()
{
Serial.println(count);
delay(10);
}

void handleEncoder()
{
if(digitalRead(encoderI) == digitalRead(encoderQ))
{ count++;
}
else
{ count--;
}

}
