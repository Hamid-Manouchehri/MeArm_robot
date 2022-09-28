#include <Servo.h>

Servo counter;

int pin7 = 7, push=0;
boolean buttonState = LOW;

void setup()
{
  Serial.begin(9600);
  pinMode(pin7,INPUT);
  counter.attach(9);
}

boolean debounceButton(boolean state)
{
  boolean stateNow = digitalRead(pin7);
  if(state!=stateNow)
  {
    delay(10);
    stateNow = digitalRead(pin7);
  }
  return stateNow; 
}

void loop()
{
  int reading = digitalRead(pin7);
  if (reading == LOW && debounceButton(buttonState) == HIGH )
     {
      counter.write(push);
      Serial.print("degree: ");
      Serial.println(push);
      push ++;
      buttonState = HIGH;   
     }
}
