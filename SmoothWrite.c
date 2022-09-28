#include <Servo.h>

Servo myservo;
int sensorPin = A0;
int sensorValue = 0;

void setup() {
 myservo.attach(9); 
 sensorValue = analogRead(sensorPin);
 if (sensorValue > 79)                //Initial angle > 0
   {
     
   }
 else                                 //Initial angle < 0
   {
  
   }
 softwrite(90);
}

void loop() {
  

}

void softwrite(int angle)
{
  int i;
  for(i = 0; i <= angle; i++)
  {
    myservo.write(i);
    delay(30);
}
}
