#include <Servo.h>

Servo myservo;

int sensorPin = A0;    // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor
double voltage = 0;
const int numReadings = 10;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

int inputPin = A0;

void setup() {
  Serial.begin(9600);
  myservo.attach(9);
  myservo.write(0);
  pinMode(ledPin, OUTPUT);
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
//  pinMode(pin7,INPUT);
}

void loop() {

  total = total - readings[readIndex];
  readings[readIndex] = analogRead(inputPin);
  total = total + readings[readIndex];
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) {
    readIndex = 0;
  }
  average = total / numReadings;
  delay(1);        
  sensorValue = analogRead(sensorPin);
  delay(sensorValue);
  voltage = sensorValue * 5.0/1023;  
  delay(sensorValue);
  //Offset calibration for G_servo

  if (voltage>0 && voltage<.75)
    voltage = voltage + .12;
  else if(voltage>.75 && voltage<1.50) 
    voltage = voltage + .11;
  else if(voltage>1.50 && voltage<2.00)
    voltage = voltage + .10;
  
  Serial.println(voltage);
  delay(500);
}
