#include <Servo.h>

Servo myservo;

double voltageArr[95];
int i=0, j=0;
boolean flag = true;
//{.4, .43, .46, .49, .52, .56, .58, .61, .64, .67, .7, .72, .75, .78, .81, .84, .87, .9,
//.92, .94, .98, 1.02, 1.05, 1.08, 1.11, 1.115, 1.145, 1.175, 1.205, 1.235, 1.255, 1.295, 1.315, 1.345, 1.375, 
//1.405, 1.435, 1.47, 1.495, 1.535, 1.565, 1.575, 1.605, 1.635, 1.675, 1.705, 1.735, 1.755, 1.785, 1.8, 1.81};
//
//int degreeArr[] = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48,
//50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 100};

const int numReadings = 10;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0, Total = 0;       // the running total
int average = 0;                // the average
int degree;
int inputPin = A0;
int ReadAdc = 0; 
double Voltage = 0;

int ReadAvg(void);

void setup() {
  myservo.attach(9);
//  myservo.write(90);
  Serial.begin(9600);
  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
}

void loop() {
// if(flag == true)
// {
//  for(i=0;i<15;i++)  
//  {
//    Total = AvgRd();
//  }
  
//  Voltage = AvgRd() * 5.00/1023;          //It is really important to set arduino voltage to the exact input voltage 5v
                                            //otherwise there is varriation in measurment.(varry the voltage of input in ...
                                            //arduino to set the voltage measured in multimeter with ADC.
  
    if (Voltage>0 && Voltage<.75)
      Voltage = Voltage + .12;
    else if(Voltage>.75 && Voltage<1.50) 
      Voltage = Voltage + .11;
    else if(Voltage>1.50 && Voltage<2.00)
      Voltage = Voltage + .10;
//    voltageArr[j] = Voltage;
//    delay(500);        // delay in between reads for stability
    Serial.print(voltageArr[j]);
    Serial.println();
//  flag = false;
// }
// myservo.write(30);
// for(i=0;i<96;i++)
// {
   
// }
 
}

int AvgRd(void){
//  inputPin = ReadAdc;
  for (i=0;i<16;i++)
  {
  total = total - readings[readIndex];
  readings[readIndex] = analogRead(inputPin);
  total = total + readings[readIndex];
  readIndex = readIndex + 1;
    if (readIndex >= numReadings) {
      readIndex = 0;
    }
  delay(1);        
  }
  average = total / numReadings;
  Voltage = average * 5.00/1023;
  Serial.println(Voltage);
  delay(500);
  return Voltage;
}
