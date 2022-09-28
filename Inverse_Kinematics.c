#include <Servo.h>
#include <math.h>

#define ChangeTo_Degree  180/PI
#define ChangeTo_Radian  PI/180

#define FirstPosOfB      80
#define FirstPosOfL      50
#define FirstPosOfU      20

#define extensionDelayB  5  
#define extensionDelayL  20 
#define extensionDelayU  20 
#define retractionDelayB 10
#define retractionDelayL 20
#define retractionDelayU 20

//input Cartesian doordinations: 
//#define x 0
//#define y 0
//#define z 0

Servo baseServo, jawServo, lowerServo, upperServo;

float        x=0, y=0, z=0;//100.098, 78.338,130.137           //Forward kinematic paramiters(Known)     
float        Teta, L, U;                                       //Inverse kinematic parameters(Variable) [===> Teta: angle of base body
float        xTo2, yTo2, r;                                    //                                       |===> L   : angle of Lower servo based upon vertical axis
float        temp1, temp2;                                     //                                       [===> U   : angle of Upper servo based upon vertical axis
int          pos;
volatile int flag = 0;
int          FinalPosOfJ, FinalPosOfL, FinalPosOfU, FinalPosOfB;
int          lower_finalDegree = 0, upper_finalDegree = 0;  
char         jaw, base, lower, upper;
boolean      limitation_1 = false, limitation_2 = true, limitation_3 = false, flag_1 = true, flag_3 = true;
byte         i = 3, flag_2 = 3;
int          val;
char         b1;
float        previous[3] = {0}, current[3] = {0};

const int buttonPin_2 = 2, buttonPin_3 = 3;
int buttonState_2, buttonState_3;
int lastButtonState_2 = LOW, lastButtonState_3 = LOW;
unsigned long lastDebounceTime_2 = 0, lastDebounceTime_3 = 0;
unsigned long debounceDelay = 50;

void setup() {
       Serial.begin(115200);

       pinMode(buttonPin_2,INPUT);
       pinMode(buttonPin_3,INPUT);
  
   lowerServo.attach(5);
   upperServo.attach(6);
     jawServo.attach(9);
    baseServo.attach(10);

    baseServo.write(FirstPosOfB);
    delay(1000);
    lowerServo.write(FirstPosOfL);
    delay(1000);
    upperServo.write(FirstPosOfU);
    delay(1000);
     jawServo.write(0);
    delay(1000);
}
                                                      //         \\
                                 //******************(  main Loop )******************\\
                                                      \\         //
void loop() {
   int reading_2 = digitalRead(buttonPin_2);
   int reading_3 = digitalRead(buttonPin_3);

   if(reading_2 != lastButtonState_2 ){
       lastDebounceTime_2 = millis();

   }
   if(reading_3 != lastButtonState_3){
       lastDebounceTime_3 = millis();
       
   }
   if ((millis() - lastDebounceTime_2) > debounceDelay) {
      if (reading_2 != buttonState_2) {
           buttonState_2 = reading_2;
           if (buttonState_2 == HIGH) {
                  flag = 1;
           }
      }
   }
   
   if ((millis() - lastDebounceTime_3) > debounceDelay) {
      if (reading_3 != buttonState_3) {
           buttonState_3 = reading_3;
           if (buttonState_3 == HIGH) {
                  flag = 2;
           }
      }
   }
   
   while(i){//************************( read coordinations over USART )******************
    switch(i){
      case 3:
           Serial.println("enter x: ");
           break;
      case 2:
           Serial.println("enter y: ");
           break;
      case 1:
           Serial.println("enter z: ");
           break;
      } 
      
    while (!Serial.available()) {}
    char b0 = Serial.read();             //sign or sadgan
    if (b0 == 45) {                      // '45' is the ASCII code of '-'
      while (!Serial.available()) {}
      b1 = Serial.read();                //sadgan if b0 is sign
      b1 = b1 - 48;                   
    }
    else 
      b0 = b0 - 48;
      
    while (!Serial.available()) {}
    char b2 = Serial.read();             //dahgan
    b2 = b2 - 48;

    while (!Serial.available()) {}
    char b3 = Serial.read();             //yekan
    b3 = b3 - 48;
    
    if (b0 == 45) {
      val = -(b3 + b2 * 10 + b1 * 100);
    }
    else
      val = b3 + b2 * 10 + b0 * 100 ;
        
    if (flag_2 == 3)
       x = val;
      
    else if (flag_2 == 2)
       y = val;
      
    else if (flag_2 == 1)
       z = val;      
       
    Serial.print(val);
    Serial.println();
      i--;
      flag_2--;   
   }
   if (flag == 1){//**************************( flag = 1 )***********************************
       xTo2 = pow(x,2), yTo2 = pow(y,2);
          r = sqrt(xTo2+yTo2);            
      temp1 = 2*atan2((53-z),(r-63))*ChangeTo_Degree;
      temp2 = 2*asin((53-z)/(160*sin(temp1*ChangeTo_Radian/2)))*ChangeTo_Degree;//input of triangular functions is in Radian.
    
       Teta = atan2(y,x)*ChangeTo_Degree;          //baseServo  angle
          L = .5*(temp2+temp1);                    //lowerServo angle
          U = .5*(temp2-temp1);                    //upperServo angle      
          
//*****************************( limitations of mechanical constraints )*******************                        

      if (L+U >= 60 && L+U <= 210)                 //avoid upperArm and lowerArm collision. 
          limitation_1 = true;
      else    
          Serial.println("Warning: collision of upperArm to lowerArm :(");    
      if (L <= 20 && U > 110){                     //avoid upperArm to behind body plate collision.
          limitation_2 = false;
          Serial.println("Warning: collision of upperArm to body :(");
      }
        if(L <= 100 && U <= 130)                   //avoid upperArm and lowerArm exceed their maximum degree.
          limitation_3 = true;
        else
          Serial.println("Warning: Upper or Lower ARM exceeds its maximum degree :(");
           
//****************************************************************************************     
//      Serial.print("    r: ");
//      Serial.println(r);
//      Serial.print("temp1: ");
//      Serial.println(temp1);
//      Serial.print("temp2: ");
//      Serial.println(temp2);
      Serial.print(" Teta: ");
      Serial.println(Teta);
      Serial.print("    L: ");
      Serial.println(L);
      Serial.print("    U: ");
      Serial.println(U);
//      Serial.println(x);
//      Serial.println(y);
//      Serial.println(z);
      Serial.println("\n");

      if(limitation_1 && limitation_2 && limitation_3){
              current[1] = Teta, current[2] = L, current[3] = U;
              if(flag_3){
                 previous[1] = FirstPosOfB, previous[2] = FirstPosOfL, previous[3] = FirstPosOfU;
                 flag_3 = false;
              }
              comparison(previous,current);
              previous[1] = Teta, previous[2] = L, previous[3] = U;
           }
      flag = 3;
      i = 3;
      limitation_1 = false;
      limitation_2 = true;
      limitation_3 = false;
   }
   lastButtonState_2 = reading_2;
   lastButtonState_3 = reading_3;
   flag_2 = 3;
}
                      //********************( extend function Definition )***********************
                      
int extend( String servoArm, int del, int Initial_degree, int Final_degree){
  
  int delayBetweenEachDegree = del;
  Servo call_servo;
     
  if (servoArm == "jaw") 
     call_servo = jawServo;
  else if(servoArm == "base"){
     call_servo = baseServo;
     if(Final_degree < Initial_degree){
       retract( "base", retractionDelayB, Initial_degree, Final_degree);
     }
  }
  else if(servoArm == "lower"){
     call_servo = lowerServo;
     if(Final_degree < Initial_degree){
       retract( "lower", retractionDelayL, Initial_degree, Final_degree);
     }
  }
  else if(servoArm == "upper"){
     call_servo = upperServo;
     if(Final_degree < Initial_degree){
       retract( "upper", retractionDelayU, Initial_degree, Final_degree);
     }
  }
     for (pos = Initial_degree; pos <= Final_degree; pos++) { 
       call_servo.write(pos);
       delay(delayBetweenEachDegree);                    //pay attention, the argument is in milisecond.                      
     }
     
  return Final_degree;
}

                      //*******************( retract function Definition )*********************

void retract( String servoArm, int del, int Final_degree, int Initial_degree){
  int delayBetweenEachDegree = del;
  Servo call_servo;

  if (servoArm == "jaw") 
     call_servo = jawServo;
  else if(servoArm == "base"){
     call_servo = baseServo;
     if(Final_degree < Initial_degree){
       extend( "base", retractionDelayB, Final_degree, Initial_degree);
     }
  }
  else if(servoArm == "lower"){
     call_servo = lowerServo;
     if(Final_degree < Initial_degree){
       extend( "lower", retractionDelayL, Final_degree, Initial_degree);
     }
  }
  else if(servoArm == "upper"){
     call_servo = upperServo;
     if(Final_degree < Initial_degree){
       extend( "upper", retractionDelayU, Final_degree, Initial_degree);
     }
  }  
  for (pos = Final_degree; pos >= Initial_degree; pos--) { 
    call_servo.write(pos);              
    delay(delayBetweenEachDegree);                       
  }
}

void comparison(float Previous[], float Current[]){
  
     if (Current[1] >= Previous[1]){
        extend("base", extensionDelayB, Previous[1], Current[1]);
     }
     else
        retract( "base", retractionDelayB, Previous[1], Current[1]);
         
     if (Current[2] >= Previous[2]){
        extend("lower", extensionDelayB, Previous[2], Current[2]);
     }
     else
        retract( "lower", retractionDelayB, Previous[2], Current[2]);
        
     if (Current[3] >= Previous[3]){
        extend("upper", extensionDelayB, Previous[3], Current[3]);
     }
     else
        retract( "upper", retractionDelayB, Previous[3], Current[3]);
}
