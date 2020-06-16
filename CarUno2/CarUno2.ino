#include <SoftwareSerial.h>
//Final Code.......
#include <Servo.h> 
//////////////////////////////////////////Defining constants for car ports//////////////////////////////////////////
#define enA 5
#define in1 4
#define in2 9
#define enB 6
#define in3 10 
#define in4 7 
#define servoPin 11 
#define statPin 8 
//////////////////////////////////////////Defining constants for Ultra Sonic Sensor//////////////////////////////////////////
#define trigger 12
#define echo 13
#define triggerSer
#define echoSer
#define Buzzer A0
char FromWhere='F';
//////////////////////////////////////////UltraSonic sensor Vars//////////////////////////////////////////
long time_taken ;
long dist;
long cm;
//////////////////////////////////////////Motors speed//////////////////////////////////////////
int motorSpeedA = 0;
int motorSpeedB = 0;
int yAxis=0;
int xAxis=0;
int servoAngle=90;
int Delay;
int en =0 ; // the enable of the self driving car
int Status = 0;
int count =0;
int error=0;
int maximum;
int timer=0;
bool go=false;
int preTimer=0;
Servo Servo1;
enum ModesOfOperation
{
Mode_Default,
Mode_SelfDriving  
};
ModesOfOperation NosaCar =Mode_Default;
//////////////////////////////////////////RX recieved data//////////////////////////////////////////
int data[2];
SoftwareSerial BTSerial(2, 3);
//////////////////////////////////////////Setup Function//////////////////////////////////////////
void setup() {

 
//////////////////////////////////////////Motor ports setup//////////////////////////////////////////        
      Servo1.attach(servoPin);
      pinMode(statPin,INPUT); 
      pinMode(enA, OUTPUT);
      pinMode(enB, OUTPUT);
      pinMode(in1, OUTPUT);
      pinMode(in2, OUTPUT);
      pinMode(in3, OUTPUT);
      pinMode(in4, OUTPUT);
//////////////////////////////////////////UltraSonic sensor pins setup//////////////////////////////////////////
      pinMode(trigger, OUTPUT);
      pinMode(echo, INPUT);
      Serial.begin(9600);
      BTSerial.begin(38400);
}
//////////////////////////////////////////Loop Function//////////////////////////////////////////
void loop() {
//////////////////////////////////////////Serial printing of the data array//////////////////////////////////////////
 //preTimer=millis();

 //do
 //{
//////////////////////////////////////////Recieving the Accelerometer data//////////////////////////////////////////
if(BTSerial.available() > 0)
 { 
   data[0]=BTSerial.read();
   data[1]=BTSerial.read();
 }

   Serial.print("yAxis F/B : ");
   Serial.print(data[0]);
   Serial.print("xAxis R/L : ");
   Serial.print(data[1]);
   Serial.print(" Motor Speed: ");
   Serial.print(motorSpeedA);
   Serial.println();
  
if(data[0]==255&&data[1]==255)
{
 yAxis=126;
 xAxis=126;
  Serial.print(" ModeOut ");       
 analogWrite(enA, 0 ); // Send PWM signal to motor A    
 analogWrite(enB,  0); // Send PWM signal to motor B 
 if(NosaCar==Mode_Default)
    {NosaCar=Mode_SelfDriving ;
      Serial.print(" SELFDRIVE ");  
    }
 else 
    NosaCar=Mode_Default; 
}
if(NosaCar==Mode_Default)
{
if (data[0]!=-1 && data[1]!=-1)
     { yAxis=data[0];
      xAxis=data[1];}  
      servoAngle = map(xAxis,250,0,-40,220);
      if (servoAngle <0)
      {
      servoAngle=0;
      }
      if (servoAngle >180)
      {
      servoAngle=180;
      }
      Servo1.write(servoAngle);
/* if((servoAngle<60||servoAngle>120)&&!go)
 {
  go=true;
 }
 timer=millis();
 if(timer>preTimer+100)
 {
  go=false;
 }
}while(go);*/
//////////////////////////////////////////Moving//////////////////////////////////////////
if(digitalRead(statPin)==LOW)
{
      yAxis=126; 
      xAxis =126;
}
if (yAxis < 100) 
{
//////////////////////////////////////////Set Motor A backward//////////////////////////////////////////
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
//////////////////////////////////////////Set Motor B backward//////////////////////////////////////////
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
//////////////////////////////////////////Convert the declining Y-axis readings for going backward////////////////////////////////////////// 
      motorSpeedA = map(yAxis, 126,0 , 0, 255);
      motorSpeedB = map(yAxis, 126,0 , 0, 255);
  }
  else if (yAxis > 156) 
  {
//////////////////////////////////////////Set Motor A forward//////////////////////////////////////////
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
//////////////////////////////////////////Set Motor B forward//////////////////////////////////////////
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
//////////////////////////////////////////Convert the increasing Y-axis readings for going forward//////////////////////////////////////////
      motorSpeedA = map(yAxis, 126, 250, 0, 150);
      motorSpeedB = map(yAxis, 126, 250, 0, 150);
  }
//////////////////////////////////////////If No Send data stays in middle the motors are not moving//////////////////////////////////////////
  else 
  {
      motorSpeedA = 0;
      motorSpeedB = 0;
  }
//////////////////////////////////////////X-axis used for left and right control//////////////////////////////////////////
  if (xAxis > 156) 
   {
//////////////////////////////////////////Convert the declining X-axis//////////////////////////////////////////
      int xMapped = map(xAxis, 156,250 , 0, motorSpeedA);
//////////////////////////////////////////Move to left - decrease left motor speed, increase right motor speed//////////////////////////////////////////
      motorSpeedA = 0;
      motorSpeedB = motorSpeedB + xMapped;
//////////////////////////////////////////Confine the range from 0 to 255//////////////////////////////////////////
    if (motorSpeedA < 0) 
    {
      motorSpeedA = 0;
    }
    if (motorSpeedB > 150) 
    {
      motorSpeedB = 150;
    }
  }
  if (xAxis < 100)
  {
//////////////////////////////////////////Convert the increasing X-axis readings//////////////////////////////////////////
    int xMapped = map(xAxis, 126, 0, 0, motorSpeedB);
//////////////////////////////////////////Move right - decrease right motor speed, increase left motor speed//////////////////////////////////////////
    motorSpeedA = motorSpeedA + xMapped;
    motorSpeedB = 0;
//////////////////////////////////////////Confine the range from 0 to 255//////////////////////////////////////////
    if (motorSpeedA > 150) 
    {
      motorSpeedA = 150;
    }
    if (motorSpeedB < 0) {
      motorSpeedB = 0;
    }
  }
//////////////////////////////////////////Prevent buzzing at low speeds////////////////////////////////////////// 
  if (motorSpeedA < 70) 
  {
      motorSpeedA = 0;
  }
  if (motorSpeedB < 70) 
  {
      motorSpeedB = 0;
  }
//////////////////////////////////////////UltraSonic stop before hitting


      
 cm=calculate_distance(trigger,echo);
// maximum=maximum(motorSpeedA,motorSpeedB);     
//need to be changed
/*if(cm<((motorSpeedA%10)*10))
{
      motorSpeedA=0; 
      motorSpeedB =0;
}
*/

        analogWrite(enA, motorSpeedA ); // Send PWM signal to motor A
    
        analogWrite(enB,  motorSpeedB); // Send PWM signal to motor B 
}
else if (NosaCar==Mode_SelfDriving)
{
en = 1;  
count = Status;
Status = SelfMoving(en,count);  
}
}
////

///





void MoveForward ()
{

//////////////////////////////////////////Set Motor A forward//////////////////////////////////////////
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
//////////////////////////////////////////Set Motor B forward//////////////////////////////////////////
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
//////////////////////////////////////////Convert the increasing Y-axis readings for going forward//////////////////////////////////////////
      motorSpeedA = 100;
      motorSpeedB = 100;
      analogWrite(enA, motorSpeedA ); // Send PWM signal to motor A 
      analogWrite(enB,  motorSpeedB); // Send PWM signal to motor B 


}


void Stop()
{
      motorSpeedA = 0;
      motorSpeedB = 0;
      analogWrite(enA, motorSpeedA ); // Send PWM signal to motor A 
      analogWrite(enB,  motorSpeedB); // Send PWM signal to motor B 
      delay(200);
     
      
}
void MoveRight ()
{
  //////////////////////////////////////////Set Motor A Backward//////////////////////////////////////////
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
//////////////////////////////////////////Set Motor B forward//////////////////////////////////////////
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
//////////////////////////////////////////Convert the increasing Y-axis readings for going forward//////////////////////////////////////////
      motorSpeedA = 100;   // right motor
      motorSpeedB = 100; //left motor
      analogWrite(enA, motorSpeedA ); // Send PWM signal to motor A 
      analogWrite(enB,  motorSpeedB); // Send PWM signal to motor B 
      delay(500);
      //Stop(); 
       
}
void MoveLeft ()
{
  //////////////////////////////////////////Set Motor A forward//////////////////////////////////////////
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
//////////////////////////////////////////Set Motor B backward//////////////////////////////////////////
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
//////////////////////////////////////////Convert the increasing Y-axis readings for going forward//////////////////////////////////////////
      motorSpeedA = 100; // right motor
      motorSpeedB = 100; //left motor
      analogWrite(enA, motorSpeedA ); // Send PWM signal to motor A 
      analogWrite(enB,  motorSpeedB); // Send PWM signal to motor B 
      delay(500);
       //Stop();   
}

int SelfMoving(int en, int count )
{

  if (en == 1 && count < 10)
  {
    delay(2/0);
    cm = calculate_distance(trigger, echo);
    
    if (cm > 35 )
    {
      
      MoveForward();
      FromWhere ='F';
      return 0 ;
    }

    else if (cm < 35 && count < 5 )
     {
      if (FromWhere =='F')
      {Stop();}
      FromWhere ='R';
      MoveRight();
      count = count +1;
      return count;
     }

    else 
    {
      if (FromWhere =='F')
      {Stop();}
      MoveLeft();
      FromWhere ='L';
      count = count +1;
      return count;
     }

   }
   else 
   {
    /// MAKE BUZZZERRR
    NosaCar=Mode_Default;
    digitalWrite(Buzzer,HIGH);
    delay(2000);
     digitalWrite(Buzzer,LOW);
    return 0;
   }
      
}


//////////////////////////////////////////functions used//////////////////////////////////////////
float calculate_distance(int trig,int ech)
{
      digitalWrite(trig, LOW);
      delayMicroseconds(2);
      digitalWrite(trig, HIGH);
      delayMicroseconds(10);
      digitalWrite(trig, LOW);
      
      time_taken = pulseIn(ech, HIGH);
      dist= time_taken*0.034/2;
      return dist;
}
int MaxOf(int a, int b)
{
if(a>b)
return a;
if(b>a)
return b;
else 
return a;  
  
}
