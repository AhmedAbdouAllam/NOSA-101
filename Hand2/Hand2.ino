///////////////////////////////////////////Including needed Libraries///////////////////////////////////////////
#include <SoftwareSerial.h>
#include <Wire.h>
SoftwareSerial BTSerial(3, 4);
///////////////////////////////////////////Defining address for accelerometer
#define DEVICE (0x53) // Device address as specified in data sheet
#define TX 3
#define ResetButton 12
#define ModeInverter 11
///////////////////////////////////////////Acccelerometer vars///////////////////////////////////////////
byte _buff[6];
char POWER_CTL = 0x2D; //Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32; //X-Axis Data 0
char DATAX1 = 0x33; //X-Axis Data 1
char DATAY0 = 0x34; //Y-Axis Data 0
char DATAY1 = 0x35; //Y-Axis Data 1
char DATAZ0 = 0x36; //Z-Axis Data 0
char DATAZ1 = 0x37; //Z-Axis Data 1
///////////////////////////////////////////TX Vars///////////////////////////////////////////
int data[2];
int xc=0;
bool Set=false;
int preTimer;
int Timer;
int yc=0;
int zc=0;
int x,y,z;
int notrecived=0;
///////////////////////////////////////////Setup Function///////////////////////////////////////////
void setup(){

///////////////////////////////////////////Setup for accelerometer///////////////////////////////////////////
      Wire.begin(); 
      BTSerial.begin(38400);
      Serial.begin(9600); // start serial for output
///////////////////////////////////////////Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register///////////////////////////////////////////
      writeTo(DATA_FORMAT, 0x01);
///////////////////////////////////////////Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register///////////////////////////////////////////
      writeTo(POWER_CTL, 0x08);
///////////////////////////////////////////Calibrating///////////////////////////////////////////      
ResetAcclerometer();
}
void ResetAcclerometer()
{
      readAccel(); // read the x/y/z tilt
      xc = x;
      yc = y;
      zc = z;
}
///////////////////////////////////////////Loop Function///////////////////////////////////////////
void loop()
{
 
///////////////////////////////////////////Reading Measurements then calibrating///////////////////////////////////////////
      readAccel(); // read the x/y/z tilt
      x-=xc;
      y-=yc;
      z-=zc;
      
      data[0]=map(y,-180,180,0,250);
      data[1]=map(x,-180,180,0,250);;
 //////////////////////////////////////////Checking the Reset Button///////////////////////////////////////////     
if(digitalRead(ResetButton)==HIGH)
     { ResetAcclerometer();}
 
 if(digitalRead(ModeInverter)==HIGH) 
    {
      data[0]=255;
      data[1]=255;
    Set=true;  
       BTSerial.write(data[0]);
       BTSerial.write(data[1]);
       Serial.print("State Changed");
    while(digitalRead(ModeInverter)==HIGH);
    }  
    
///////////////////////////////////////////Sending the measurements///////////////////////////////////////////      
      if(!Set)
      if (!BTSerial.overflow())
      { 
        
       BTSerial.write(data[0]);
       BTSerial.write(data[1]);
      }
      Set=false;
      delay(250);
      Serial.print("yAxis F/B : ");
      Serial.print(data[0]);
      Serial.print("xAxis R/L : ");
      Serial.print(data[1]);
      Serial.println();
}
///////////////////////////////////////////Functions needed///////////////////////////////////////////
void readAccel() {
uint8_t howManyBytesToRead = 6;
readFrom( DATAX0, howManyBytesToRead, _buff); //read the acceleration data from the ADXL345
// each axis reading comes in 10 bit resolution, ie 2 bytes. Least Significat Byte first!!
// thus we are converting both bytes in to one int

x = ((((int)_buff[1]) << 8) | _buff[0]);
y = ((((int)_buff[3]) << 8) | _buff[2]);
z = ((((int)_buff[5]) << 8) | _buff[4]);
}
void writeTo(byte address, byte val) {
Wire.beginTransmission(DEVICE); // start transmission to device
Wire.write(address); // send register address
Wire.write(val); // send value to write
Wire.endTransmission(); // end transmission
}
// Reads num bytes starting from address register on device in to _buff array
void readFrom(byte address, int num, byte _buff[]) {
Wire.beginTransmission(DEVICE); // start transmission to device
Wire.write(address); // sends address to read from
Wire.endTransmission(); // end transmission
Wire.beginTransmission(DEVICE); // start transmission to device
Wire.requestFrom(DEVICE, num); // request 6 bytes from device
int i = 0;
while(Wire.available()) // device may send less than requested (abnormal)
{
_buff[i] = Wire.read(); // receive a byte
i++;
}
Wire.endTransmission(); // end transmission
}
