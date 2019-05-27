#define address 0x1E //0011110b, I2C 7bit address of HMC5883
#include <SoftwareSerial.h>
#include <Wire.h>
#include <math.h>
#include <TinyGPS++.h>

float finallat=0;
float finallong=0;        //Enter Latitude and Longitude of Final Destination
float wheelradius = 5;    //Enter radius of wheel used
float axisradius = 10;    //Enter axis radius of Bot

float currentlat;
float currentlong;
float latdiff;
float longdiff;
float finalangle;
float rotationtime;       //Variable required for further calculation
int x,y,z;
double xgauss,ygauss,D;
float traveldistance;
float traveltime;

//char currentdirection[32];
SoftwareSerial ss(11, 10);    //Initializing pins of NEO-6M Module

const int motorPin1  = 5;  
const int motorPin2  = 6;     // Motor A

const int motorPin3  = 3; 
const int motorPin4  = 4;     //Motor B
TinyGPSPlus gps;

void setup(){
  Serial.begin(9600);
  Wire.begin();
  ss.begin(9600);             //Initalizing Magnetometer
  
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();   
  digitalWrite(13,HIGH);
}


void loop(){
  gps.encode(ss.read());
  currentlat = gps.location.lat();
  currentlong = gps.location.lng();                                 // Get current location

  while (finallat != currentlat && finallong != currentlong){
  

  //Tell the HMC5883L where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
 
 //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb 
  }                                             // Gets magnetic fiels strength in 3 directions
  xgauss = x*0.48828125;
  ygauss = y*0.48828125;
  if(xgauss == 0)
  {
    if (ygauss < 0){D = 90;}
    else{D = 0;}     
  }
  else{
      D = atan2(ygauss,xgauss);
      D = (D*180)/3.14;
  }
  if (D>360){D = 360-D;}
  if (D<0){D = 360+D;}                    // Convert magnetic field strength to Compass Scale

/*if (D>337.25 || D< 22.5){strncpy( currentdirection, "North", sizeof(currentdirection) );}
  if (D<337.25 && D> 292.5){strncpy( currentdirection, "NorthWest", sizeof(currentdirection) );}
  if (D<292.5 && D> 247.5){strncpy( currentdirection, "West", sizeof(currentdirection) );}
  if (D<247.5 && D> 202.5){strncpy( currentdirection, "SouthWest", sizeof(currentdirection) );}
  if (D<202.5 && D> 157.5){strncpy( currentdirection, "South", sizeof(currentdirection) );}
  if (D<157.5 && D> 112.5){strncpy( currentdirection, "SouthEast", sizeof(currentdirection) );}
  if (D<112.5 && D> 67.5){strncpy( currentdirection, "East", sizeof(currentdirection) );}
  if (D<67.5 && D> 0){strncpy( currentdirection, "NorthEast", sizeof(currentdirection) );}*/

  latdiff = finallat - currentlat;
  longdiff = finallong - currentlong;
  finalangle = (atan2(latdiff,longdiff))*(180/3.14);
  traveldistance = sqrt((latdiff*latdiff)+(longdiff*longdiff));       // Gets direction of final location from present position
  
  if (finalangle>0 && finalangle<90){finalangle = 90 - finalangle;}
  if (finalangle>90 && finalangle<180){finalangle = 360-(finalangle -90);}
  if (finalangle>180 && finalangle<270){finalangle = 360-(finalangle -90);}
  if (finalangle>270 && finalangle<360){finalangle = 360-(finalangle -90);}
  if (finalangle == 0 || finalangle == 360){finalangle = 90;}
  if (finalangle == 90){finalangle = 0;}
  if (finalangle == 180){finalangle = 270;}
  if (finalangle == 270){finalangle = 180;}                         // Convert both directions on same scale

  
  if (D<finalangle)
  {
    rotationtime = ((0.04777070064)*axisradius*(finalangle-D)*(3.14/180))/wheelradius;  //Derive time required to rotate the motor for turning specific angle
    digitalWrite(motorPin2,HIGH);
    digitalWrite(motorPin1,LOW);
    digitalWrite(motorPin4,LOW);
    digitalWrite(motorPin3,HIGH);
    delay(rotationtime);
    digitalWrite(motorPin2,LOW);
    digitalWrite(motorPin1,LOW);
    digitalWrite(motorPin4,LOW);
    digitalWrite(motorPin3,LOW);                                       
  }
  if (D>finalangle)
  {
    rotationtime = ((0.04777070064)*axisradius*(D-finalangle)*(3.14/180))/wheelradius;
    digitalWrite(motorPin2,LOW);
    digitalWrite(motorPin1,HIGH);
    digitalWrite(motorPin4,HIGH);
    digitalWrite(motorPin3,LOW);
    delay(rotationtime/1000);
    digitalWrite(motorPin2,LOW);
    digitalWrite(motorPin1,LOW);
    digitalWrite(motorPin4,LOW);
    digitalWrite(motorPin3,LOW);                                    // Rotate Motor towards final position
  }
  traveltime = (0.04777070064/wheelradius)*(traveldistance/100);
    digitalWrite(motorPin2,HIGH);
    digitalWrite(motorPin1,LOW);
    digitalWrite(motorPin4,HIGH);
    digitalWrite(motorPin3,LOW);
    delay(traveltime/1000);
    digitalWrite(motorPin2,LOW);
    digitalWrite(motorPin1,LOW);
    digitalWrite(motorPin4,LOW);
    digitalWrite(motorPin3,LOW);                                    // Travel upto 1/100 of total distance to correct error later
  }
}
