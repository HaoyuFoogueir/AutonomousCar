#include <Wire.h>
#include <RPLidar.h>

RPLidar lidar;          // define lidar as RPLIDAR Object
#define RPLIDAR_MOTOR 3 // motor pin for lidar speed control

void setup() {
  pinMode(RPLIDAR_MOTOR, OUTPUT); // set lidar speed control pin as output
  lidar.begin(Serial);            // begin communication with lidar
  Wire.begin(8);                  // join i2c bus with address 8 as SLAVE
  Wire.onRequest(requestEvent);   // call "requestEvent" function when receives a request
  Wire.onReceive(receiveEvent);   // call "receiveEvent" function when receives a byte
}

int left = 0;                     // variable for detected points on left hand side
int right = 0;                    // variable for detected points on right hand side
unsigned long time = millis();    // time variable for reseting variables

int c1;                           // variable for received integer
char c2;                           // variable for received char
float amax=90, amin=90, aMaxOld=90, aMinOld=90;
float dmax, dMaxOld; 

void receiveEvent(int bytes)
{
  // read the received byte as integer. 
  while ( Wire.available()) {          // loop through all 
    c2  = Wire.read();
  }
}

void requestEvent() {
    Wire.write(right);
    Wire.write(left);
}
 
void loop() 
{
  if (IS_OK(lidar.waitPoint())) { 
    
    // if lidar is working properly (waiting time less than timeout)
    // read angle and distance of the obstacle

    float distance = lidar.getCurrentPoint().distance;//distance value in mm unit    
    float angle = lidar.getCurrentPoint().angle;//angle value in degree    
    
    bool startBit = lidar.getCurrentPoint().startBit; //whether this point belongs to a new scan    
    byte quality = lidar.getCurrentPoint().quality; //quality of the current measurement

    if ( (distance>500) && (distance<1000) && (20 >= angle)) {     
      right++; 
    } else if ((distance>500) && (distance<1000) && (angle >= 340)){
      left++;
    }

    if(millis() - time > 1000){
      
      time = millis();
      amax=90;
      amin=90; 
      aMinOld=90; 
      aMaxOld=90;
      dmax=0;
      dMaxOld=0;
      right = 0;
      left = 0;
    }
  } else {                                                  // if lidar is not responding
    analogWrite(RPLIDAR_MOTOR, 0);                          //stop the rplidar motor 
    rplidar_response_device_info_t info;                    // try to detect RPLIDAR... 
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {            // if detected, 
      lidar.startScan();                                    // start scan 
      analogWrite(RPLIDAR_MOTOR, 255);                      // start motor rotating at max speed 
      delay(1000); 
    }
  }
}


