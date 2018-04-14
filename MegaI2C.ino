#include <Adafruit_BNO055.h>
#include <FlexiTimer2.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <Adafruit_GPS.h>
#include "DFR_Key.h"

Adafruit_GPS GPS(&Serial3);                   // define GPS object DO NOT USE Serial0
DFR_Key keypad;                               // define keypad object
Servo myservo;                                // define servo object
Adafruit_BNO055 bno = Adafruit_BNO055(55);    // define BNO sensor object

#define GPSECHO  false                        // echo GPS Sentence 
#define Threshold 5                           // Threshold for Obstacle avoidance (number of obstacles)
#define DEC_ANGLE .17889625                   //   Global variables that are changed across functions

LiquidCrystal lcd( 8, 9, 4, 5, 6, 7); // define lcd pins use these default values for OUR LCD

// Global variables that change across functions
int STEERANGLE = 90;        // servo initial angle (range is 0:180)
float HEADING = 0;          // Heading in degree
int LidarRight;             // LIDAR left
int LidarLeft;              // LIDAR right
boolean usingInterrupt = false;
int carSpeedPin = 2;              // pin for DC motor (PWM for motor driver). don't use other pins....
float errorHeadingRef = 0;        // error
long int lat;                     // GPS latitude in degree decimal * 100000   |     we multiply decimal degree by 100000 to convert it to meter  https://en.wikipedia.org/wiki/Decimal_degrees
long int lon;                     // GPS latitude in degree decimal * 100000   |     0.00001 decimal degree is equal to 1.0247 m at 23 degree N/S
long int latDestination = 33.425891 * 100000;       // define an initial reference Latitude of destination
long int lonDestination =  -111.940458 * 100000;    // define an initial reference Longitude of destination
float Bearing = 0;                                  // initialize bearing
int localkey = 0;                                   // var
float hmb;
double distance;

void setup() {
  //pin setting up
  myservo.attach(44);     // servo is connected to pin 44
  lcd.begin( 16, 2 );     // LCD type is 16x2 (col & row)
  
  Serial.begin(9600);     // serial for monitoring
  
  Serial.println("Orientation Sensor Calibration"); Serial.println("");
  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) { //OPERATION_MODE_COMPASS is for a precise tilt compensated compass 
    Serial.print("Ooops, no BNO055 detected ... ");
    while (1);
  }

  //gps setup
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);

  byte c_data[22] = {0, 0, 0, 0, 0, 0, 17, 255, 8, 3, 12, 3, 0, 0, 255, 255, 2, 0, 232, 3, 139, 4};

  bno.setCalibData(c_data);                                                                                       // SET CALIBRATION DATA
  bno.setExtCrystalUse(true);
  // set timer interrupts
  
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 59016;           // every 0.1 second
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << TOIE1);  // enable timer compare interrupt

  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4  = 336;             // every 1 second
  TCCR4B |= (1 << CS42);    // 256 prescaler
  TIMSK4 |= (1 << TOIE4);  // enable timer compare interrupt
  interrupts();
  useInterrupt(true);

  ///Setting the reference (Lat and Lon)
  localkey = 0;
      Serial.println("hello");

  while (localkey != 1) {    // wait for select button
    lcd.clear();
    localkey = keypad.getKey();
    lcd.print("Press Select");
    lcd.setCursor(0, 1);
    lcd.print("to save dest.");
    delay(100);               // delay to make display visible
  }
  Serial.println("key pressed 2nd time");
  
  GPSRead();
  latDestination = lat;     // saving the destiantion point  
  lonDestination = lon;     // saving the destiantion point
  
  localkey = 0;
  while (localkey != 1) {   // wait for select button
    lcd.clear();
    localkey = keypad.getKey();
    lcd.print("Press Select");
    lcd.setCursor(0, 1);
    lcd.print("to drive!");
    delay(100);             // delay to make display visible
  }
}

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();            // interrupt for reading GPS sentences char by char
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
#endif
}

void useInterrupt(boolean v) {    // enable inttrupt for GPS
  if (v) {
    OCR0A = 0xAF;               // Timer0 is already used for millis()
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

ISR(TIMER4_OVF_vect) {  // Timer interrupt for reading GPS DATA
  sei();        //   reset interrupt flag
  TCNT4  = 336; //   re-initialize timer value
  GPSRead();    //   read GPS data
}

void GPSRead() {
  // read GPS data
  if(GPS.newNMEAreceived())
    GPS.parse(GPS.lastNMEA());
   if(GPS.fix){
    lat = GPS.latitude * 1000;   
    lon = - GPS.longitude * 1000;
   }
}


void ReadHeading()
{
  // calculate HEADING
    // read Heading angle
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055:: VECTOR_MAGNETOMETER); 

  // Calculate heading when the magnetometer is level, then correct for signs of axis.   
  HEADING = atan2(euler.y(), euler.x());  
  HEADING += DEC_ANGLE;
    
  // Correct for when signs are reversed.
  if(HEADING < 0)
  {
    HEADING += 2*PI;
  } 
  // Check for wrap due to addition of declination.
  if(HEADING >= 2* PI){
    HEADING -= 2*PI;
  }
  // Convert radians to degrees .
  HEADING = HEADING * 180/ M_PI ; 
  HEADING -= 90;
  if(HEADING < 0){
      HEADING += 360;
    }
  delay(100);
}

void CalculateBearing() {
  // Calculate Steer angle based on GPS data and IMU
    Bearing = (atan2((lonDestination - lon),(latDestination - lat))) * 180 / PI;
  if(Bearing < 0){
    Bearing += 360;
  }
}

void CalculateSteer() {
  hmb = HEADING - Bearing;
  if(10 < hmb && hmb < 180)
  {
    STEERANGLE = 60;
  }
  else if( 180 < hmb && hmb < 360 ){
        STEERANGLE = 120;
        //Serial.println("exe2");
  }
  else if ( -180 < hmb && hmb < 0){
        STEERANGLE = 120;
        //Serial.println("exe3");
  }
  else if(-350 < hmb && hmb < -180){
        STEERANGLE = 60;
        //Serial.println("exe4");
  }
  else{
         STEERANGLE = 90;
  }
}

void SetCarDirection() {    // Input: Lidar data
  // Set Steering angle,
  // If any obstacle is detected by Lidar, Ignore steering angle and turn left or right based on observation
  
  if(LidarRight >= Threshold || LidarLeft >= Threshold){ 
      //Obsticle detected, turn left or right
      if(LidarRight >= LidarLeft){  //turn left
        myservo.write(60);
      }else{  //turn right
        myservo.write(120);
      }
  }
  else{
    myservo.write(STEERANGLE);
  }
}

void SetCarSpeed() {  // Input: GPS data
  // set speed,
  // if destination is within 5 meters of current position, set speed to zero.
  distance = sqrt((latDestination - lat)*(latDestination - lat)+ (lonDestination - lon) * (lonDestination - lon) );
  
  if(GPS.newNMEAreceived()){
    if (!GPS.parse(GPS.lastNMEA())) return;
  }
  if(GPS.fix){
    if(distance > 2){
      analogWrite(carSpeedPin, 30);
    }
    else{
      analogWrite(carSpeedPin, 0);
    }
  }else{
    analogWrite(carSpeedPin, 0);
  }
}

void ReadLidar() {    // Output: Lidar Data
  // read Lidar Data from Nano Board (I2C)
   Wire.requestFrom(8, 2);    // request 1 byte from slave device #55
 
     while (Wire.available()) {     // slave may send less than requested

     LidarRight = Wire.read();    // receive a byte as character
     LidarLeft = Wire.read();    // receive a byte as character

           Serial.print("right: ");
           Serial.print(LidarRight);         // print the character
           Serial.print("Left: ");
           Serial.print(LidarLeft);      
     }

  //delay(500); 
}

ISR(TIMER1_OVF_vect) {        // function will be call every 0.1 seconds
  sei();                  // reset interrupt flag
  TCNT1  = 59016;
  ReadHeading();
  ReadLidar();
  CalculateBearing();
  CalculateSteer();
  SetCarDirection();
  SetCarSpeed();
}


void printHeadingOnLCD() {

}

void printLocationOnLCD() {
    lcd.setCursor(0,0);
    lcd.print("lat: ");lcd.print(lat, 3);lcd.print(" ");// print latitude
    lcd.setCursor(0,1);
    lcd.print("lon: ");lcd.print(lon, 3);
}

void printDistanceOnLCD() {

}

void printObstacleOnLCD() {

}

void loop() {
  lcd.clear();      // clear lcd
  // Pring data to LCD in order to debug your program!
  //printHeadingOnLCD();
  //printObstacleOnLCD();
  printLocationOnLCD() ;
  delay(100);
}
