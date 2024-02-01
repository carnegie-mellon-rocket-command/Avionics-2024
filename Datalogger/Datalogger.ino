/*
  SD card datalogger
 
 This example shows how to log data from two digital sensors and one analog sensor
 to an SD card using the SD library.

In this example you can fiddle with a potentiometer, 
press a button to save the value of the pot to the teensy 4.1 microSD 
and press a different button to display what value was saved.
   
 The circuit:
 * analog sensors on analog ins 0, 1, and 2
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11, pin 7 on Teensy with audio board
 ** MISO - pin 12
 ** CLK - pin 13, pin 14 on Teensy with audio board
 ** CS - pin 4,  pin 10 on Teensy with audio board
 
 created  24 Nov 2010
 modified 9 Apr 2012
 by Tom Igoe
then modified again (horrible) by Andrew C. 

 This example code is in the public domain.
   
 */

#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include "SparkFunMPL3115A2.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


//Create an instance of the object
MPL3115A2 myPressure;

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


// On the Ethernet Shield, CS is pin 4. Note that even if it's not
// used as the CS pin, the hardware CS pin (10 on most Arduino boards,
// 53 on the Mega) must be left as an output or the SD library
// functions will not work.

// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
// Teensy audio board: pin 10
// Teensy 3.5 & 3.6 & 4.1 on-board: BUILTIN_SDCARD
// Wiz820+SD board: pin 4
// Teensy 2.0: pin 0
// Teensy++ 2.0: pin 20
//===============================================================================================================================


const int chipSelect = BUILTIN_SDCARD; 
sensors_event_t angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;

long start_time, temp_time, curr_time, timer;
long loop_target = 10;
long prev_loop = 1;
float loop_time = 0;

int LED = 13;

String buf = "";
int buf_size = 500;

String s;

float altitude, altitude_0;

float altimeter_k = .9;
float altimeter_filtered = 0;
float prev_alt;

float accel_k = .9;
float accel_filtered = 0;
float accel;
double x, y, z;
float acc_vert = 0;



float vel_k = .9;
float vel_filtered = 0;
float vel;

float prediction_k = .9;
float prediction_filtered = 0;
float prediction;

float fusion_vel = 0;
float imu_vel = 0;
float fusion_k = .985;

bool SD_active = true;

float accel_thresh = 10;

bool launched = false;

Servo ATS;
int ATS_pin = 6;
int ATS_min = 75;
int ATS_max = 15;
float ATS_pos = 0;
//================================================================================================================================
//this part sets things up!

void setup()
{
  
 // Open serial communications and wait for port to open:
   Serial.begin(115200);
  
  Serial.print("Initializing SD card...");
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    setup();
  }
  Serial.println("card initialized.");
  
  Wire.begin();
  myPressure.begin(); // Get sensor online

  //Configure the sensor
  myPressure.setModeAltimeter(); // Measure altitude above sea level in meters
  //myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa

  myPressure.setOversampleRate(1); // Set Oversample to the recommended 128
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags 

  altitude_0 = myPressure.readAltitudeFt();
  
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

 
  ATS.attach(ATS_pin);
  set_ATS(ATS_pos);
  delay(500);
  ATS.detach();
  pinMode(LED, OUTPUT);
  start_time = millis();
  prev_loop = start_time;
  WriteData("***********************************************************************************");
}

void loop()
{
    buf = "";
    for (int i = 0; i < buf_size; i++) {
      //timeout1.timeOut(300, setup);
    // make a string for assembling the data to log:
  
  
      run_timer();

      read_altimeter();
      read_IMU();

      filter_altimeter();
      filter_accel(&linearAccelData);
      filter_velocity();
      velocity_fusion();
      make_prediction();

      //time, loop_time, alt, alt_filtered, vel, vel_filtered;
      String s1 = String(timer) + ", " + String(loop_time) + ", " + String(altitude) + ", " + String(altimeter_filtered) + ", " + String(vel) + ", " + String(vel_filtered);
      String s2 = String(prediction) + ", " + String(prediction_filtered) + ", " + String(accel) + ", " + String(accel_filtered);
      String s3 = printEvent(&linearAccelData);
      String s4 = printEvent(&accelerometerData);
      String s5 = printEvent(&gravityData);
      String s6 = printEvent(&angVelocityData);
      String s7 = printEvent(&magnetometerData);
      s = s1 + String(", ") + s2 + String(", ") + s3 + String(", ") + s4 + String(", ") + s5 + String(", ") + s6 + String(", ") + s7; 
      Serial.println(fusion_vel);

      
      //Serial.println(s);
      if (i != buf_size - 1) s = s + "\n";
      
      buf += s;

      if (accel_filtered > accel_thresh && launched == false){
        launched = true;
        ATS.attach(ATS_pin);
      }

      if (launched) {
        set_ATS(1);
      }
    
     
   }

   if (launched) {
     digitalWrite(LED, LOW);
     WriteData(buf);
     //Serial.println(buf);
     digitalWrite(LED, HIGH);
     //set_ATS(1);
 
   } 
   
 }



void run_timer() {
  temp_time = millis() - prev_loop;
  if (temp_time < loop_target) {
    delayMicroseconds((loop_target - temp_time) * 1000);
  }
  curr_time = millis();
  timer = curr_time - start_time;
  loop_time = (curr_time - prev_loop);
  //if ((curr_time - prev_loop) > loop_time) loop_time = (curr_time - prev_loop); //log worst loop time if desired
  prev_loop = curr_time;
}

 void WriteData(String text) {
    
    // open the file named datalog.txt on the sd card
    //Serial.println("open");
    if (SD_active) {
      File dataFile = SD.open("datalog.txt", FILE_WRITE);
      //Serial.println("opened");
      // if the file is available, write the contents of datastring to it
      if (dataFile) {
        dataFile.println(text);
        dataFile.close();
        Serial.println("writing");
      }  
      // if the file isn't open, pop up an error:
      else {
        SD_active = false;
        Serial.println("error opening datalog.txt");
        //SD.begin(chipSelect);
      }  
    } else {
      Serial.println("SD Failed. Continuing without logging"); 
    }
     
 }


void read_IMU() {
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
}

 String printEvent(sensors_event_t* event) {
  String val;
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    //val = "Accl:";
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    //val = "Orient:";
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    //val = "Mag:";
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    //val = "Gyro:";
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    //val = "Rot:";
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    //val = "Linear:";
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    acc_vert = y;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    //val = "Gravity:";
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    //Serial.print("Unk:");
  }

  return String(x)+ ", " + String(y) + ", " + String(z);
}

void config_acc(int addr) {

    //G range
    Wire.beginTransmission(0x28);
    Wire.write(0x08);
    Wire.write(0b00001111);
    Wire.endTransmission();
}

void read_altimeter() {
  altitude = myPressure.readAltitudeFt() - altitude_0;
}

 void filter_accel(sensors_event_t* event) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    accel = float(sqrt(x*x + y*y + z*z));
    accel_filtered = accel*(1-accel_k) + accel_filtered*accel_k; 
 }


void filter_altimeter() {
  prev_alt = altimeter_filtered;
  altimeter_filtered = altimeter_k * altimeter_filtered + (1 - altimeter_k) * altitude;
}

 void filter_velocity() {
    vel = (altimeter_filtered - prev_alt) / loop_time * 1000;
    vel_filtered = vel*(1-vel_k) + vel_filtered*vel_k;
 }

 void make_prediction() {
  float v = vel_filtered * .3048;
  prediction = v*v / ((accel_filtered + 9.81)*.3048) + altimeter_filtered;
  prediction_filtered = prediction*(1-prediction_k) + prediction_filtered*(prediction_k);
 }

 void velocity_fusion() {
  imu_vel = fusion_vel - ((acc_vert<0) ? 1: -1)*3.2808*accel_filtered*loop_time/1000.0;
  if (acc_vert < 20) {
    fusion_vel = imu_vel*fusion_k + (1-fusion_k)*vel_filtered;
  } else {
    fusion_vel = vel_filtered;
  }
  
 }


//Takes a val between 0 and 1. 0 is full retraction, 1 is full extension
void set_ATS(float val) {
  float pos = (ATS_max - ATS_min) * val + ATS_min;
  ATS.write(int(pos));
}
