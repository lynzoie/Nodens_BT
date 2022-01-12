// Bluetooth libraries and variables
#include <SoftwareSerial.h>
SoftwareSerial HM10(2,3);
char appData;  
String inData = "";

// Accelerometer & Gyroscope libraries and variables
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
Adafruit_MPU6050 mpu;

// Heart Rate Sensor variables
int PulseSensorPurplePin = 0;        // Pulse Sensor PURPLE WIRE connected to ANALOG PIN 0
int LED13 = 13;   //  The on-board Arduion LED


int Signal;                // holds the incoming raw data. Signal value can range from 0-1024
int Threshold = 550;            // Determine which Signal to "count as a beat", and which to ingore.

void setup() {
  //////////// HEART RATE SENSOR SETUP //////////////
  pinMode(LED13,OUTPUT);         // pin that will blink to your heartbeat!
  
  ///////////////// BLUETOOTH SETUP /////////////////
  Serial.begin(9600);
  Serial.println("HM10 serial started at 9600");
  HM10.begin(9600);

  ////////// ACCELEROMETER/GYROSCOPE SETUP //////////
  //Serial.begin(115200);
  //Serial.println("Initialize MPU6050");

  // find MPU6050
  if (!mpu.begin()) {
    //Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

/*
void BT_Mod(val) {
  HM10.listen();  // listen the HM10 port
  while (HM10.available() > 0) {   // if HM10 sends something then read
    appData = HM10.read();
    inData = String(appData);  // save the data in string format
    //Serial.write(appData);
  }
    
  if (Serial.available()) {           // Read user input if available.
    delay(10);
    HM10.write(Serial.read());
  }
  if ( inData == "F") {
    Serial.println("LED OFF");
    digitalWrite(12, LOW);
    //digitalWrite(13, LOW); // switch OFF LED
    delay(1000);
  }
  if ( inData == "N") {
    Serial.println("LED ON");
    digitalWrite(12, HIGH);
    delay(1000);
    digitalWrite(12, HIGH);
    delay(1000);
  }
}*/


int HR_Sensor() {
  Signal = analogRead(PulseSensorPurplePin);    // read PulseSensor's value 
  Serial.println(Signal);                    // Send the Signal value to Serial Plotter.
  return Signal;
}

void Acc_Gyro() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print(",");
  Serial.print(a.acceleration.x);
  Serial.print(",");
  Serial.print(a.acceleration.y);
  Serial.print(",");
  Serial.print(a.acceleration.z);
//  Serial.println(" m/s^2");

  Serial.print(",");
  Serial.print(g.gyro.x);
  Serial.print(",");
  Serial.print(g.gyro.y);
  Serial.print(",");
  Serial.print(g.gyro.z);
//  Serial.println(" rad/s");

  Serial.println("");
  
}

int HR_Val;
void loop() {
  HR_Val = HR_Sensor();
  //Acc_Gyro();

  // Sending HR_Val
  HM10.write(HR_Val);

  
//  Signal = analogRead(PulseSensorPurplePin);    // read PulseSensor's value 
//  Serial.println(Signal);                    // Send the Signal value to Serial Plotter.

//  if(Signal > Threshold){                          // If the signal is above "550", then "turn-on" Arduino's on-Board LED.
//    digitalWrite(LED13,HIGH);
//  } else {
//    digitalWrite(LED13,LOW);                //  Else, the sigal must be below "550", so "turn-off" this LED.
//  }
  
  /* Get new sensor events with the readings */
//  sensors_event_t a, g, temp;
//  mpu.getEvent(&a, &g, &temp);
//
//  /* Print out the values */
//  Serial.print(",");
//  Serial.print(a.acceleration.x);
//  Serial.print(",");
//  Serial.print(a.acceleration.y);
//  Serial.print(",");
//  Serial.print(a.acceleration.z);
////  Serial.println(" m/s^2");
//
//  Serial.print(",");
//  Serial.print(g.gyro.x);
//  Serial.print(",");
//  Serial.print(g.gyro.y);
//  Serial.print(",");
//  Serial.print(g.gyro.z);
////  Serial.println(" rad/s");
//
//  Serial.println("");
  delay(10);
  
}
