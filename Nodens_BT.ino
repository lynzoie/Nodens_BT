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


int HR_Val;                // holds the incoming raw data. Signal value can range from 0-1024
int Threshold = 550;            // Determine which Signal to "count as a beat", and which to ingore.

void setup() {
  //////////// HEART RATE SENSOR SETUP //////////////
  pinMode(LED13,OUTPUT);         // pin that will blink to your heartbeat!
  
  ///////////////// BLUETOOTH SETUP /////////////////
   Serial.begin(9600);
  // Serial.println("HM10 serial started at 9600");
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

void Send_Data(String String_Message, SoftwareSerial HM10) {      // Converts strings to char arrays so that we can transmit data via Bluetooth
  char* char_message = (char*) malloc(sizeof(char)*String_Message.length()+1);
  String_Message.toCharArray(char_message, String_Message.length()+1);
  HM10.write(char_message);
  free(char_message);
  
}
String HR_Str;
String AG_Str[6];

void loop() {
  
  HR_Val = analogRead(PulseSensorPurplePin); // 10-bit number, convert to string

  // get values of Accelerometer/Gyroscope
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp); 

  // Display data on serial plotter
  Serial.println(HR_Val);
  Serial.print(a.acceleration.x); 
  Serial.print(",");
  Serial.print(a.acceleration.y);
  Serial.print(",");
  Serial.print(a.acceleration.z);
  Serial.print(",");
  Serial.print(g.gyro.x);
  Serial.print(",");
  Serial.print(g.gyro.y);
  Serial.print(",");
  Serial.print(g.gyro.z);  
  
  // Creating data packets to send to Bluetooth module
  HR_Str    = "H"  + String(HR_Val);
  AG_Str[0] = "AX" + String(a.acceleration.x);
  AG_Str[1] = "AY" + String(a.acceleration.y);
  AG_Str[2] = "AZ" + String(a.acceleration.z);
  AG_Str[3] = "GX" + String(g.gyro.x);
  AG_Str[4] = "GY" + String(g.gyro.y);
  AG_Str[5] = "GZ" + String(g.gyro.z);
  // Sending heart-rate, accelerometer, and gyroscope data
  Send_Data(HR_Str, HM10);
  Send_Data(AG_Str[0],HM10);
  Send_Data(AG_Str[1],HM10);
  Send_Data(AG_Str[2],HM10);
  Send_Data(AG_Str[3],HM10);
  Send_Data(AG_Str[4],HM10);
  Send_Data(AG_Str[5],HM10);
  
  // Send_Data(GPS.x);
  
  // Send_Data(GPS.y);
  
  // Send_Data(GPS.z);
  delay(10);
  
}
