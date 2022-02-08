// Bluetooth libraries and variables
#include <SoftwareSerial.h>
#define USE_ARDUINO_INTERRUPTS true
#include <PulseSensorPlayground.h>
SoftwareSerial HM10(2,3);
char appData;  
String inData = "";

// Accelerometer & Gyroscope libraries and variables
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
Adafruit_MPU6050 mpu;

// Heart Rate Sensor variables
int PulseWire = 0;                  // Pulse Sensor PURPLE WIRE connected to ANALOG PIN 0
int LED13 = 13;
int HR_Val;                         // holds the incoming raw data. Signal value can range from 0-1024
int Threshold = 400;                // Determine which Signal to "count as a beat", and which to ingore.
PulseSensorPlayground pulseSensor;  // Create PulseSensorPlayground object

// GPS Module variables
#include <TinyGPS++.h>
static const int RXPin = 6, TXPin = 5;
// The TinyGPS++ object
TinyGPSPlus gps;
// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup() {
  Serial.begin(9600);
  
  //////////// HEART RATE SENSOR SETUP //////////////
  pulseSensor.analogInput(PulseWire);
  pulseSensor.blinkOnPulse(LED13);   // pin that will blink to your heartbeat!
  pulseSensor.setThreshold(Threshold);
  pulseSensor.begin();

  ///////////////// BLUETOOTH SETUP /////////////////
  HM10.begin(9600);
  HM10.write("Connection Established");   // indicate BT-Phone connection has been made
  ////////// ACCELEROMETER/GYROSCOPE SETUP //////////
  // find MPU6050
  if (!mpu.begin()) {
    //Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);   // set accelerometer range to +-8G
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);        // set gyro range to +- 500 deg/s
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);     // set filter bandwidth to 21 Hz
  
  ///////////////// GPS SETUP ///////////////////////
  ss.begin(9600);
}

void Send_Data(String String_Message, SoftwareSerial HM10) {      // Converts strings to char arrays so that we can transmit data via Bluetooth
  char* char_message = (char*) malloc(sizeof(char)*String_Message.length()+1);
  String_Message.toCharArray(char_message, String_Message.length()+1);
  HM10.write(char_message);
  free(char_message);
  
}
String Data_Packet;
bool flag = true;
void loop() {
  HR_Val = pulseSensor.getBeatsPerMinute();

  // get values of Accelerometer/Gyroscope
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);  

//  ss.listen();    // listen to GPS port
//  // creating GPS data
//  if (ss.available() > 0){
//     gps.encode(ss.read());
//     if (gps.location.isUpdated()){
//       Serial.print("Latitude= "); 
//       Serial.print(gps.location.lat(), 6);
//       Serial.print(" Longitude= "); 
//       Serial.println(gps.location.lng(), 6);
//     }
//   }

  // setup Bluetooth to listen to request to send data
  HM10.listen();  // listen to Bluetooth port
  while (HM10.available() > 0) {   // if HM10 sends something then read
    appData = HM10.read();
    inData = String(appData);  // save the data in string format
    //Serial.write(appData);
  }
  if (Serial.available()) {           // Read user input if available.
    delay(10);
    HM10.write(Serial.read());
  }
  if ( inData == "S") {     // send packets
    // Creating data packets to send to Bluetooth module
    Data_Packet = "HR" + String(HR_Val) + "AX" + String(a.acceleration.x) + "AY" + String(a.acceleration.y) + "AZ" + String(a.acceleration.z) + "GX" + String(g.gyro.x) + "GY" + String(g.gyro.y) + "GZ" + String(g.gyro.z) + "LA" + String(gps.location.lat()) + "LO" + String(gps.location.lng());
    
    // Sending heart-rate, accelerometer, and gyroscope data
    if (pulseSensor.sawStartOfBeat()) {
      Serial.println(HR_Val);
      Send_Data(Data_Packet, HM10);
    }
  }
  
  // Send_Data(GPS.latitude);
  // Send_Data(GPS.longitude);
  
  delay(20);
  
}
