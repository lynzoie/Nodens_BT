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
int HR_Val = 0;                         // holds the incoming raw data. Signal value can range from 0-1024
int Threshold = 560;                // Determine which Signal to "count as a beat", and which to ingore.
PulseSensorPlayground pulseSensor;  // Create PulseSensorPlayground object

// GPS Module variables
#include <TinyGPS++.h>
static const int RXPin = 6, TXPin = 5;
float gps_lat = 0;
float gps_lng = 0;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);    // The serial connection to the GPS device

void setup() {
  Serial.begin(9600);
  Serial.println("Starting Nodens...");
  //////////// HEART RATE SENSOR SETUP //////////////
  pulseSensor.analogInput(PulseWire);
  pulseSensor.blinkOnPulse(LED13);   // pin that will blink to your heartbeat!
  pulseSensor.setThreshold(Threshold);
//  pulseSensor.begin();

  ///////////////// BLUETOOTH SETUP /////////////////
  HM10.begin(9600);
  HM10.write("Connection Established");   // indicate BT-Phone connection has been made
  
  ////////// ACCELEROMETER/GYROSCOPE SETUP //////////
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
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

void loop() {
  // setup Bluetooth to listen to request to send data
  HM10.listen();
  while (HM10.available() > 0) {   // if HM10 sends something then read
    appData = HM10.read();
    inData = String(appData);      // save the data in string format
    Serial.write(appData);
  }
  if (Serial.available()) {        // Read user input if available.
    delay(10);
    HM10.write(Serial.read());
  }
  if (inData == "S"){
    HM10.write("Sending packets...");
  }
  inData = "S";
  while ( inData == "S" && inData != "T") {     // send packets when start condition is given "S", stop when stop condition is given "T"
    // get heart-rate data
    pulseSensor.begin();
    HR_Val = pulseSensor.getBeatsPerMinute();
 
    // get values of Accelerometer/Gyroscope
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);  

    // get GPS data
//    ss.listen(); 
//    while (ss.available() > 0){
//      gps.encode(ss.read());
//      if (gps.location.isUpdated()){ 
//        gps_lat = gps.location.lat();       // store lattitude in double variable
//        gps_lng = gps.location.lng();       // store longitude in double variable
//      }
//    }
    // Test GPS Values
    gps_lat = 39.1317078;
    gps_lng = -84.5174585;
//    Serial.println(gps_lat,7);
//    Serial.println(gps_lng,7);
    if (gps_lat != 0 && gps_lng != 0) {     // if we get valid GPS data, get ready to send data packet via BT
      // Creating data packets to send to Bluetooth module
      String Data_Packet = "FFFFHR" + String(HR_Val) + "AX" + String(a.acceleration.x) + "AY" + String(a.acceleration.y) + "AZ" + String(a.acceleration.z) + "GX" + String(g.gyro.x) + "GY" + String(g.gyro.y) + "GZ" + String(g.gyro.z) + "LA" + String(gps_lat,7) + "LO" + String(gps_lng,7) + "DDDD";
      HM10.listen();                        // listen to BT again so we can send data ports
//      Send_Data(Data_Packet, HM10);         // assume we're getting heart rate for now, use below if statement
      if (pulseSensor.sawStartOfBeat()) {   // send packets only if we get valid heart beat
//        Serial.println(HR_Val);
        Send_Data(Data_Packet, HM10);
      } 
      delay(20);
      // Listen to BT port to see if we need to exit
      appData = HM10.read();
      String tempData = String(appData);    // check what inData value we're getting
      if (tempData == "T"){
        inData = "T";
        HM10.write("Stopping packets...");
      }
      else {
        inData = "S";
      }
    }
  }
}
