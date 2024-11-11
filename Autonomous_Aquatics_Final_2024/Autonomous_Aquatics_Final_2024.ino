#include <string.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <ArduinoJson.h>
#include <Ezo_i2c.h>
#include <Ezo_i2c_util.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_PWMServoDriver.h>
#include <PIDController.h>

//----------THE FOLLOWING SETUP IS BOILERPLATE COPIED DIRECTLY FROM MAKERFABS.COM----------
#define LTE_RESET_PIN  6
#define LTE_PWRKEY_PIN 5
#define LTE_FLIGHT_PIN 7
#define SD_SELECT_PIN  4
#define UART_BAUD      115200
#define MODEM_RXD      0
#define MODEM_TXD      1
#define SERVER_IP      "REDACTED_IP_ADDRESS"
#define SERVER_PORT    8080
//----------END OF COPIED BOILERPLATE----------
#define PWM_FREQ       50
#define EARTH_RAD      6371000
#define DEBUG_AT       1

float* waypoints;
int wpCount;
int currentWP = 0;
int home = 0;

bool waitingForUDP = false;
bool incoming = false;

StaticJsonDocument<200> data;
StaticJsonDocument<200> settings;
char output[128];

unsigned long currentMillis;
unsigned long sendMillis;
unsigned long saveMillis;
unsigned long GPSMillis;
unsigned long sensorMillis;
unsigned long ATMillis;
int sendCooldown = 500;
int saveCooldown = 500;
int GPSCooldown = 500;
int sensorCooldown = 500;
int ATCooldown = 50;


Ezo_board PH = Ezo_board(99, "PH");
Ezo_board EC = Ezo_board(100, "EC");
Ezo_board TP = Ezo_board(102, "TP");

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_LIS3MDL lis3mdl;

PIDController pid;

Sd2Card card;
SdVolume volume;
SdFile root;
File wpFile;
File dataFile;

const float hard_iron[3] = {
  16.82,  49.35,  -112.35
};

const float soft_iron[3][3] = {
  {  1.042,  0.023, -0.038  },
  {  0.023,  1.027,  0.039  },
  { -0.038,  0.039,  0.938  }
};

const float mag_decl = -7.22;

float ph = 0;
float ec = 0;
float tp = 0;
float lat = 0;
float lon = 0;

float p = 0.5;
float i = 0;
float d = 0;

int speed = 0;
float differential = 0;
float heading = 0;

//Begin Serial readings, sensor communication, SD communication, and the PID system
void setup(){
  SerialUSB.begin(115200);
  Serial1.begin(115200);
  Wire.begin();
  setupAT();
  PH.send_read_cmd();
  EC.send_read_cmd();
  TP.send_read_cmd();
  initPWM();
  SD.begin(SD_SELECT_PIN);
  lis3mdl.begin_I2C();
  loadWaypoints("wpts.txt", &waypoints, &wpCount);
  dataFile = SD.open("data.txt", FILE_WRITE);
  dataFile.println("latitude,longitude,pH,salinity,temp");
  pid.begin();
  pid.setpoint(0);
  pid.tune(0.5, 0, 0);
  pid.limit(-1, 1);
  sendMillis = saveMillis = sensorMillis = GPSMillis = millis();
}

void loop() {
  //If the vehicle is to home, set the waypoint to zero, otherwise, point it to the current waypoint
  int wp = home ? 0 : currentWP;
  currentMillis = millis();
  handleAT();
  if (currentMillis - sensorMillis > sensorCooldown){
    handleSensor(PH, &ph);
    handleSensor(EC, &ec);
    handleSensor(TP, &tp);
    sensorMillis = currentMillis;
  }
  //Create a JSON packet containing the message type, GPS coordinates, and sensor readings
  data["type"] = "Vehicle";
  data["coord"][0] = lat;
  data["coord"][1] = lon;
  data["sensor"][0] = ph;
  data["sensor"][1] = ec;
  data["sensor"][2] = tp;
  data["currentWP"] = wp;
  data["dist"] = getDistance(lat, lon, waypoints[2*wp], waypoints[1+(2*wp)]);
  if (currentMillis - saveMillis > saveCooldown){
    logData();
    saveMillis = currentMillis;
  }
  float angleToWP = bearing(lat, lon, waypoints[2*wp], waypoints[1+(2*wp)]);
  heading = getHeading();
  differential = pid.compute(headingError(heading, angleToWP));
  differentialThrust(differential, speed);
  if ((getDistance(lat, lon, waypoints[2*wp], waypoints[1+(2*wp)]) < 3) && !home){
    if (currentWP = (wpCount - 1)){
      currentWP = 0;
    }
    currentWP++;
  }
  //SerialUSB.println(differential);
}

void parseGPS(String sentence) {
  lat = (sentence.substring(11, 13).toFloat() + sentence.substring(13, 22).toFloat()/60) * ((sentence.charAt(23) == 'S') ? -1 : 1);
  lon = (sentence.substring(25, 28).toFloat() + sentence.substring(28, 37).toFloat()/60) * ((sentence.charAt(38) == 'W') ? -1 : 1);
}

String readline() {
  char received[256];
  int index = 0;
  char rc;
  while (Serial1.available() > 0){
    rc = Serial1.read();
    if (rc == '\n'){
      received[index] = '\0';
      return String(received);
    }
    else if (rc == '>'){
      received[0] = rc;
      received[1] = '\0';
      return String(received);
    }
    else{
      received[index] = rc;
      index++;
    }
    if (index >= 256){
      return String(received);
    }
  }
}

void handleAT() {
  //Read the incoming GPS and UDP data incoming from the SIM7600
  if (Serial1.available() > 0){
    String currentLine = readline();
    if (DEBUG_AT){
      SerialUSB.println(currentLine);
      //SerialUSB.println("\t lnd");
    }
    if (currentLine.indexOf("+CGPSINFO:") != -1){
      parseGPS(currentLine);
    }
    else if (currentLine.indexOf(">") != -1){
      serializeJson(data, output);
      output[127] = '\0';
      Serial1.print(output);
      Serial1.write(0x1A);
      Serial1.println();
      Serial1.flush();
      //This delay is necessary due to a bug in Arduino SAMD21, which causes a crash upon a serial write of too large a size, no matter the set buffer size
      delay(100);
      waitingForUDP = false;
      ATMillis = currentMillis;
    }
    else if (currentLine.indexOf("+CIPRXGET: 1,0") != -1){
      incoming = true;
    }
    //If we recieve a line containing settings, update the settings
    else if (currentLine[0] == '{'){
      deserializeJson(settings, currentLine.c_str());
      speed = settings["speed"];
      p = settings["pid"][0];
      i = settings["pid"][1];
      d = settings["pid"][2];
      home = settings["home"];
      pid.tune(p, i, d);
    }
  }
  //In this iteration, each SIM7600 operation runs on a cooldown--so that they do not interfere with one another
  if ((currentMillis - ATMillis > ATCooldown) && !waitingForUDP){
    if (currentMillis - sendMillis > sendCooldown){
      Serial1.println("AT+CIPSEND=0,,\"REDACTED_IP_ADDRESS\",8080");
      waitingForUDP = true;
      sendMillis = currentMillis;
      return;
    }
    if (currentMillis - GPSMillis > GPSCooldown){
      Serial1.println("AT+CGPSINFO");
      GPSMillis = currentMillis;
      return;
    }
    if (incoming){
      Serial1.println("AT+CIPRXGET=2,0");
      incoming = false;
      return;
    }
  }
  //SerialUSB.println(waitingForUDP);
}

void setupAT() {
  //----------THE FOLLOWING SETUP IS BOILERPLATE COPIED DIRECTLY FROM MAKERFABS.COM----------
  pinMode(LTE_RESET_PIN, OUTPUT);
  digitalWrite(LTE_RESET_PIN, LOW);

  pinMode(LTE_PWRKEY_PIN, OUTPUT);
  digitalWrite(LTE_RESET_PIN, LOW);
  delay(100);
  digitalWrite(LTE_PWRKEY_PIN, HIGH);
  delay(2000);
  digitalWrite(LTE_PWRKEY_PIN, LOW);
  
  pinMode(LTE_FLIGHT_PIN, OUTPUT);
  digitalWrite(LTE_FLIGHT_PIN, LOW);
  //----------END OF COPIED BOILERPLATE----------

  while(!Serial1.available()){
    Serial1.println("AT");
    delay(200);
  }
  delay(1000);
  //Send some configuration commands to the SIM7600 to connect to internet and begin the GPS
  Serial1.println("AT+CGPS=1");
  delay(1000);
  Serial1.println("AT+NETOPEN");
  delay(1000);
  Serial1.println("AT+CIPRXGET=1");
  delay(2000);
  Serial1.println("AT+CIPOPEN=0,\"UDP\",,,8080");
  delay(1000);
}

//Request a reading from the referenced sensor, and place the result in a variable
void handleSensor(Ezo_board &Device, float *out) {
  Device.receive_read_cmd();
  float tmpOut = Device.get_last_received_reading();
  if (Device.get_error() == Ezo_board::SUCCESS){
    *out = tmpOut;
    Device.send_read_cmd();
  }
}

void initPWM(){
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(PWM_FREQ);
  delay(1000);
  pwm.writeMicroseconds(0, mapSpeed(0)); //left
  pwm.writeMicroseconds(1, mapSpeed(0)); //right
  delay(5000);
}

//Convert a speed value to a PWM value
int mapSpeed(int speed){
  return map(speed, 0, 100, 1000, 2000);
}

//Calculate the speed of each motor, and drive the motors
void differentialThrust(float differential, int speed){
  differential = constrain(differential, -1, 1);
  speed = constrain(speed, 0, 100);
  pwm.writeMicroseconds(0, mapSpeed(constrain(speed+(differential*speed), 0, 100)));
  pwm.writeMicroseconds(1, mapSpeed(constrain(speed-(differential*speed), 0, 100)));
}

float bearing(float aLat, float aLon, float bLat, float bLon){
  aLat = rad(aLat);
  aLon = rad(aLon);
  bLat = rad(bLat);
  bLon = rad(bLon);
  float dLon = bLon - aLon;
  return deg(atan2(sin(dLon) * cos(bLat), cos(aLat) * sin(bLat) - sin(aLat) * cos(bLat) * cos(dLon)));
}

float getDistance(float aLat, float aLon, float bLat, float bLon){
  aLat = rad(aLat);
  aLon = rad(aLon);
  bLat = rad(bLat);
  bLon = rad(bLon);
  float dLat = bLat - aLat;
  float dLon = bLon - aLon;
  float a = (sin(dLat/2) * sin(dLat/2)) + (cos(aLat) * cos(bLat) * sin(dLon/2) * sin(dLon/2));
  float c = 2* atan2(sqrt(a), sqrt(1-a));
  return c * EARTH_RAD;
}

float headingError(float current, float desired){
  float left = current - desired;
  float right = desired - current;
  if (left < 0) left += 360;
  if (right < 0) right += 360;
  return left < right ? -left : right;
}

float rad(float theta){
  return (theta*M_PI)/180;
}

float deg(float theta){
  return (theta*180)/M_PI;
}

void loadWaypoints(const char* file, float** waypoints, int* wpCount){
  char buffer[64];
  char recv;
  int pos = 0;
  int lineCount = 0;
  int wpPos = 0;
  int dataStart;
  wpFile = SD.open(file);
  if (wpFile) {
    while (wpFile.available()){
      if (wpFile.read() == '\n'){
        if (lineCount == 0){
          dataStart = wpFile.position();
        }
        lineCount++;
      }
    }
    *wpCount = lineCount;
    *waypoints = (float*)malloc(lineCount*sizeof(float)*2);
    wpFile.seek(dataStart);
    while (wpFile.available()) {
      recv = wpFile.read();
      if (recv == ',' || recv == '\n'){
        buffer[pos] = '\0';
        //SerialUSB.println(buffer);
        (*waypoints)[wpPos] = atof(buffer);
        pos = 0;
        wpPos++;
        continue;
      }
      buffer[pos] = recv;
      pos++;
    }
    wpFile.close();
  } else {
    SerialUSB.println("Could not open file");
  }
}

//Store the collected data in the SD card
void logData(){
    dataFile.print(lat, 8);
    dataFile.print(",");
    dataFile.print(lon, 8);
    dataFile.print(",");
    dataFile.print(ph, 8);
    dataFile.print(",");
    dataFile.print(ec, 8);
    dataFile.print(",");
    dataFile.println(tp, 8);
    dataFile.flush();
}

//----------THIS FUNCTION IS HEAVILY BASED ON EXAMPLE CODE FOUND ON ADAFRUIT.COM----------
float getHeading(){
  static float hi_cal[3];
  static float heading = 0;
  // Get new sensor event with readings in uTesla
  sensors_event_t event;
  lis3mdl.getEvent(&event);
  float mag_data[] = {event.magnetic.x,
                      event.magnetic.y,
                      event.magnetic.z};
  // Apply hard-iron offsets
  for (uint8_t i = 0; i < 3; i++) {
    hi_cal[i] = mag_data[i] - hard_iron[i];
  }
  // Apply soft-iron scaling
  for (uint8_t i = 0; i < 3; i++) {
    mag_data[i] = (soft_iron[i][0] * hi_cal[0]) +  
                  (soft_iron[i][1] * hi_cal[1]) +
                  (soft_iron[i][2] * hi_cal[2]);
  }
  // Calculate angle for heading, assuming board is parallel to
  // the ground and  Y points toward heading.
  heading = -1 * (atan2(mag_data[0], mag_data[1]) * 180) / M_PI;
  heading += mag_decl + 8;

  if (heading < 0) {
    heading += 360;
  }
  else if (heading > 360){
    heading -= 360;
  }
  return heading;
}
