#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>

#define GPSSerial Serial1
#define DEG2RAD PI/180

const char *ssid = "yourAp";
const char *password = "esp32";

WiFiServer server(80);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

Adafruit_GPS GPS(&GPSSerial);

#define SERVOMIN  205
#define SERVOMAX  410
#define SERVO_FREQ 50

uint32_t timer = millis();

int waypoints[9][2];
float currentx;
float currenty;

int earthRadius = 6366000;

float ref_lat = REDACTED_COORDINATE;
float ref_lon = REDACTED_COORDINATE;

float k = cos(DEG2RAD*ref_lat);
float z = DEG2RAD*earthRadius;
int speed;
bool go = false;
int wpNum;
int currentwaypoint = 0;
int currentAngle = 45;
int changeAmount;
int divisor;
int cap;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(19200);

  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.begin();
  Serial.println("Server started");
  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  delay(1000);

  GPSSerial.println(PMTK_Q_RELEASE);

  //Collect input parameters including waypoint count, waypoint coordinates, speed, and steering parameters
  Serial.print("Number of Waypoints: ");
  while(Serial.available() == 0) {
    }
  wpNum = Serial.parseInt();
  Serial.println(wpNum);

  for (int i = 0; i < wpNum; i++){
    Serial.print("Waypoint " + String(i) + " X:");
    while(Serial.available() == 0) {
    }
    waypoints[i][0] = Serial.parseInt();
    Serial.println(waypoints[i][0]);

    Serial.print("Waypoint " + String(i) + " Y:");
    while(Serial.available() == 0) {
    }
    waypoints[i][1] = Serial.parseInt();
    Serial.println(waypoints[i][1]);
  }

  Serial.print("Speed 0-100: ");
  while(Serial.available() == 0) {
    }
  //Set the vehicle speed to an integer value between 0-100; used to set the drive motor power
  speed = Serial.parseInt();
  Serial.println(speed);

  Serial.print("Servo Angle Change: ");
  while(Serial.available() == 0) {
    }
  //Set the angle at which the rudder should sit while initiating a turn
  changeAmount = Serial.parseInt();
  Serial.println(changeAmount);

  Serial.print("Divisor: ");
  while(Serial.available() == 0) {
    }
  //Set the amount that the distance from the target waypoint impacts the speed--used to slow down when near waypoints
  divisor = Serial.parseInt();
  Serial.println(divisor);

  Serial.print("Cap: ");
  while(Serial.available() == 0) {
    }
  //Set the maximum possible speed that the vehicle should be allowed to attain
  cap = Serial.parseInt();
  Serial.println(cap);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(1000);
  pwm.writeMicroseconds(1, mapSpeed(0));
  delay(5000);
}


void loop()
{
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) 
      return;
  }

  if (millis() - timer > 2000) {
    timer = millis();
    if (GPS.fix) {
      convertCoords();
      Serial.print("Location: ");
      Serial.print(currentx, 4);
      Serial.print(", ");
      Serial.println(currenty, 4);
    }
  }
  //----------MUCH OF THE FOLLOWING CODE IS COPIED DIRECTLY FROM EXAMPLE CODE FOUND ON SPARKFUN.COM----------
  //Check for HTTP requests, send necessary data to client, and execute any commands
  WiFiClient client = server.available();
  if (client) {
    Serial.println("New Client.");
    String currentLine = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        if (c == '\n') {
          if (currentLine.length() == 0) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
            client.print("<a href=\"/O\"><button>RELOAD</button></a>");
            client.println();
            break;
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
         if (currentLine.endsWith("GET /A")) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
            client.print(GPS.latitude, 4);
            client.println();
            break;
        }
         else if (currentLine.endsWith("GET /O")) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
            client.print(GPS.longitude, 4);
            client.println();
            break;
        }
         else if (currentLine.endsWith("GET /S")) {
            go = !go;
            if (!go){
              pwm.writeMicroseconds(1, mapSpeed(0));
            }
        }
      }
    }
    client.stop();
    Serial.println("Client Disconnected.");
  }
  //----------END OF EXAMPLE CODE----------
  //Main navigation loop
  if (go == true){
    float angle = findAngle(currentx, currenty, waypoints[currentwaypoint][0], waypoints[currentwaypoint][1]);
    float GPSangle = GPS.angle;
    //Change state of the rudder position based on the angle error
    if (GPSangle - angle > 0){
      currentAngle -= changeAmount;
      currentAngle = constrain(currentAngle, 0, 90);
      pwm.setPWM(0, 0, mapAngle(currentAngle));
    }
    else if (GPSangle - angle < 0){
      currentAngle += changeAmount;
      currentAngle = constrain(currentAngle, 0, 90);
      pwm.setPWM(0, 0, mapAngle(currentAngle));
    }
    //Send a PWM pulse to the ESC controlling the thrusters, with a duty cycle dependent on the speed setting, distance from waypoint, and divisor setting
    pwm.writeMicroseconds(1, mapSpeed(constrain(speed*(calculateDistance(currentx, currenty, waypoints[currentwaypoint][0], waypoints[currentwaypoint][1])/divisor), 0, cap)));
  }
  //If the vehicle is close enough to the waypoint, pass through to the next waypoint
  if (calculateDistance(currentx, currenty, waypoints[currentwaypoint][0], waypoints[currentwaypoint][1]) < 1){
    if (currentwaypoint < wpNum){
      currentwaypoint ++;
    }
  }
}

//Convert NMEA statement coordinate format to decimal degrees
float decimal_degrees(float x){
  float deg = (int(x)-(int(x) % 100)) / 100;  
  float minutes = x - 100*deg;
  Serial.print("Decimal Degree Location: ");
  Serial.println(deg+(minutes/60));
  return deg+(minutes/60);
}

//Convert the NMEA statement coordinates to the metrical coordinate system with an origin specified by the user
void convertCoords(){
  float dlat = decimal_degrees(GPS.latitude) - ref_lat;
  float dlon = -decimal_degrees(GPS.longitude) - ref_lon;

  currenty = -dlat*z;
  currentx = -dlon*z*k;
}

//Convert servo angle to PWM duty cycle
int mapAngle(int angleIn){
  int pulselength = map(angleIn, 0, 90, SERVOMIN, SERVOMAX);
  return pulselength;
}

//Convert motor speed to PWM duty cycle
int mapSpeed(int speed){
  //ESC takes min PWM at 1000 us and max at 2000 us. Possible stop @ 1500 us.
  int pulselength = map(speed, 0, 100, 1000, 2000);
  Serial.print("Motor PL: ");
  Serial.println(pulselength);
  return pulselength;
}

float findAngle(float currentx, float currenty, float waypointx, float waypointy){
  float dx = waypointx - currentx;
  float dy = waypointy - currenty;
  float angle = degrees(atan2(dy,dx));
  if (angle < 0){
    angle += 180;
  }
  return angle;
}

float calculateDistance(float currentx, float currenty, float waypointx, float waypointy){
  float dist = sqrt(sq(waypointx - currentx) + sq(waypointy - currenty));
  return dist;
}
