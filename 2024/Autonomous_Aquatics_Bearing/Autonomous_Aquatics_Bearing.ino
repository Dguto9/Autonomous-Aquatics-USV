float aLat = 39.099912;
float aLon = -94.581213;
float bLat = 38.627089;
float bLon = -90.200203;

#define EARTH_RAD 6371000

//This program tests the calculation of the bearing, or target angle, of the vehicle

void setup() {
  SerialUSB.begin(115200);
  while (!SerialUSB){
    ;
  }
  SerialUSB.println("Bearing Test:");
  SerialUSB.println(bearing(aLat, aLon, bLat, bLon));
}

void loop() {
  return;
}

float bearing(float aLat, float aLon, float bLat, float bLon){
  aLat = rad(aLat);
  aLon = rad(aLon);
  bLat = rad(bLat);
  bLon = rad(bLon);
  float dLon = bLon - aLon;
  return deg(atan2(sin(dLon) * cos(bLat), cos(aLat) * sin(bLat) - sin(aLat) * cos(bLat) * cos(dLon)));
}

float rad(float theta){
  return (theta*M_PI)/180;
}

float deg(float theta){
  return (theta*180)/M_PI;
}