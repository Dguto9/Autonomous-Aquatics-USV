#include<string.h>

//----------MUCH OF THE FOLLOWING SETUP IS BOILERPLATE COPIED DIRECTLY FROM MAKERFABS.COM----------
#define LTE_RESET_PIN  6
#define LTE_PWRKEY_PIN 5
#define LTE_FLIGHT_PIN 7
#define UART_BAUD      115200
#define MODEM_RXD      0
#define MODEM_TXD      1

String lat;
String lon;
String currentLine;

void setup(){
  SerialUSB.begin(115200);
  Serial1.begin(115200);

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

  SerialUSB.println("GPS Parsing Test:");
  while(!Serial1.available()){
    Serial1.println("AT");
    delay(200);
  }
  delay(1000);
  Serial1.println("AT+CGPS=1");
  delay(1000);
}

void loop() {
  //Read the current line of output from the SIM7600
  currentLine = readline();
  //If the line contains the tag relevant to GPS information, parse the line as GPS coordinates
  if (currentLine.indexOf("+CGPSINFO:") != -1){
    parseGPS(currentLine);
    SerialUSB.println(currentLine);
  }
  //Make a new request for GPS coordinates
  getGPS();
  SerialUSB.println(lat);
  SerialUSB.println(lon);
}

void getGPS() {
  Serial1.println("AT+CGPSINFO");
}

//Parse the string returned by the SIM7600 upon requesting GPS coordinates, to store decimal degree latitude and longitude values
void parseGPS(String sentence) {
  lat = (sentence.substring(11, 13).toFloat() + sentence.substring(13, 22).toFloat()/60) * ((sentence.charAt(23) == 'S') ? -1 : 1);
  lon = (sentence.substring(25, 28).toFloat() + sentence.substring(28, 37).toFloat()/60) * ((sentence.charAt(38) == 'W') ? -1 : 1);
}

String readline() {
  char received[128] = "";
  int index = 0;
  char rc;
  bool done = false;
  //If there is more string to be read, keep reading
  while (done == false){
    while (Serial1.available() > 0){
      //Read the next character of the buffer
      rc = Serial1.read();
        //If it is a newline, terminate the string and return it
        if (rc == '\n'){
          received[index] = '\0';
          done = true;
          return String(received);
        }
        else{
          //Otherwise, continue unless we reach our buffer limit
          received[index] = rc;
          index++;
          if (index >= 128){
            return String(received);
        }
      }
    }
  }
}