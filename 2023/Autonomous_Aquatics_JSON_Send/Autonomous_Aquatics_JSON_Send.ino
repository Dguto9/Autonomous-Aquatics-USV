#include<string.h>
#include <ArduinoJson.h>

//----------THE FOLLOWING SETUP IS BOILERPLATE COPIED DIRECTLY FROM MAKERFABS.COM----------
#define LTE_RESET_PIN  6
#define LTE_PWRKEY_PIN 5
#define LTE_FLIGHT_PIN 7
#define UART_BAUD      115200
#define MODEM_RXD      0
#define MODEM_TXD      1
#define SERVER_IP      "REDACTED_IP_ADDRESS"
#define SERVER_PORT    20001
//----------END OF COPIED BOILERPLATE----------

float lat;
float lon;
String currentLine;  
char output[128];

bool waitingForUDP = false;
int sendToggle = true;
bool incoming = false;

StaticJsonDocument<200> doc;

unsigned long prevMillis;
unsigned long currentMillis;

void setup(){
  //----------THE FOLLOWING SETUP IS BOILERPLATE COPIED DIRECTLY FROM MAKERFABS.COM----------
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

  SerialUSB.println("Send JSON Over Network:");
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
  Serial1.println("AT+CIPOPEN=0,\"UDP\",,,20001");
  delay(1000);
  SerialUSB.println("Setup Complete");
  prevMillis = millis();
}

void loop() {
  currentMillis = millis();
  //Read the incoming GPS and UDP data incoming from the SIM7600
  if (Serial1.available() > 0){
    currentLine = readline();
    SerialUSB.println(currentLine);
    if (currentLine.indexOf("+CGPSINFO:") != -1){
      parseGPS(currentLine);
    }
    else if (currentLine.indexOf(">") != -1){
      Serial1.print(String(output));
      Serial1.write(0x1A);
      Serial1.println();
      waitingForUDP = false;
    }
    else if (currentLine.indexOf("+CIPRXGET: 1,0") != -1){
      incoming = true;
    }
  }
  //Every half-second (if we aren't waiting for the opportunity to write a UDP message), either request GPS info, request to send a UDP message, or request to see recieved messages
  if ((currentMillis - prevMillis >= 500) && waitingForUDP == false)
  {
    if (sendToggle == 0){
      Serial1.println("AT+CGPSINFO");
    }
    else if (sendToggle == 1){
      Serial1.println("AT+CIPSEND=0,,\"REDACTED_IP_ADDRESS\",20001");
      waitingForUDP = true;
    }
    else if (sendToggle == 2 && incoming == true){
      Serial1.println("AT+CIPRXGET=2,0");
      incoming = false;
    }
    prevMillis = currentMillis;
    if (sendToggle == 2){
      sendToggle = 0;
    }
    else{
      sendToggle++;
    }
  }
  //Store the most recently read position in a JSON container, which can be sent over UDP with ease
  doc["lon"] = lon;
  doc["lat"] = lat;
  serializeJson(doc, output);
  //SerialUSB.println(output);
}

void parseGPS(String sentence) {
  lat = (sentence.substring(11, 13).toFloat() + sentence.substring(13, 22).toFloat()/60) * ((sentence.charAt(23) == 'S') ? -1 : 1);
  lon = (sentence.substring(25, 28).toFloat() + sentence.substring(28, 37).toFloat()/60) * ((sentence.charAt(38) == 'W') ? -1 : 1);
}

String readline() {
  char received[128] = "";
  int index = 0;
  char rc;
  bool done = false;
  while (done == false){
    if (Serial1.available() > 0){
      rc = Serial1.read();
        if (rc == '\n'){
          received[index] = '\0';
          done = true;
          return String(received);
        }
        else if (rc == '>'){
          done = true;
          received[index] = rc;
          return String(received);
        }
        else{
          received[index] = rc;
          index++;
          if (index >= 128){
            return String(received);
        }
      }
    }
  }
}