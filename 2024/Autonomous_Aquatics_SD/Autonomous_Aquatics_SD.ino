#include<stdio.h>
#include<string.h>
#include <SPI.h>
#include <SD.h>

#define DEBUG true
#define PIN_SD_SELECT 4

Sd2Card card;
SdVolume volume;
SdFile root;

File wpFile;
float* waypoints;
int wpCount;

void setup()
{
  SerialUSB.begin(115200);
  while (!SerialUSB){
    ;
  }
  
  SD.begin(PIN_SD_SELECT)
  delay(1000);

  loadWaypoints("wpts.txt", &waypoints, &wpCount);
  for (int i = 0; i < wpCount; i++){
    SerialUSB.println(waypoints[i]);
  }
}

void loop () {
  return;
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