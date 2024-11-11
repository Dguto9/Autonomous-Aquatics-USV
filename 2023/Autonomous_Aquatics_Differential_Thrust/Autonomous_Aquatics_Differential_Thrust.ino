#include <Adafruit_PWMServoDriver.h>

#define SERVO_FREQ 50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int speed = 0;
float differential = 0;  

void setup() {
  SerialUSB.begin(115200);
  while(!SerialUSB){
  }
  SerialUSB.println("Differential Thrust Test:");
  SerialUSB.println("Input speed (0-100):");
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(1000);
  pwm.writeMicroseconds(0, mapSpeed(0)); //left
  pwm.writeMicroseconds(1, mapSpeed(0)); //right
  delay(5000);
}

void loop() {
  //Collect speed and differential value, and set the thrust of each motor accordingly
  while(SerialUSB.available() == 0) {
    }
  speed = SerialUSB.parseInt();
  SerialUSB.println(speed);
  while(SerialUSB.available() == 0) {
    }
  differential = SerialUSB.parseFloat();
  SerialUSB.println(differential);
  differentialThrust(differential, speed);
}

//Convert a speed value to a PWM value
int mapSpeed(int speed){
  int pulselength = map(speed, 0, 100, 1000, 2000);
  return pulselength;
}

//Calculate the speed of each motor, and drive the motors
void differentialThrust(float differential, int speed){
  differential = constrain(differential, -50, 50);
  pwm.writeMicroseconds(0, mapSpeed(speed+(differential*10)));
  pwm.writeMicroseconds(1, mapSpeed(speed-(differential*10)));
}