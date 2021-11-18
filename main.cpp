
#include <Arduino.h>
#include <wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN 0 // pulsewidth based
#define SERVOMAX 4096
#define SERVO_FREQ 50 //50hz max update

uint8_t rightThumb = 0; 
uint8_t rightPointer = 2; 
uint8_t rightMiddle = 4; 
uint8_t rightRing = 6; 
uint8_t rightPinky = 8; 
uint8_t rightWrist = 5;





void setup() {
  Serial.begin(9600);
  Serial.println("initializing");
  Serial.println("flux capacitors engaging");
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
}


void setServoPulse(uint8_t n, double pulse){
double pulselength;
pulselength= 1000000;
pulselength /=SERVO_FREQ;
Serial.print(pulselength);
Serial.println("us per period");
pulselength /= 4096;
Serial.print(pulselength); 
Serial.println("us per bit");
pulse *= 1000000;
pulse /= pulselength;
Serial.print(pulse);

}

void loop() {
  // (pin,0,4096) = off 
  // (pin,4096,0) = on
  for (uint8_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(rightThumb,0,pulselen); //thumb
    pwm.setPWM(rightPointer,0,pulselen); //pointer
    pwm.setPWM(rightMiddle,0,pulselen); //middle
    pwm.setPWM(rightRing,0,pulselen); //ring
    pwm.setPWM(rightPinky,0,pulselen); //pinky
    pwm.setPWM(rightWrist,0,pulselen); //wrist rotation @ 180deg
    
  }

  delay(500);
  for (uint8_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(rightThumb,0,pulselen); 
    pwm.setPWM(rightPointer,0,pulselen); 
    pwm.setPWM(rightMiddle,0,pulselen);
    pwm.setPWM(rightRing,0,pulselen);
    pwm.setPWM(rightPinky,0,pulselen);
    pwm.setPWM(rightWrist,0,pulselen);
    

  delay(500);
  }
}
