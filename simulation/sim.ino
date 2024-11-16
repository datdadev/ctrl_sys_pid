#include <TimerOne.h>
#include <TimerThree.h>

float current = 0;
float setpoint = 100; // default

float deltaT = 0.005; // 0.01 / 2

float e = 0;
float ePrev = 0;

float derivative = 0;
float integral = 0;

float Kp = 1;
float Kd = 0;
float Ki = 0;

bool start = false;

const int maxBuffer = 5;
byte buffer[maxBuffer];
int bufferCount = 0;

float output;

void setup() {
  Serial.begin(9600);
  // Serial1.begin(9600);
  Timer1.initialize((int)(deltaT * 1.0e6)); // microseconds
  Timer1.attachInterrupt(timerCallback1);
  Timer3.initialize(1000000);
  Timer3.attachInterrupt(timerCallback2);
}

void loop() {
  // do nothing
}

void timerCallback1() {
  while (Serial.available()) {
    buffer[bufferCount] = (byte) Serial.read();
    if (bufferCount == 0) {
      if (buffer[bufferCount] == 0x00) {
        start = true;
      } else if (buffer[bufferCount] == 0x01) {
        start = false;
        Kp = 1;
        Ki = 0;
        Kd = 0;
        e = 0;
        ePrev = 0;
        derivative = 0;
        integral = 0;
        setpoint = 100;
      } else {
        bufferCount++;
      }
    } else if (bufferCount == 4) {
      if (buffer[0] == 0x02) { // F
        current = *((float*)&buffer[1]);
        // Serial1.println(current);
      } else if (buffer[0] == 0x03) { // SETPOINT
        setpoint = *((float*)&buffer[1]);
      } else if (buffer[0] == 0x04) {
        Kp = *((float*)&buffer[1]);
      } else if (buffer[0] == 0x05) {
        Ki = *((float*)&buffer[1]);
      } else if (buffer[0] == 0x06) {
        Kd = *((float*)&buffer[1]);
      } 

      memset(buffer, 0, sizeof(buffer));
      bufferCount = 0;
    } else {
      bufferCount++;
    }
  }
}

void timerCallback2() {
  if (start) {
    e = setpoint - current;
    derivative = (e - ePrev) / deltaT;
    integral += e * deltaT;
    output = Kp * e + Kd * derivative + Ki * integral;

    if (output > 255) {
        output = 255;
    } else if (output < -255) {
        output = -255;
    }

    output = output / 255 * 12.0;

    ePrev = e;

    // Serial1.println(output);
    Serial.write((byte)0x02);
    byte* bytePtr = (byte*)&output;
    for (int i = 0; i < sizeof(output); i++) {
      Serial.write(bytePtr[i]);
    }
  }
}
