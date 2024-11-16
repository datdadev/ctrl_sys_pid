#define PWM 10
#define IN  12

#define ENCA 21
#define ENCB 20
#define readA bitRead(PIND, 1)
#define readB bitRead(PIND, 0)

#define RATIO 100.0/143 * 1.041 // not met ? decrease : increase

volatile float current = 0;
float setpoint = 100; // Default setpoint

float prevT = 0;

float e = 0;
float ePrev = 0;

float derivative = 0;
float integral = 0;

float Kp = 1;
float Kd = 0;
float Ki = 0;

float output;

const int maxBuffer = 5;
byte buffer[maxBuffer];
int bufferCount = 0;

bool start = false;

float deltaT = 0.0001;

void setup() {
  Serial.begin(9600);

  pinMode(PWM, OUTPUT);
  pinMode(IN, OUTPUT);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB), readEncoderB, CHANGE);
}

void loop() {
  if (Serial.available() > 0) {
    timerCallback1();
  }

  if (start) {
    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / (1.0e6);
    prevT = currT;

    float pos = 0;
    pos = (static_cast<int>(current) >> 2) * RATIO;

    e = setpoint - pos;
    derivative = (e - ePrev) / deltaT;
    integral += e * deltaT;
    output = Kp * e + Kd * derivative + Ki * integral;

    // Ensure PWM is within bounds
    float pwm = fabs(output);
    if (pwm > 255) pwm = 255;
    if (pwm < 40) pwm = 40;

    setMotor(PWM, pwm, (output < 0) ? 0 : 1, IN);
    ePrev = e;

    Serial.write((byte)0x02);
    byte* bytePtr = (byte*)&pos;
    for (int i = 0; i < sizeof(pos); i++) {
      Serial.write(bytePtr[i]);
    }
  } else {
    current = 0;
  }
}

void setMotor(int pwm, int pwmVal, int direction, int in) {
  analogWrite(pwm, pwmVal);
  digitalWrite(IN, direction);
}

void readEncoderA() {
  if (readA != readB) {
    current++;
  } else {
    current--;
  }
}

void readEncoderB() {
  if (readA == readB) {
    current++;
  } else {
    current--;
  }
}

void timerCallback1() { 
  while (Serial.available()) {
    buffer[bufferCount] = (byte) Serial.read();
    if (bufferCount == 0) {
      if (buffer[bufferCount] == 0x00) {
        // current = 0;
        start = true;
      } else if (buffer[bufferCount] == 0x01) {
        start = false;
        setMotor(PWM, 0, (output < 0) ? 0 : 1, IN);
      } else {
        bufferCount++;
      }
    } else if (bufferCount == 4) {
      if (buffer[0] == 0x02) { // F
        current = *((float*)&buffer[1]);
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
