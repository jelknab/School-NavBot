#include <RotaryEncoder.h>
#include <NeoSWSerial.h>

#define LEFT_PWM_A_PIN 5
#define LEFT_PWM_B_PIN 6
#define LEFT_EN_PIN 7
#define LEFT_CURRENT_PIN A0
#define LEFT_DIAG_PIN A2
#define LEFT_ENC_A_PIN 3
#define LEFT_ENC_B_PIN 4

#define RIGHT_PWM_A_PIN 9
#define RIGHT_PWM_B_PIN 10
#define RIGHT_EN_PIN 8
#define RIGHT_CURRENT_PIN A1
#define RIGHT_DIAG_PIN A3
#define RIGHT_ENC_A_PIN 11
#define RIGHT_ENC_B_PIN 12

#define ESP_RX 2
#define ESP_TX 13
#define ESP_DATA_HEADER 0x42

#define BATTERY_PIN A5

#define ACCELERATION_M_S 0.05f
#define MAX_SPEED_M_S 0.2f
#define ACCELERATION_TIME MAX_SPEED_M_S / ACCELERATION_M_S
#define ACCELERATION_DISTANCE 0.5 * ACCELERATION_M_S * ACCELERATION_TIME * ACCELERATION_TIME
#define MOTOR_ENCODER_TICKS 816.335f
#define WHEEL_CIRCUMFERENCE (2.0f * PI * 0.04f)

typedef struct {
  byte id;
  char command;
  float val;
} Command_t;

typedef union CommandPacket {
  Command_t command;
  byte byteArray[sizeof(Command_t)];
};

typedef union ProgressPacket {
  float progress;
  byte byteArray[sizeof(progress)];  
};

struct motorDistances {
  float leftDistance;
  float rightDistance;
  float progress;
};

class PID {
  public:
    PID(float kp, float ki, float kd) {
      this->kp = kp;
      this->ki = ki;
      this->kd = kd;
      this->integral = 0;
      this->timeStamp = millis();      
    }

    float calculate(float setpoint, float measurement) {
      double secondsPassed = (millis() - this->timeStamp) / 1000.0f;
      this->timeStamp = millis();

      float error = setpoint - measurement;
      float proportional = error;
      this->integral += error * secondsPassed;
      float derivative = (error - this->errorPrior) / secondsPassed;
      float output = this->kp * proportional + this->ki * integral + this->kd * derivative;
      this->errorPrior = error;

      return output;
    }

    float kp;
    float ki;
    float kd;
    float errorPrior;
    float integral;
    unsigned long timeStamp;
};

class Motor {
  public:
    Motor::Motor(uint8_t pwmPinA, uint8_t pwmPinB, uint8_t encPinA, uint8_t encPinB): encoder(encPinA, encPinB, RotaryEncoder::LatchMode::TWO03) {
      this->pwmPinA = pwmPinA;
      this->pwmPinB = pwmPinB;
    }

    void resetEncoder() {
      this->encoder._position = 0;
      this->encoder._positionExt = 0;
      this->encoder._positionExtPrev = 0;
    }

    float distance() {
      return this->encoder.getPosition() / MOTOR_ENCODER_TICKS * WHEEL_CIRCUMFERENCE;
    }

    float rpm() {
      double secondsPassed = (micros() - this->rpmTimeStamp) / 1000.0d / 1000.0d;

      long encoderPosition = this->encoder.getPosition();
      long positionDifference = this->encoder.getPosition() - this->rpmPositionPrior;
      double rpm = (positionDifference / MOTOR_ENCODER_TICKS) / secondsPassed * 60.0d;

      if (positionDifference != 0) {
        this->rpmPositionPrior = encoderPosition;
        this->rpmTimeStamp = micros();
      }

      return rpm;
    }


    uint8_t pwmPinA;
    uint8_t pwmPinB;
    RotaryEncoder encoder;
    long rpmPositionPrior = 0;
    unsigned long rpmTimeStamp;
};

Motor motorLeft(LEFT_PWM_A_PIN, LEFT_PWM_B_PIN, LEFT_ENC_A_PIN, LEFT_ENC_B_PIN);
PID motorLeftPid(2500, 500, 250);
Motor motorRight(RIGHT_PWM_A_PIN, RIGHT_PWM_B_PIN, RIGHT_ENC_A_PIN, RIGHT_ENC_B_PIN);
PID motorRightPid(2500, 500, 250);

CommandPacket commandPacket;
ProgressPacket progressPacket;

NeoSWSerial esp(ESP_RX, ESP_TX);


void setup() {
  Serial.begin(9600);
  esp.begin(9600);
  setupIO();
  digitalWrite(LEFT_EN_PIN, HIGH);
  digitalWrite(RIGHT_EN_PIN, HIGH);

  commandPacket.command.id = 0;
}

void setupIO() {
  PCIFR = B00000000;
  PCMSK0 |= (1 << PCINT3) | (1 << PCINT4);
  PCMSK2 |= (1 << PCINT19) | (1 << PCINT20);
  PCICR |= (1 << PCIE2) | (1 << PCIE0);
  attachInterrupt(digitalPinToInterrupt(ESP_RX), ESP_ISR, CHANGE);

  pinMode(LEFT_PWM_A_PIN, OUTPUT);
  pinMode(LEFT_PWM_B_PIN, OUTPUT);
  pinMode(LEFT_EN_PIN, OUTPUT);
  pinMode(LEFT_CURRENT_PIN, INPUT);
  pinMode(LEFT_DIAG_PIN, INPUT);

  pinMode(RIGHT_PWM_A_PIN, OUTPUT);
  pinMode(RIGHT_PWM_B_PIN, OUTPUT);
  pinMode(RIGHT_EN_PIN, OUTPUT);
  pinMode(RIGHT_CURRENT_PIN, INPUT);
  pinMode(RIGHT_DIAG_PIN, INPUT);
}

unsigned long lastCommandTimestamp = 0;

float batterySmooth = 6.0;
void loop() {
  if (esp.available()) {
    if (esp.read() == ESP_DATA_HEADER) {
      commandPacket.command.id = esp.readStringUntil(',').toInt();
      commandPacket.command.command = esp.readStringUntil(',').charAt(0);
      commandPacket.command.val = esp.readStringUntil('\n').toFloat();

      motorLeft.resetEncoder();
      motorRight.resetEncoder();
      motorLeftPid.integral = 0;
      motorRightPid.integral = 0;
      lastCommandTimestamp = millis();

      Serial.print("Received command: ");
      Serial.print(commandPacket.command.id);
      Serial.print(", ");
      Serial.print(commandPacket.command.command);
      Serial.print(", ");
      Serial.println(commandPacket.command.val, 5);
    }
  }

  float battery = analogRead(BATTERY_PIN) * (readVcc() / 1024.0) * 2;

  batterySmooth = batterySmooth * 0.9 + battery * 0.1;
  if (batterySmooth < 6.5) {
    Serial.print("Battery low: ");
    Serial.print(batterySmooth);
    Serial.println("V");
    digitalWrite(LEFT_EN_PIN, LOW);
    digitalWrite(RIGHT_EN_PIN, LOW);
    return;
  } else {
    digitalWrite(LEFT_EN_PIN, HIGH);
    digitalWrite(RIGHT_EN_PIN, HIGH);
  }

  if (commandPacket.command.id == 0) {
    Serial.println("NO COMMAND");
    esp.write(ESP_DATA_HEADER);
    esp.print(commandPacket.command.id);
    esp.print(",");
    esp.print(0.0f, 5);
    esp.println();
    delay(100);
    return;
  }

  int maxSpeed = (6.0f / batterySmooth) * 255;
  struct motorDistances distances = getMotorDistances();
  struct motorDistances totalDistances = getTotalDistances();

  float motorLeftDistance = motorLeft.distance();
  float leftPID = motorLeftPid.calculate(distances.leftDistance, motorLeftDistance);

  float motorRightDistance = motorRight.distance();
  float rightPID = motorRightPid.calculate(distances.rightDistance, motorRightDistance);

  if (abs(totalDistances.leftDistance - motorLeftDistance) > 0.025) {
    setMotorSpeed(leftPID, motorLeft, maxSpeed);
  } else {
    setMotorSpeed(0, motorLeft, maxSpeed);
  }

  if (abs(totalDistances.rightDistance - motorRightDistance) > 0.025) {
    setMotorSpeed(rightPID, motorRight, maxSpeed);
  } else {
    setMotorSpeed(0, motorRight, maxSpeed);
  }

//  Serial.print("Writing progress: ");
//  Serial.println(distances.progress);

  float progress = ((motorRightDistance / totalDistances.rightDistance + motorLeftDistance / totalDistances.leftDistance) / 2.0f) * commandPacket.command.val;

  esp.write(ESP_DATA_HEADER);
  esp.print(commandPacket.command.id);
  esp.print(",");
  esp.print(progress, 4);
  esp.println();
  delay(10);
}

motorDistances getTotalDistances() {
  struct motorDistances distances;
  float distance;
  
  switch (commandPacket.command.command) {
    case 'f':
      distances.leftDistance = commandPacket.command.val;
      distances.rightDistance = commandPacket.command.val;
      return distances;

    case 'r':
      float rotation_distance = WHEEL_CIRCUMFERENCE / (PI * 2.0f) * commandPacket.command.val;
      distances.leftDistance = -rotation_distance;
      distances.rightDistance = rotation_distance;
      return distances;
  }
}

motorDistances getMotorDistances() {
  struct motorDistances distances;
  float distance;
  
  switch (commandPacket.command.command) {
    case 'f':
      distance = getExpectedDistance(lastCommandTimestamp, commandPacket.command.val);
      distances.leftDistance = distance;
      distances.rightDistance = distance;
      distances.progress = distance;
      return distances;

    case 'r':
      float rotation_distance = WHEEL_CIRCUMFERENCE / (PI * 2.0f) * commandPacket.command.val;
      distance = getExpectedDistance(lastCommandTimestamp, rotation_distance);
      distances.leftDistance = -distance;
      distances.rightDistance = distance;
      distances.progress = (distance / rotation_distance) * commandPacket.command.val;
      return distances;
  }
}

void setMotorSpeed(int s, Motor &motor, int maxSpeed) {
  s = constrain(s, -maxSpeed, maxSpeed);
  float rpm = motor.rpm();

  uint8_t forward = motor.pwmPinA;
  uint8_t backward = motor.pwmPinB;
  if (s < 0) {
    forward = motor.pwmPinB;
    backward = motor.pwmPinA;
  }

  if (abs(s) > maxSpeed / 4 && rpm == 0.0) {
    analogWrite(backward, 0);
    analogWrite(forward, maxSpeed);
    return;
  }

  analogWrite(backward, 0);
  analogWrite(forward, abs(s));
}

static inline int8_t sgn(float val) {
  if (val < 0) return 1;
  if (val == 0) return 0;
  return -1;
}

float getExpectedDistance(unsigned long timestampMillis, float distanceM) {
  double t = (millis() - timestampMillis) / 1000.0;

  if (abs(distanceM / 2.0f) > ACCELERATION_DISTANCE) {
    return getExpectedDistanceFullAcceleration(t, abs(distanceM)) * sgn(distanceM);
  }

  return getExpectedDistancePartialAcceleration(t, abs(distanceM)) * sgn(distanceM);
}

float getExpectedDistanceFullAcceleration(double t, float distanceM) {
  float constantDistance = distanceM - ACCELERATION_DISTANCE * 2.0;
  float constantDuration = constantDistance / MAX_SPEED_M_S;

  float accelerationT = min(t, ACCELERATION_TIME);
  float decelerationT = max(0, min(t - ACCELERATION_TIME - constantDuration, ACCELERATION_TIME));
  float constantT = max(0, min(t - ACCELERATION_TIME, constantDuration));

  float acceleration = 0.5 * ACCELERATION_M_S * accelerationT * accelerationT;
  float deceleration = MAX_SPEED_M_S * decelerationT - ACCELERATION_M_S / 2.0 * decelerationT * decelerationT;
  float constant = MAX_SPEED_M_S * constantT;

  return acceleration + constant + deceleration;
}

float getExpectedDistancePartialAcceleration(double t, float distanceM) {
  float accelerationDistance = distanceM / 2.0;
  float accelerationDuration = sqrt((2 * accelerationDistance) / ACCELERATION_M_S);
  float accelerationTopSpeed = ACCELERATION_M_S * accelerationDuration;

  float accelerationT = min(t, accelerationDuration);
  float decelerationT = min(min(max(t - accelerationDuration, 0), accelerationDuration), accelerationDuration * 2);

  float acceleration = 0.5f * ACCELERATION_M_S * accelerationT * accelerationT;
  float deceleration = accelerationTopSpeed * decelerationT - ACCELERATION_M_S / 2.0 * decelerationT * decelerationT;

  return acceleration + deceleration;
}

void motorCurrent(uint8_t pin) {
  return 10.0 / 1024.0 * analogRead(pin);
}

float readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1125300L / result; // Back-calculate AVcc in mV
  return result / 1000.0f;
}

ISR(PCINT0_vect) { // RIGHT_ENCODER CHANGE
  motorRight.encoder.tick();
}

ISR(PCINT2_vect) { // LEFT_ENCODER CHANGE
  motorLeft.encoder.tick();
}

void ESP_ISR() {
  NeoSWSerial::rxISR( *portInputRegister( digitalPinToPort( ESP_RX ) ) );
}
