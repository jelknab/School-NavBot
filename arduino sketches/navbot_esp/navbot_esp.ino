#include <SoftwareSerial.h>
#include <Servo.h>
#include <MPU9250_WE.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define SERVO_ROT 190.0
#define SERVO_ROT_DEG_SEC (2 / 60.0) // sec / 60deg
#define SERVO_PIN D6

#define MEASUREMENT_MSG_LENGTH 50

#define LUNA_TX D0
#define LUNA_RX D5
#define LUNA_DATA_HEADER 0x59

#define ARDUINO_RX D7
#define ARDUINO_TX D8
#define ARDUINO_DATA_HEADER 0x42

typedef struct {
  byte commandId;
  float progress;
  float servoRotation;
  float distanceM;
} Measurement_t;

typedef union MeasurementsPacket {
  Measurement_t measurements[MEASUREMENT_MSG_LENGTH];
  byte byteArray[sizeof(Measurement_t) * MEASUREMENT_MSG_LENGTH];
};

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

typedef struct MotorProgress {
  byte commandId;
  float progress;
};

MeasurementsPacket measurementsPacket;
CommandPacket commandPacket;
ProgressPacket progressPacket;
MotorProgress motorProgress;
bool serialSwapped = false;


const char* ssid = "navbot wifi";
const char* password = "jellybeans";

SoftwareSerial luna(LUNA_RX, LUNA_TX);
Servo servo;
MPU9250_WE myMPU9250 = MPU9250_WE(0x68);

WiFiClient espClient;
PubSubClient client(espClient);
IPAddress mqttServer(192, 168, 4, 2);

int luna_data[9];
int measurementIndex = 0;

void setup() {
  Serial.begin(9600);
  luna.begin(115200);
  servo.attach(SERVO_PIN, 500, 2500);

  commandPacket.command.id = 0;

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print('.');
  }
  
  Serial.println();
  Serial.println("Connection established!");  
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());

  client.setServer(mqttServer, 10000);
  client.setCallback(callback);

  if (!client.setBufferSize(1024)) {
    Serial.println("Could not allocate mqtt buffer size.");
  }

  Serial.println("Swapping to serial2 after 5000ms when connected");
  delay(5000);
  
  luna_data[0] = LUNA_DATA_HEADER;
  luna_data[1] = LUNA_DATA_HEADER;
}


void callback(char* topic, byte* payload, unsigned int length) {
  for (int i = 0; i < length; i++) {
    commandPacket.byteArray[i] = payload[i];
  }

  sendArduinoCommand();
}

void sendArduinoCommand() {
  Serial.write(ARDUINO_DATA_HEADER);
  Serial.print(commandPacket.command.id);
  Serial.print(",");
  Serial.print(commandPacket.command.command);
  Serial.print(",");
  Serial.print(commandPacket.command.val, 4);
  Serial.println();
  delay(50);
}

unsigned long lastRequestTimeStamp = 0;

void loop() {   
  if (!client.connected()) {
    reconnect();
  }

  client.loop();
  
  float rotationAtTime = SERVO_ROT - abs(fmod((millis() / 1000.0) / SERVO_ROT_DEG_SEC, 2 * SERVO_ROT) - SERVO_ROT);
  servo.write(rotationAtTime);

  if (Serial.available()) {
    if (Serial.read() == ARDUINO_DATA_HEADER) {
      motorProgress.commandId = Serial.readStringUntil(',').toInt();
      motorProgress.progress = Serial.readStringUntil('\n').toFloat();

      if (motorProgress.commandId == commandPacket.command.id) {
        if (abs(motorProgress.progress - commandPacket.command.val) < 0.025) {
          if (millis() - lastRequestTimeStamp > 1000) {
            lastRequestTimeStamp = millis();
            byte boredCommand[2] = {0, commandPacket.command.id};
            client.publish("robots/1", boredCommand, sizeof(boredCommand));
          }
        }
      } else {
        sendArduinoCommand();
      }
    }
  }

  if (luna.available()) {
    if (luna.read() == LUNA_DATA_HEADER) {
      if (luna.read() == LUNA_DATA_HEADER) {
        
        for (int i = 2; i < 9; i++) { luna_data[i] = luna.read(); }

        int check_sum = luna_data[0] + luna_data[1] + luna_data[2] + luna_data[3] + luna_data[4] + luna_data[5] + luna_data[6] + luna_data[7];
        
        if (luna_data[8] == (check_sum & 0xff)) {
          int distance_cm = luna_data[2] + luna_data[3] * 256;
          int strength = (luna_data[4] + luna_data[5] * 256) / 65535.0 * 100;
          float temperature = (luna_data[6] + luna_data[7] * 256.0) / 8.0 - 256.0;
          if (distance_cm == 0) return;

          Measurement_t measurement;
          measurement.commandId = commandPacket.command.id;
          measurement.progress = 0;
          measurement.servoRotation = (map(analogRead(A0), 109, 1000, 180, 0) * 71) / 4068.0; // deg to rad
          measurement.distanceM = distance_cm / 100.0f;

          measurementsPacket.measurements[measurementIndex] = measurement;
          
          measurementIndex++;

          if (measurementIndex == MEASUREMENT_MSG_LENGTH) {
            measurementIndex = 0;
            boolean success = client.publish("sensors/1", measurementsPacket.byteArray, sizeof(measurementsPacket.byteArray));
          }
        }
      }
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect("arduinoClient")) {
      if (!serialSwapped) {
        serialSwapped = true;
        Serial.swap();
      }
      
      client.subscribe("navigation/1/#");
    } else {
      delay(5000);
    }
  }
}
