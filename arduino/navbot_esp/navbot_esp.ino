#include <SoftwareSerial.h>
#include <Servo.h>
#include <MPU9250_WE.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define SERVO_ROT 190.0
#define SERVO_ROT_DEG_SEC (2 / 60.0) // sec / 60deg
#define SERVO_PIN D6
#define LUNA_DATA_HEADER 0x59

#define MEASUREMENT_MSG_LENGTH 5

#define LUNA_TX D0
#define LUNA_RX D5

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

MeasurementsPacket measurementsPacket;


const char* ssid = "Cleedkamer";
const char* password = "Cleedvermaak123";

SoftwareSerial luna(LUNA_RX, LUNA_TX);
Servo servo;
MPU9250_WE myMPU9250 = MPU9250_WE(0x68);

WiFiClient espClient;
PubSubClient client(espClient);
IPAddress mqttServer(192, 168, 50, 170);

int luna_data[9];
int measurementIndex = 0;

void setup() {
  Serial.begin(115200);
  luna.begin(115200);
  servo.attach(SERVO_PIN, 500, 2500);
  Wire.begin();

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

  
  luna_data[0] = LUNA_DATA_HEADER;
  luna_data[1] = LUNA_DATA_HEADER;
}


void callback(char* topic, byte* payload, unsigned int length) {


  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}


void loop() { 
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  float rotationAtTime = SERVO_ROT - abs(fmod((millis() / 1000.0) / SERVO_ROT_DEG_SEC, 2 * SERVO_ROT) - SERVO_ROT);
  servo.write(rotationAtTime);

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
          measurement.commandId = 0;
          measurement.progress = 0;
          measurement.servoRotation = (map(analogRead(A0), 109, 1000, 180, 0) * 71) / 4068.0; // deg to rad
          measurement.distanceM = distance_cm / 100.0f;

          measurementsPacket.measurements[measurementIndex] = measurement;
          
          measurementIndex++;

          if (measurementIndex == MEASUREMENT_MSG_LENGTH) {
            measurementIndex = 0;
            boolean success = client.publish("sensors/1", measurementsPacket.byteArray, sizeof(measurementsPacket.byteArray));
            if (success) {
              Serial.println("send");
            } else {
              Serial.println("failed");
            }
          }
          
//          Serial.print(map(analogRead(A0), 109, 1000, 180, 0));
//          Serial.print(' ');
//          Serial.print(analogRead(A0));
//          Serial.print(' ');
//          Serial.print(rotationAtTime);
//          Serial.print(' '); 
//          Serial.print(strength);
//          Serial.print(' '); 
//          Serial.println(distance_cm);
        }
      }
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    if (client.connect("arduinoClient")) {
      Serial.println("connected");
      client.subscribe("navigation/1/#");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}
