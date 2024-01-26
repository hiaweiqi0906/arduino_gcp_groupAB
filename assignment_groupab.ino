#include <ESP32Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define MOTOR_PIN 40

const char* MQTT_SERVER = "35.240.229.185";
const int MQTT_PORT = 1883;
const char* MQTT_TOPIC = "iot";

const char* TrashDoorIRSensorID = "ed9cbe85-3682-4e9b-9abf-f8b0ae944675";
const char* TrashInnerIRSensorID = "b58e057f-eff7-4191-a18f-3f475792dbbb";
const char* TrashDoorServoID = "9abba489-3b2f-4cb7-b98d-6ff6f423d715";
const char* TrashInnerLEDID = "b514c9ec-d7d1-413a-a739-0fec2168df4b";

Servo servoMotor;
int pos = 0;
int TrashDoorIRSensor = 21;
int TrashInnerIRSensor = 14;
int LED_PIN = 47;
int delay_period = 1000;
unsigned long time_now = 0;
bool status = false, ready = true;

const char* ssid = "BaskinRobbins";
const char* password = "What7YouSay?";

unsigned long previousMillis = 0;
unsigned long previous15Millis = 0;
unsigned long previousSentServerMillis = 0;
unsigned long currentMillis = 0;
unsigned long lastMsgTime = 0;

const long interval = 1000;
const long INTERVAL = 1000;

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);
  pinMode(TrashDoorIRSensor, INPUT);
  pinMode(TrashInnerIRSensor, INPUT);
  pinMode(LED_PIN, OUTPUT);
  servoMotor.attach(MOTOR_PIN);
  WiFi.begin(ssid, password);

  Serial.println("\nConnecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("\nConnected to the WiFi network");
  Serial.print("Maker Feather AIoT S3 IP: ");
  Serial.println(WiFi.localIP());

  client.setServer(MQTT_SERVER, MQTT_PORT);
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("Connected to MQTT server");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void openBin() {
  // rotates from 120 degrees to 0 degrees
  for (int pos = 120; pos >= 0; pos -= 1) {
    if (currentMillis - previous15Millis >= 15) {
      servoMotor.write(pos);
      previous15Millis = currentMillis;
    }
  }
}

void closeBin() {
  // rotates from 0 degrees to 120 degrees
  for (int pos = 0; pos <= 120; pos += 1) {
    // in steps of 1 degree
    if (currentMillis - previous15Millis >= 15) {
      servoMotor.write(pos);
      previous15Millis = currentMillis;
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  int TrashDoorIRSensorStatus = digitalRead(TrashDoorIRSensor);
  if (TrashDoorIRSensorStatus == 1)
  {
    currentMillis = millis();

    if (status) {
      previousMillis = currentMillis;
      ready = true;
      status = !status;
    } else {
      if (ready && currentMillis - previousMillis >= interval) {
        previous15Millis = previousMillis;
        closeBin();
        ready = false;
        previousMillis = currentMillis;
        char payload[10];
        client.publish(MQTT_TOPIC, "hand leaving");
      }
    }
  } else {
    if (!status) {
      status = !status;
      openBin();
      char payload[10];
      client.publish(MQTT_TOPIC, "hand detected");
    }
  }
  int TrashInnerIRSensorStatus = digitalRead(TrashInnerIRSensor);  
  if (TrashInnerIRSensorStatus == 1)                               
  {
    digitalWrite(LED_PIN, LOW);  
  } else {
    digitalWrite(LED_PIN, HIGH); 
    if (!status) {
      status = !status;
      openBin();
      char payload[10];
      client.publish(MQTT_TOPIC, "full bin");
    }
  }
}