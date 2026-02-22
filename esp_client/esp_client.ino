#include <MAX30105.h>
#include <BMI160Gen.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "secrets.h"

//WiFi Connection Variables
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
#define LED_PIN 10
#define SDA_PIN 4
#define SCL_PIN 5
#define BMI160_I2C_ADDR 0x68

//Pulse Sensor Variables
int LED = 15;   //  The on-board Arduion LED
double average_pulse = 0;
int iterations_pulse = 20;
int count_p = iterations_pulse;
int sig;                // holds the incoming raw data. Signal value can range from 0-1024

MAX30105 pulseSensor;
byte ps_iterations = 4;

unsigned long lastTimeMeasured_movement = millis();
unsigned long lastTimeMeasured_pulse = millis();
float averages_gyro[] = {0.0, 0.0, 0.0};
int iterations_gyro = 20;
int count_m = iterations_gyro;

void blink_led(unsigned int times, unsigned int duration){
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(duration);
    digitalWrite(LED_PIN, LOW); 
    delay(200);
  }
}

void setup_wifi() {
  delay(50);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(SSID);

  WiFi.begin(SSID, PASSWORD);

  int c=0;
  while (WiFi.status() != WL_CONNECTED) {
    blink_led(2,200); //blink LED twice (for 200ms ON time) to indicate that wifi not connected
    delay(1000); //
    Serial.print(".");
    c=c+1;
    if(c>10){
        ESP.restart(); //restart ESP after 10 seconds
    }
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
}

void connect_mqttServer() {
  // Loop until we're reconnected
  while (!client.connected()) {

        //first check if connected to wifi
        if(WiFi.status() != WL_CONNECTED){
          //if not connected, then first connect to wifi
          setup_wifi();
        }

        //now attemt to connect to MQTT server
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect("ESP32_client1")) { // Change the name of client here if multiple ESP32 are connected
          //attempt successful
          Serial.println("connected");
          // Subscribe to topics here
          client.subscribe("rpi/broadcast");
          //client.subscribe("rpi/xyz"); //subscribe more topics here
          
        } 
        else {
          //attempt not successful
          Serial.print("failed, rc=");
          Serial.print(client.state());
          Serial.println(" trying again in 800 ms");
    
          blink_led(3,200); //blink LED three times (200ms on duration) to show that MQTT server connection attempt failed
          // Wait 2 seconds before retrying
          delay(800);
        }
  }
  
}

//this function will be executed whenever there is data available on subscribed topics
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Check if a message is received on the topic "rpi/broadcast"
  if (String(topic) == "rpi/broadcast") {
      if(messageTemp == "10"){
        Serial.println("Action: blink LED");
        blink_led(1,1250); //blink LED once (for 1250ms ON time)
      }
  }

  //Similarly add more if statements to check for other subscribed topics 
}

void setup(void) {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("BMI160 Test.");
  Wire.begin(SDA_PIN, SCL_PIN);

  // setup BMI160 sensor
  if (!BMI160.begin(BMI160GenClass::I2C_MODE, BMI160_I2C_ADDR)) {
    Serial.println("Failed to find BMI160!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("BMI160 found!");

  Serial.println("MAX30102 Test.");
  // setup MAX30102 sensor
  if (!pulseSensor.begin(Wire, I2C_SPEED_FAST))
  {
    Serial.println("Failed to find MAX30105!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MAX30102 Found!");

  byte ledBrightness = 0x1F; //Options: 0=Off to 255=50mA
  byte sampleAverage = ps_iterations; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 3; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  pulseSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

  // mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  // mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // setup WiFi Connection to MQTT Broker
  setup_wifi();
  // 1883 is default MQTT-port
  client.setServer(MQTT_IP, 1883);
  client.setCallback(callback);

  delay(100);
}

void handlePulseSensor() {
  unsigned long currentTime = millis();
  int num_samples = pulseSensor.check();

  if (num_samples > 0) {
    int pulseDataSum = 0;
    
    // STORAGE_SIZE is the size of the ESP32-side buffer (called "sense" in the library).
    // The Data gets written from the FIFO buffer (MAX30102) to the sense buffer (ESP32). 
    // So we are basically limiting the number of samples to the size of the sense buffer.
    int samplesToRead = min(num_samples, STORAGE_SIZE);

    for (int i = 0; i < samplesToRead; i++) {
      pulseDataSum += pulseSensor.getFIFOIR();
      pulseSensor.nextSample();
    }

    char message[50];
    sprintf(message, "%lu,%d", currentTime, (int)(pulseDataSum / samplesToRead));
    client.publish("esp32/pulse", message);

    Serial.println(message);
  }
}

void handleMotionSensor() {
  unsigned long currentTime = millis();
  if (currentTime - lastTimeMeasured_movement > 0) {
    lastTimeMeasured_movement = currentTime;
    int gx, gy, gz;
    BMI160.readGyro(gx, gy, gz);

    averages_gyro[0] += gx;
    averages_gyro[1] += gy;
    averages_gyro[2] += gz;

    if(count_m == 1) {
      averages_gyro[0] /= iterations_gyro;
      averages_gyro[1] /= iterations_gyro;
      averages_gyro[2] /= iterations_gyro;

      // Format: timestamp,gyro_x,gyro_y,gyro_z
      char message[100];
      sprintf(message, "%lu,%.2f,%.2f,%.2f", currentTime,
              averages_gyro[0], averages_gyro[1], averages_gyro[2]);
      client.publish("esp32/movement", message);

      Serial.println(message); // Debugging output

      averages_gyro[0] = 0.0;
      averages_gyro[1] = 0.0;
      averages_gyro[2] = 0.0;
      count_m = iterations_gyro;
    }

    count_m--;
  }
}

void loop() {
  //Check Connection to MQTT-Server
  if (!client.connected()) {
    connect_mqttServer();
  }
  client.loop();

  //Pulse Sensor
  handlePulseSensor();

  //Motion Sensor
  handleMotionSensor();
}