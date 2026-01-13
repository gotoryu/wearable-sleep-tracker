#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

//WiFi Connection Variables
const char* ssid = "devolo-d13"; //WiFi SSID
const char* password = "..."; //WiFi Password
const char* mqtt_server = "192.168.1.56"; //MQTT Broker IP
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
#define ledPin 2

Adafruit_MPU6050 mpu;

//Pulse Sensor Variables
#define psPIN 35        // Pulse Sensor PURPLE WIRE connected to ANALOG PIN 0
int LED = 15;   //  The on-board Arduion LED
double average_pulse = 0;
int iterations_pulse = 20;
int count_p = iterations_pulse;
int sig;                // holds the incoming raw data. Signal value can range from 0-1024

unsigned long lastTimeMeasured_movement = millis();
unsigned long lastTimeMeasured_pulse = millis();
float averages_motion[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
int iterations_motion = 20;
int count_m = iterations_motion;

void blink_led(unsigned int times, unsigned int duration){
  for (int i = 0; i < times; i++) {
    digitalWrite(ledPin, HIGH);
    delay(duration);
    digitalWrite(ledPin, LOW); 
    delay(200);
  }
}

void setup_wifi() {
  delay(50);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

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
  pinMode(ledPin, OUTPUT);

  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // setup mpu6050 sensor
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // setup WiFi Connection to MQTT Broker
  setup_wifi();
  // 1883 default MQTT-port, mqtt_server contains server IP
  client.setServer(mqtt_server,1883);
  client.setCallback(callback);

  delay(100);
}

void handlePulseSensor() {
  unsigned long currentTime = millis();
  // small delay between data points
  if (currentTime - lastTimeMeasured_pulse > 0) {
    lastTimeMeasured_pulse = currentTime;
    count_p--;
    sig = analogRead(psPIN);  // Read the PulseSensor's value.
    average_pulse += sig;

    if (count_p == 0) {
      // calculate the average of the last 20 values
      int pulseData = (int)average_pulse / iterations_pulse;

      // Format: timestamp,value
      char message[50];
      sprintf(message, "%lu,%d", currentTime, pulseData);
      // send the data to the server on the topic "esp32/pulse"
      client.publish("esp32/pulse", message);

      Serial.println(message); // Debugging output

      // reset the values
      average_pulse = 0;
      count_p = iterations_pulse;
    }
  }
}


void handleMotionSensor() {
  unsigned long currentTime = millis();
  if (currentTime - lastTimeMeasured_movement > 0) {
    lastTimeMeasured_movement = currentTime;
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    averages_motion[0] += a.acceleration.x; averages_motion[1] += a.acceleration.y; averages_motion[2] += a.acceleration.z;
    averages_motion[3] += g.gyro.x; averages_motion[4] += g.gyro.y; averages_motion[5] += g.gyro.z;

    if(count_m == 0) {
      count_m = iterations_motion;
      averages_motion[0] /= iterations_motion; averages_motion[1] /= iterations_motion; averages_motion[2] /= iterations_motion;
      averages_motion[3] /= iterations_motion; averages_motion[4] /= iterations_motion; averages_motion[5] /= iterations_motion;

      // Format: timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z
      char message[100];
      sprintf(message, "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", currentTime,
              averages_motion[0], averages_motion[1], averages_motion[2],
              averages_motion[3], averages_motion[4], averages_motion[5]);
      client.publish("esp32/movement", message);

      Serial.println(message); // Debugging output#

      for (int i = 0; i < 6; i++) {
        averages_motion[i] = 0.0;
      }
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