
// External libraries
#include <SPI.h>
#include <WiFi101.h>

#include <MQTT.h>
// https://github.com/256dpi/arduino-mqtt by Joël Gähwiler

#include "DHT.h"
// https://github.com/adafruit/DHT-sensor-library
// https://github.com/adafruit/Adafruit_Sensor

//#include "ArduinoLowPower.h"


#define DHTPIN 7     // what digital pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);

#define TriLED_RED 10
#define TriLED_GREEN 12
#define TriLED_BLUE 11

#define PIN_REMOTE_LIGHT 9

#define Motor1_EN 0
#define Motor1_FWD 2 // PWM Pin
#define Motor1_REV 3 // PWM Pin

#define Motor2_EN 1
#define Motor2_FWD 4 // PWM Pin
#define Motor2_REV 5 // PWM Pin

// Push Buttons
#define pb0 14
#define pb1 13
#define pb2 8


// TODO: Drive pump
//        https://www.arduino.cc/documents/datasheets/H-bridge_motor_driver.PDF
// TODO: Battery levels as measurements
//        https://github.com/rlogiacco/BatterySense

// https://io.adafruit.com/neozenith/dashboards/bonsai

// Secure credentials
#include "env.h"

WiFiClient net;
MQTTClient client;

long lastConnectionTime = 0;
long lastMeasurement = 0;
int interval = 60000;
int status = WL_IDLE_STATUS;

int motor = 0;
int motorSpeed = 0;
int remotelight = LOW;

void setTriLED(int red, int green, int blue) {
  digitalWrite(TriLED_RED, red);   // R
  digitalWrite(TriLED_GREEN, green); // G
  digitalWrite(TriLED_BLUE, blue);   // B
}

void connectWiFi() {
  Serial.print("Attempting to connect to SSID:");
  Serial.println(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    status = WiFi.begin(ssid, password);

    delay(1000);
  }

  Serial.println("WiFi Connected!");
  Serial.print("SSID: "); Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("IP: ");   Serial.println(ip);
}

void logWifi() {
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void connectMQTT() {

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.print("\nconnecting...");
  while (!client.connect("Arduino MKR 1000", IO_USERNAME, IO_KEY)) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");

  // TODO: Refactor into array of subscriptions
  client.subscribe("neozenith/feeds/bonsai.remotelight");
  client.subscribe("neozenith/feeds/bonsai.measurement-interval");

}

void publishInitialInterval() {

  if (!client.connected()) {
    connectMQTT();
  }
  // On reset publish the reset initial value
  client.publish("neozenith/feeds/bonsai.measurement-interval", String(interval / 1000));
}

void setMotorSpeed(int motor, int speedFwd, int speedRev){
  if (motor == 1){
    digitalWrite(Motor1_EN, HIGH);
    analogWrite(Motor1_FWD, speedFwd);
    analogWrite(Motor1_REV, speedRev);
  
  }

  if (motor == 2){
    digitalWrite(Motor2_EN, HIGH);
    analogWrite(Motor2_FWD, speedFwd);
    analogWrite(Motor2_REV, speedRev);
  }
  
}

// the setup function runs once when you press reset or power the board
void setup() {

 //L293D H-Bridge Motor Control Chip
  pinMode(Motor1_EN, OUTPUT); // Motor 1 EN
  pinMode(Motor1_FWD, OUTPUT); // Motor 1 Forward
  pinMode(Motor1_REV, OUTPUT); // Motor 1 Reverse

  pinMode(Motor2_EN, OUTPUT); // Motor 2 EN
  pinMode(Motor2_FWD, OUTPUT); // Motor 2 Forward
  pinMode(Motor2_REV, OUTPUT); // Motor 2 Reverse

  // Ensure motors are stationary as soon as possible.
  setMotorSpeed(1, 0, 0);
  setMotorSpeed(2, 0, 0);

  Serial.begin(115200);
  Serial.println("SETUP");

  // TriLED
  pinMode(TriLED_RED, OUTPUT);
  pinMode(TriLED_GREEN, OUTPUT);
  pinMode(TriLED_BLUE, OUTPUT);
  
  // Builtin LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Sensors
  // TODO: Convert to declared variable or #define
  pinMode(A0, INPUT); // Moisture A
  pinMode(A1, INPUT); // Moisture B
  pinMode(A2, INPUT); // Phototransistor
  pinMode(A3, INPUT); // TMP36
  pinMode(A4, INPUT); // Battery Sensor

  //DHT22
  pinMode(DHTPIN, INPUT);

  // LED
  pinMode(PIN_REMOTE_LIGHT, OUTPUT);

  // Push buttons
  pinMode(pb0, INPUT_PULLUP);
  pinMode(pb1, INPUT_PULLUP);
  pinMode(pb2, INPUT_PULLUP);
  

  int pause = 300;
  setTriLED(HIGH, LOW, LOW); // RED
  delay(pause);
  setTriLED(LOW, HIGH, LOW); // GREEN
  delay(pause);
  setTriLED(LOW, LOW, HIGH); // BLUE
  delay(pause);
  setTriLED(LOW, HIGH, HIGH); // AQUA
  delay(pause);
  setTriLED(HIGH, HIGH, LOW); // YELLOW
  delay(pause);
  setTriLED(HIGH, LOW, HIGH);

  // DHT: Digital Humidity and Temperature
  dht.begin();

  Serial.println("Pin setup complete.");

  connectWiFi();

  // MQTT Handler
  client.begin(AIO_SERVER, AIO_SERVERPORT, net);
  client.onMessage(messageReceived);

  publishInitialInterval();

}


float measureTemperature() {
  // ------------------------------------------------------------
  // TEMPERATURE - TMP36
  // http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/Temp/TMP35_36_37.pdf
  // ------------------------------------------------------------
  // converting that reading to voltage, for 3.3v arduino use 3.3
  float voltage = analogRead(A3) * 3.3;
  voltage /= 1024.0;

  // now print out the temperature
  float temperature = (voltage - 0.5) * 100 - 3.57 ;  //converting from 10 mv per degree wit 500 mV offset
  //to degrees ((voltage - 500mV) times 100)
  return temperature;
}
float measureMoisture(int pin) {
  // ------------------------------------------------------------
  // MOISTURE
  // https://www.dfrobot.com/wiki/index.php/Moisture_Sensor_(SKU:SEN0114)
  // the sensor value description
  // 0  ~300     dry soil
  // 300~700     humid soil
  // 700~950     in water
  // ------------------------------------------------------------
  return analogRead(pin);
}
float measureLight() {
  // ------------------------------------------------------------
  // Ambient Light
  // https://www.arduino.cc/documents/datasheets/HW5P-1.pdf
  // 0.0-1.0: 0-10 Night
  // 1.0-2.3: 10-200 Lit room
  // 2.3-2.5: 200-300 Shady daylight
  // 2.5-3.0: 300-1000 Full Ambient light
  // ------------------------------------------------------------
  float sensorValue;
  int analogValue = analogRead(A2);//connect light sensors to Analog 0
  sensorValue = log10(analogValue);
  return sensorValue;
}
float measureBattery() {
  // ------------------------------------------------------------
  // BATTERY
  // Using 10k + 10k voltage divider on 3.7V LiPo
  // ------------------------------------------------------------
  int analogValue = analogRead(A4);
  float sensorValue = analogValue / 1024.0 * 3.3 * 2.0;
  return sensorValue;
}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
  if (topic == "neozenith/feeds/bonsai.remotelight") {
    mqttHandleRemoteLight(payload);
  }
  if (topic == "neozenith/feeds/bonsai.measurement-interval") {
    mqttHandleMeasurementInterval(payload);
  }

}

void mqttHandleRemoteLight(String& payload) {
  if (payload == "1") {
    remotelight = HIGH;
  } else {
    remotelight = LOW;
  }
}
void mqttHandleMeasurementInterval(String& payload) {
  interval = payload.toInt() * 1000; //ms
}

// the loop function runs over and over again forever
void loop() {

  long timestamp = millis();

  client.loop();

  if (!client.connected()) {
    connectMQTT();
  }

  boolean button0 = digitalRead(pb0);
  if (button0 == LOW) {
    
    while (button0 == LOW){
      button0 = digitalRead(pb0);
    }
    motor = (motor + 1) % 3;
  }

  boolean button1 = digitalRead(pb1);
  if (button1 == LOW) {
    
    while (button1 == LOW){
      button1 = digitalRead(pb1);
    }
    motorSpeed = motorSpeed + 64;
    
    if (motorSpeed > 255) motorSpeed = 255;
  }

  boolean button2 = digitalRead(pb2);
  if (button2 == LOW) {
    
    while (button2 == LOW){
      button2 = digitalRead(pb2);
    }
    motorSpeed = motorSpeed - 64;
    
    if (motorSpeed <= 0) motorSpeed = 0;
  }
  
  digitalWrite(PIN_REMOTE_LIGHT, remotelight);

  
  
  if (motor == 1) {
    setTriLED(LOW, HIGH, LOW);
    setMotorSpeed(1, motorSpeed, 0);
    setMotorSpeed(2, 0, 0);
  }
  else if (motor == 2) {
    setTriLED(LOW, LOW, HIGH);
    setMotorSpeed(1, 0, 0);
    setMotorSpeed(2, motorSpeed, 0);
  }
  else {
    setTriLED(LOW, LOW, LOW);
    setMotorSpeed(1, 0, 0);
    setMotorSpeed(2, 0, 0);
  }

  if (timestamp - lastMeasurement > interval) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)

    float temperature = measureTemperature();
    float moistureA = measureMoisture(A0);
    float moistureB = measureMoisture(A1);
    float ambientlight = measureLight();
    float battery = measureBattery();
    float dht22_h = dht.readHumidity();
    float dht22_t = dht.readTemperature();

    Serial.print("Timestamp:"); Serial.println(timestamp);
    Serial.print("TEMP : ");
    Serial.println(temperature);

    Serial.print("DHT22 Temp: ");
    Serial.println(dht22_t);
    Serial.println(temperature - dht22_t);

    Serial.print("DHT22 Humidity: ");
    Serial.println(dht22_h);

    Serial.print("MOIST: ");
    Serial.println(moistureA);
    Serial.println(moistureB);

    Serial.print("LIGHT: ");
    Serial.println(ambientlight);

    Serial.print("BATTERY: ");
    Serial.println(battery);

    logWifi();

    client.publish("neozenith/feeds/bonsai.temperature", String(temperature));
    client.publish("neozenith/feeds/bonsai.moisturea", String(moistureA));
    client.publish("neozenith/feeds/bonsai.moistureb", String(moistureB));
    if (!isnan(dht22_t)) {
      client.publish("neozenith/feeds/bonsai.dht22-humidity", String(dht22_h));
    }
    if (!isnan(dht22_t)) {
      client.publish("neozenith/feeds/bonsai.dht22-temperature", String(dht22_t));
    }
    client.publish("neozenith/feeds/bonsai.ambientlight", String(ambientlight));
    client.publish("neozenith/feeds/bonsai.calibratebattery", String(battery));


    lastMeasurement = timestamp;
  }

  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW

}
