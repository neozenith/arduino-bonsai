
// External libraries
#include <SPI.h>
#include <WiFi101.h>
#include <MQTT.h>


// TODO: https://io.adafruit.com/neozenith/dashboards/bonsai

// Secure credentials
#include "env.h"

WiFiClient net;
MQTTClient client;

int lastConnectionTime = 0;
int lastMeasurement = 0;
int status = WL_IDLE_STATUS;

float moistureLevel;
float moistMax = 780.0; // ~ 3.7V 100% moisture, 
float moistHigh = 585.0; // 75% moisture
float moistLow = 390.0; // 50% moisture

float temperature;

int remotelight = LOW;

void setTriLED(int red, int green, int blue){

    analogWrite(A6, red);   // R
    analogWrite(A4, green); // G
    analogWrite(A5, blue);   // B
}

void connectWiFi(){
  Serial.print("Attempting to connect to SSID:"); 
  Serial.println(ssid);  
  while (WiFi.status() != WL_CONNECTED){
    status = WiFi.begin(ssid, password);

    delay(5000);
  }
  
  
  Serial.println("WiFi Connected!");
  Serial.print("SSID: "); Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("IP: ");   Serial.println(ip);
}

void logWifi(){
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");  
}
/*__________________________________________________________SETUP__________________________________________________________*/

void connectMQTT(){
 
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

  client.subscribe("neozenith/feeds/bonsai.remotelight");
  
}


// the setup function runs once when you press reset or power the board
void setup() {
  
  Serial.begin(115200);
  Serial.println("SETUP");

// initialize digital pin LED_BUILTIN as an output.
  setTriLED(255, 0, 255);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  
  pinMode(5, OUTPUT);

  Serial.println("Pin setup complete.");

  connectWiFi(); 
  
  client.begin("io.adafruit.com", net);
  client.onMessage(messageReceived);
  
  
  
}

/*__________________________________________________________LOOP__________________________________________________________*/

float measureTemperature(){
  // ------------------------------------------------------------
  // TEMPERATURE
  // ------------------------------------------------------------
  // converting that reading to voltage, for 3.3v arduino use 3.3
  float voltage = analogRead(A1) * 3.3;
  voltage /= 1024.0; 
 
  // now print out the temperature
  float temperature = (voltage - 0.5) * 100 ;  //converting from 10 mv per degree wit 500 mV offset
                                               //to degrees ((voltage - 500mV) times 100)
  return temperature;
}

float measureMoisture(){
  // ------------------------------------------------------------
  // MOISTURE
  // ------------------------------------------------------------
  float reading = analogRead(A0);
  moistureLevel = (reading);

  moistureLevel = moistureLevel / moistMax * 100.0;
  
  return moistureLevel;
}

float measureLight(){
  return analogRead(A2);
}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
  if (topic == "neozenith/feeds/bonsai.remotelight"){
    mqttHandleRemoteLight(payload);
  }
  
}

void mqttHandleRemoteLight(String& payload){
  if (payload == "1"){
    remotelight = HIGH;  
  } else {
    remotelight = LOW;
  }
}

// the loop function runs over and over again forever
void loop() {

  int timestamp = millis();

  client.loop();

  if (!client.connected()) {
    connectMQTT();
  }

  int button0 = digitalRead(0);
  int button1 = digitalRead(1);
  int button2 = digitalRead(3);
  int button3 = digitalRead(4);

  digitalWrite(5, remotelight);  
  setTriLED(255 - button1*255, 255 - button2*255, 255 - button3*255);

  
    
  if (timestamp - lastMeasurement > 30000) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    
    float temperature = measureTemperature();
    float moisture = measureMoisture();
    float light = measureLight();

    Serial.print("Timestamp:"); Serial.println(timestamp);
    Serial.print("TEMP : ");
    Serial.println(temperature);
    client.publish("neozenith/feeds/bonsai.temperature", String(temperature));
    Serial.print("MOIST: ");
    Serial.println(moisture);  
    client.publish("neozenith/feeds/bonsai.moisture", String(moisture));
    Serial.print("LIGHT: ");
    Serial.println(light);  
//    client.publish("bonsai.light", light);
      logWifi();

    lastMeasurement = timestamp;
  }
  
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  

}


