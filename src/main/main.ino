
// External libraries
#include <SPI.h>
#include <WiFi101.h>
#include <MQTT.h>
#include "DHT.h"
//#include "ArduinoLowPower.h"


#define DHTPIN 7     // what digital pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);

// TODO: Drive pump 
//        https://www.arduino.cc/documents/datasheets/H-bridge_motor_driver.PDF
// TODO: Battery levels as measurements
//        https://github.com/rlogiacco/BatterySense

// https://io.adafruit.com/neozenith/dashboards/bonsai

// Secure credentials
#include "env.h"

WiFiClient net;
MQTTClient client;

int lastConnectionTime = 0;
int lastMeasurement = 0;
int interval = 60000;
int status = WL_IDLE_STATUS;

int remotelight = LOW;

void setTriLED(int red, int green, int blue){
  
    //TODO: move to digital pins to free up analog inputs.
    analogWrite(A6, red);   // R
    analogWrite(A4, green); // G
    analogWrite(A5, blue);   // B
}

void connectWiFi(){
  Serial.print("Attempting to connect to SSID:"); 
  Serial.println(ssid);  
  while (WiFi.status() != WL_CONNECTED){
    status = WiFi.begin(ssid, password);

    delay(1000);
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

  // TODO: Refactor into array of subscriptions
  client.subscribe("neozenith/feeds/bonsai.remotelight");
  client.subscribe("neozenith/feeds/bonsai.measurement-interval");
  
}

void publishInitialInterval(){

  if (!client.connected()) {
    connectMQTT();
  }
  // On reset publish the reset initial value
  client.publish("neozenith/feeds/bonsai.measurement-interval", String(interval/1000));  
}

// the setup function runs once when you press reset or power the board
void setup() {
  
  Serial.begin(115200);
  Serial.println("SETUP");
  int pause = 200;
  setTriLED(255, 0, 0);
  delay(pause);
  setTriLED(0, 255, 0);
  delay(pause);
  setTriLED(0, 0, 255);
  delay(pause);
  setTriLED(0, 255, 255);
  delay(pause);
  setTriLED(255, 255, 0);
  delay(pause);
  setTriLED(255, 0, 255);
  
  
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);

  pinMode(5, OUTPUT);

  pinMode(7, INPUT);

  // DHT: Digital Humidity and Temperature
  dht.begin();

  Serial.println("Pin setup complete.");

  connectWiFi(); 

  // MQTT Handler
  client.begin(AIO_SERVER, AIO_SERVERPORT, net);
  client.onMessage(messageReceived);

  publishInitialInterval();

}

/*__________________________________________________________LOOP__________________________________________________________*/

float measureTemperature(){
  // ------------------------------------------------------------
  // TEMPERATURE - TMP36
  // http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/Temp/TMP35_36_37.pdf
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
  // https://www.dfrobot.com/wiki/index.php/Moisture_Sensor_(SKU:SEN0114)
  // the sensor value description
  // 0  ~300     dry soil
  // 300~700     humid soil
  // 700~950     in water
  // ------------------------------------------------------------
  return analogRead(A0);
}

float measureLight(){
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
float measureBattery(){
  // ------------------------------------------------------------
  // BATTERY
  // Using 10k + 10k voltage divider on 3.7V LiPo
  // ------------------------------------------------------------
  int analogValue = analogRead(A3);
  float sensorValue = analogValue / 1024.0 * 3.3 * 2.0;
  return sensorValue;
}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
  if (topic == "neozenith/feeds/bonsai.remotelight"){
    mqttHandleRemoteLight(payload);
  }
  if (topic == "neozenith/feeds/bonsai.measurement-interval"){
    mqttHandleMeasurementInterval(payload);
  }
  
}

void mqttHandleRemoteLight(String& payload){
  if (payload == "1"){
    remotelight = HIGH;  
  } else {
    remotelight = LOW;
  }
}
void mqttHandleMeasurementInterval(String& payload){
  interval = payload.toInt() * 1000; //ms
}

// the loop function runs over and over again forever
void loop() {

  int timestamp = millis();

  client.loop();

  if (!client.connected()) {
    connectMQTT();
  }

  boolean button0 = digitalRead(0);
  if (button0 == LOW && remotelight == HIGH){

    Serial.println("reset remote light");
    remotelight = LOW;  
    client.publish("neozenith/feeds/bonsai.remotelight", 0);
  }
  digitalWrite(5, remotelight);  
  
  int button1 = digitalRead(1);
  int button2 = digitalRead(3);
  int button3 = digitalRead(4);
  setTriLED(255 - button1*255, 255 - button2*255, 255 - button3*255);

  
  
  

  if (timestamp - lastMeasurement > interval) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    
    float temperature = measureTemperature();
    float moisture = measureMoisture();
    float ambientlight = measureLight();
    float battery = measureBattery();
    float dht22_h = dht.readHumidity();
    float dht22_t = dht.readTemperature();

    Serial.print("Timestamp:"); Serial.println(timestamp);
    Serial.print("TEMP : ");
    Serial.println(temperature);
    
    Serial.print("DHT22 Temp: ");
    Serial.println(dht22_t);
    
    Serial.print("DHT22 Humidity: ");
    Serial.println(dht22_h);
    
    Serial.print("MOIST: ");
    Serial.println(moisture);  
    
    Serial.print("LIGHT: ");
    Serial.println(ambientlight); 

    Serial.print("BATTERY: ");
    Serial.println(battery); 
    
    logWifi();

    client.publish("neozenith/feeds/bonsai.temperature", String(temperature));
    client.publish("neozenith/feeds/bonsai.moisture", String(moisture));
    if (!isnan(dht22_t)){
      client.publish("neozenith/feeds/bonsai.dht22-humidity", String(dht22_h));
    }
    if (!isnan(dht22_t)){
      client.publish("neozenith/feeds/bonsai.dht22-temperature", String(dht22_t));
    }
    client.publish("neozenith/feeds/bonsai.ambientlight", String(ambientlight));
    client.publish("neozenith/feeds/bonsai.calibratebattery", String(battery));

    
    lastMeasurement = timestamp;
  }
  
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW

  //LowPower.sleep(2000);
  //delay(2000);

}


