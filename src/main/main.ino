
// External libraries
#include <SPI.h>
#include <WiFi101.h>
#include <LiquidCrystal.h>

// TODO: https://github.com/arduino-libraries/NTPClient
// TODO: https://www.arduino.cc/en/Reference/WiFi101OTA

// Secure credentials
#include "env.h"

int lastConnectionTime = 0;
int status = WL_IDLE_STATUS;
float moistureLevel;
float moistMax = 780.0; // ~ 3.7V 100% moisture, 
float moistHigh = 585.0; // 75% moisture
float moistLow = 390.0; // 50% moisture

float temperature;

void setTriLED(int red, int green, int blue){

    analogWrite(A6, red);   // R
    analogWrite(A4, green); // G
    analogWrite(A5, blue);   // B
}

/*__________________________________________________________SETUP__________________________________________________________*/
// the setup function runs once when you press reset or power the board
void setup() {
  
  // initialize digital pin LED_BUILTIN as an output.
  setTriLED(255, 0, 255);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(A1, INPUT);
  Serial.begin(115200);

  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  LiquidCrystal lcd(12, 11, 6, 7, 8, 9);
  // Switch on the LCD screen
  lcd.begin(16, 2);
  // Print these words to my LCD screen
  lcd.print("Starting up...");

  while (status != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID:"); 
    lcd.print("Wifi:");
    Serial.println(ssid);  
    status = WiFi.begin(ssid, password);

    // Wait 10 sec to connect
    delay(10000);
  }
  setTriLED(0, 255, 255);
  delay(500);
  
  Serial.println("WiFi Connected!");
	Serial.print("SSID: ");	Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
	Serial.print("IP: ");		Serial.println(ip);
  
}

/*__________________________________________________________LOOP__________________________________________________________*/

// the loop function runs over and over again forever
void loop() {

  int timestamp = millis();
  Serial.print("Timestamp:"); Serial.println(timestamp);

  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  
  // Create a 'heart beat' for when readings are taken.
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(5000);                       // wait for a second

  // ------------------------------------------------------------
  // TEMPERATURE
  // ------------------------------------------------------------
  // converting that reading to voltage, for 3.3v arduino use 3.3
  float voltage = analogRead(A1) * 3.3;
  voltage /= 1024.0; 
 
  // now print out the temperature
  float temperature = (voltage - 0.5) * 100 ;  //converting from 10 mv per degree wit 500 mV offset
                                               //to degrees ((voltage - 500mV) times 100)
  Serial.print("TEMP : ");
  Serial.println(temperature);
  

  // ------------------------------------------------------------
  // MOISTURE
  // ------------------------------------------------------------
  float reading = analogRead(A0);
  moistureLevel = (reading);

  if (moistureLevel >= moistHigh){
     setTriLED(0, 255, 0); 
  } else if ( moistureLevel >= moistLow && moistureLevel < moistHigh) {
     setTriLED(0, 0, 255); 
  } else {
     setTriLED(255, 0, 0); 
  }

  moistureLevel = moistureLevel / moistMax * 100.0;
  Serial.print("MOIST: ");
  Serial.println(moistureLevel);
  
}


