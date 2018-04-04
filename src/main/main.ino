
// External libraries
#include <SPI.h>
#include <WiFi101.h>
#include <LiquidCrystal.h>

// Secure credentials
#include "env.h"

int status = WL_IDLE_STATUS;
float moistureLevel;
float mositMax = 780.0;
float moistHigh = 640.0;
float moistLow = 340.0;

float temperature;

void setTriLED(int red, int green, int blue){

    analogWrite(A6, red);   // R
    analogWrite(A4, green); // G
    analogWrite(A5, blue);   // B
}

LiquidCrystal lcd(12, 11, 6, 7, 8, 9);

// the setup function runs once when you press reset or power the board
void setup() {

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("hello, world!");
  
  // initialize digital pin LED_BUILTIN as an output.
  setTriLED(255, 0, 255);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(A1, INPUT);
  Serial.begin(115200);

  while (status != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID:"); 
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

// the loop function runs over and over again forever
void loop() {

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
  Serial.print("MOIST: ");
  Serial.println(moistureLevel);
  


  if (moistureLevel >= moistHigh){
     setTriLED(0, 255, 0); 
  } else if ( moistureLevel >= moistLow && moistureLevel < moistHigh) {
     setTriLED(0, 0, 255); 
  } else {
     setTriLED(255, 0, 0); 
  }
  

  
}
