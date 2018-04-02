
// External libraries
#include <SPI.h>
#include <WiFi101.h>

// Secure credentials
#include "env.h"

int status = WL_IDLE_STATUS;
float moistureLevel;
float mositMax = 780.0;
float moistHigh = 640.0;
float moistLow = 340.0;

float temperature;


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(A3, INPUT);
  Serial.begin(115200);

  while (status != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID:"); 
    Serial.println(ssid);  
    status = WiFi.begin(ssid, password);

    // Wait 10 sec to connect
    delay(10000);
  }
  Serial.println("WiFi Connected!");
	Serial.print("SSID: ");	Serial.println(WiFi.SSID());
	Serial.print("IP: ");		Serial.println(WiFi.localIP());
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
  float voltage = analogRead(A3) * 3.3;
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
    analogWrite(A5, 0);   // R
    analogWrite(A4, 255); // G
    analogWrite(A6, 0);   // B
      
  } else if ( moistureLevel >= moistLow && moistureLevel < moistHigh) {
    
    analogWrite(A5, 0);  
    analogWrite(A4, 0);  
    analogWrite(A6, 255);
    
  } else {
    analogWrite(A5, 255);  
    analogWrite(A6, 0);
    analogWrite(A4, 0);   
  }
  

  
}
