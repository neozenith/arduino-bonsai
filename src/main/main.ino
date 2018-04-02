
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
}

// the loop function runs over and over again forever
void loop() {

  // Create a 'heart beat' for when readings are taken.
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(5000);                       // wait for a second

  float reading = analogRead(A0);
  moistureLevel = (reading);
  Serial.print("MOIST: ");
  Serial.println(moistureLevel);
  
  temperature = analogRead(A3);
  Serial.print("TEMP : ");
  Serial.println(temperature);

  if (moistureLevel >= moistHigh){
    analogWrite(A5, 0);  
    analogWrite(A6, 0);
    analogWrite(A4, 255);  
  } else if ( moistureLevel >= moistLow && moistureLevel < moistHigh) {
    
    analogWrite(A5, 0);  
    analogWrite(A6, 255);
    analogWrite(A4, 0);  
  } else {
    analogWrite(A5, 255);  
    analogWrite(A6, 0);
    analogWrite(A4, 0);   
  }
  

  
}
