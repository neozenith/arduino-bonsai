/*
  Blink
  */

#include <SPI.h>
#include <WiFi101.h>

int status = WL_IDLE_STATUS;
float lightLevel;
char ssid[] = "teamtj";
char password[] = "";

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  while (status != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID:"); 
    Serial.print(ssid);  
    status = WiFi.begin(ssid, password);

    // Wait 10 sec to connect
    delay(10000);
  }
  Serial.println("WiFi Connected!");
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(5000);                       // wait for a second

  float reading = analogRead(A0);
  lightLevel = (reading /1024.0)*100;

  Serial.println(lightLevel);
  
}
