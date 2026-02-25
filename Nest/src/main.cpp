#include "HX711.h"
#include <stdlib.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Ticker.h>
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
Ticker timer;
 

#define DOUT  23
#define CLK  19
HX711 scale;
 
int rbutton = 18; // this button will be used to reset the scale to 0. 
String myString; 
String cmessage; // complete message
char buff[10];
float weight; 
float calibration_factor = 206140; // for me this vlaue works just perfect 206140  

//Wifi data
const char* ssid = "Jerald";
const char* password = "gehjkwqn";
const char* serverName = "https://script.google.com/macros/s/AKfycbxKe8RibJaiie_4nkjhCXyuEtf36cuxcMbGaRow03R2j3k_B1gzM3FlAt1bEJDG-7sM/exec";

// for the OLED display
 
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
 
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
 
void getSendData();

void setup() {
  
  scale.begin(DOUT, CLK);
  Serial.begin(9600);
  //rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);
  pinMode(rbutton, INPUT_PULLUP); 
  scale.set_scale();
  scale.tare(); //Reset the scale to 0
  long zero_factor = scale.read_average(); //Get a baseline reading
 
 
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  timer.attach_ms(1000, getSendData);
  display.clearDisplay();
  display.setTextColor(WHITE);
  
 
}
void loop() {
  
//timer.run(); Deprecated
 
 
scale.set_scale(calibration_factor); //Adjust to this calibration factor
 
weight = scale.get_units(5); //5
myString = dtostrf(weight, 3, 3, buff);
cmessage = cmessage + "Weight" + ":" + myString + "Kg"+","; 
Serial.println(cmessage); 
cmessage = "";
 
  Serial.println();
 
  if ( digitalRead(rbutton) == LOW)
  {
     scale.set_scale();
  scale.tare(); //Reset the scale to 0
  }
 
  
}
 
 
void getSendData()
{
  
        
// Oled display
  display.clearDisplay();
  display.setTextSize(3);
  display.setCursor(0,0); // column row
  display.print("Weight:");
 
  display.setTextSize(4);
  display.setCursor(0,30);
  display.print(myString);
 
 display.display(); 
}