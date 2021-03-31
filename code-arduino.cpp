#include <ESP8266WiFi.h>
#include <FirebaseArduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "MQ135.h"
#include <BH1750.h> // adds BH1750 library file 

#define FIREBASE_HOST "de1-c-de7fa-default-rtdb.firebaseio.com"                     //Your Firebase Project URL goes here without "http:" , "\" and "/"
#define FIREBASE_AUTH "4Fusi1Tkqpu0y6s2Y8hDBJHZjcyQ0BxppiVCDFxY" //Your Firebase Database Secret goes here

#define WIFI_SSID "HARI"                                               //WiFi SSID to which you want NodeMCU to connect
#define WIFI_PASSWORD "9894825692"  
 
#define BMP280_I2C_ADDRESS  0x76
#define sensorPower 14 
 
Adafruit_BMP280 bme; // I2C
int PinOFmotor = 12; 
int num_Measure = 128 ; // Set the number of measurements   
int pinSignal = 10; // pin connected to pin O module sound sensor   
long Sound_signal;    // Store the value read Sound Sensor   
long sum = 0 ; // Store the total value of n measurements   
long level = 0 ; // Store the average value 
int val = 0;

void setup() {
  Serial.begin(9600);

  //WiFi connection....
  Serial.print("Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting...");
  while (WiFi.status() != WL_CONNECTED)
  {
      Serial.print(".");
      delay(500);
  }
  Serial.println();
  Serial.print("Connected to: ");
  Serial.println(WiFi.localIP());

  //Firebase connection...
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Serial.println(F("BMP280 test"));

  //Barometer(BMP280) connection... 
  if (!bme.begin(BMP280_I2C_ADDRESS))
  {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  pinMode(PinOFmotor, OUTPUT);  
  pinMode (pinSignal, INPUT);
}
 
void loop() {
  //Barometer(BMP280)... 
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");
  Firebase.setInt("temp",bme.readTemperature());
   
  Serial.print("Pressure = ");
  Serial.print(bme.readPressure());
  Serial.println(" Pa");
  Firebase.setInt("pressure",bme.readPressure());
  
  Serial.print("Approx altitude = ");
  Serial.print(bme.readAltitude(1013.25)); // this should be adjusted to your local forcase
  Serial.println(" m");
  Firebase.setInt("alt",bme.readAltitude(1013.25));
   
  Serial.println();
  
  delay(2000);

  //UV sensor...
  float sensorVoltage; 
  float sensorValue;
 
  sensorValue = digitalRead(16);
  sensorVoltage = sensorValue/1024*3.3;
  Serial.print("sensor reading = ");
  Serial.print(sensorValue);
  Firebase.setInt("uv",sensorValue);
  Serial.println("");
  delay(1000);

  //Gas sensor...
  MQ135 gasSensor = MQ135(A0);
  float air_quality = gasSensor.getPPM();
  Serial.print("Air Quality: ");  
  Serial.print(air_quality);
  Serial.println(" PPM"); 
  Firebase.setInt("gas",air_quality);
  if(air_quality > 100){
    digitalWrite(PinOFmotor, HIGH);  
  }else{
    digitalWrite(PinOFmotor, LOW); 
  }

  //Raindrop sensor...
  int digital = digitalRead(13);
  Serial.print("Rain: ");
  Serial.println (digital);
  Firebase.setInt("rain",digital);

  int water = digitalRead(10);
  Serial.print("waterlvl: ");
  Serial.println (water);
  Firebase.setInt("waterlvl",water);

  for ( int i = 0 ; i <num_Measure; i++)  
  {  
   Sound_signal = digitalRead (pinSignal);  
    sum =sum + Sound_signal;  
  }  

  level = sum / num_Measure; // Calculate the average value   
  Serial.print("Sound Level: ");
  Serial.println (level);  
  Firebase.setInt("sound",level);
  sum = 0 ; // Reset the sum of the measurement values  

  int digitalValue = digitalRead(15);
  Serial.print("light = ");
  Serial.println(digitalValue);   // the raw analog reading
  Firebase.setInt("light",digitalValue);

  int soil = digitalRead(9);
  Serial.print("soil: ");
  Serial.println(soil);
  Firebase.setInt("soil",soil);
}
