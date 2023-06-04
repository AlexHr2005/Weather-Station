#include <DHT.h>
#include <DHT_U.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define RX 3
#define TX 4
#define dht_PIN 2
#define dht_TYPE DHT11

uint8_t mqPin = A1;
uint8_t uvPin = A2;

SoftwareSerial BT(RX, TX);
DHT dht(dht_PIN, dht_TYPE);
Adafruit_BMP280 bmp;

void setup() {
  Serial.begin(9600);
  BT.begin(9600);
  if (!bmp.begin(0x76)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  dht.begin();
}

void loop() {
  float humidity = dht.readHumidity();

  float temperature_C = dht.readTemperature();
  float temperature_F = dht.convertCtoF(temperature_C);
  float heatIndex_F = dht.computeHeatIndex();
  float heatIndex_C  = dht.convertFtoC(heatIndex_F);

  float pressure = bmp.readPressure() / 100; //to convert pressure from Pa in hPa
  float altitude = bmp.readAltitude();

  int mqSensorValue = analogRead(mqPin);
  String airQuality = "";
  if(mqSensorValue < 190) {
    airQuality = "Good";
  }
  else if(mqSensorValue < 300) {
    airQuality = "Moderate";
  }
  else airQuality = "Bad";

  int uvSensorValue = analogRead(uvPin);
  float voltage = uvSensorValue * (5.0 / 1013.0);
  float uvIntensity = mapValue(voltage, 0.99, 2.9, 0.0, 15.0);
  String UVrisk = "";
  if(uvIntensity <= 2) {
    UVrisk = "Low";
  }
  else if(uvIntensity <= 5) {
    UVrisk = "Moderate";
  }
  else if(uvIntensity <= 7) {
    UVrisk = "High";
  }
  else if(uvIntensity <= 10) {
    UVrisk = "Very High";
  }
  else UVrisk = "Extreme";

  BT.print(temperature_C);
  BT.print("|");
  BT.print(temperature_F);
  BT.print("|");
  BT.print(humidity);
  BT.print("|");
  BT.print(heatIndex_C);
  BT.print("|");
  BT.print(heatIndex_F);
  BT.print("|");
  BT.print(pressure);
  BT.print("|");
  BT.print(altitude);
  BT.print("|");
  BT.print(airQuality);
  BT.print("|");
  BT.print(UVrisk);

  delay(60000);
}

float mapValue(float value, float inMin, float inMax, float outMin, float outMax) {
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}