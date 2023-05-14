#include <DHT.h>
#include <DHT_U.h>
#include <SoftwareSerial.h>

#define dht_PIN 2
#define dht_TYPE DHT11
#define BT_RX_PIN 0;
#define BT_TX_PIN 1;

int UVOUT = A1; //Output from the UV sensor
char incoming_BT_Value = 0;


DHT dht(dht_PIN, dht_TYPE);
SoftwareSerial bt(BT_RX_PIN, BT_TX_PIN);

void setup() {
  Serial.begin(9600);

  dht.begin();
}

void loop() {
  /*int UV_value = analogRead(UVOUT);
  float UV_voltage = UV_value * (5.0 / 1023.0);
  float UV_intensity = UV_voltage / 0.1; //this value is seen by the user
  
  Serial.print("UV intensity: ");
  Serial.print(UV_intensity);
  Serial.println(" mw/cm^2");
  Serial.println(UV_voltage);

  delay(300);*/
  
  float humidity = dht.readHumidity();

  float temperature_C = dht.readTemperature();
  float temperature_F = dht.convertCtoF(temperature_C);
  float heatIndex_F = dht.computeHeatIndex();
  float heatIndex_C  = dht.convertFtoC(heatIndex_F);

  Serial.print("Temperature (C): ");
  Serial.println(temperature_C);
  Serial.print("Temperature (f): ");
  Serial.println(temperature_F);
  Serial.print("Heat index (C): ");
  Serial.println(heatIndex_C);
  Serial.print("Heat index (F): ");
  Serial.println(heatIndex_F);
  Serial.print("Humidity (%): ");
  Serial.println(humidity);
  Serial.println();

  delay(4000);

  /*
  float mq_135_value = analogRead(3);
  float digital_val = digitalRead(7);
  Serial.println(mq_135_value, DEC);
  Serial.println(digital_val, DEC);
  */

  
}