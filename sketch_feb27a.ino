#include <DHT.h>
#include <DHT_U.h>
#include <SoftwareSerial.h>

#define RX 3
#define TX 4
#define dht_PIN 2
#define dht_TYPE DHT11

int UVOUT = A1; //Output from the UV sensor
char incoming_BT_Value = 0;

SoftwareSerial BT(RX, TX);
DHT dht(dht_PIN, dht_TYPE);

void setup() {
  Serial.begin(9600);
  BT.begin(9600);

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

  BT.print(temperature_C);
  BT.print("|");
  BT.print(temperature_F);
  BT.print("|");
  BT.print(humidity);
  BT.print("|");
  BT.print(heatIndex_C);
  BT.print("|");
  BT.print(heatIndex_F);

  delay(10000);

  /*
  float mq_135_value = analogRead(3);
  float digital_val = digitalRead(7);
  Serial.println(mq_135_value, DEC);
  Serial.println(digital_val, DEC);
  */

  
}