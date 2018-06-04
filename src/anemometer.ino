
/*
 * Miltiple Ultrasonic Sensors
 * Prints the distance read by many ultrasonic sensors in
 * centimeters and inches. They are supported to four pins
 * ultrasound sensors (liek HC-SC04) and three pins
 * (like PING))) and Seeed Studio sesores).
 *
 * The circuit:
 * * In this circuit there is an ultrasonic module HC-SC04,
 *   PING))) and a Seeed Studio (4 pins, 3 pins, 3 pins,
 *   respectively), attached to digital pins as follows:
 * ---------------------     ---------------------     ---------------------
 * | HC-SC04 | Arduino |     | PING))) | Arduino |     |  Seeed  | Arduino |
 * ---------------------     ---------------------     ---------------------
 * |   Vcc   |   5V    |     |   Vcc   |   5V    |     |   Vcc   |   5V    |
 * |   Trig  |   12    | AND |   SIG   |   10    | AND |   SIG   |    8    |
 * |   Echo  |   13    |     |   Gnd   |   GND   |     |   Gnd   |   GND   |
 * |   Gnd   |   GND   |     ---------------------     ---------------------
 * ---------------------
 * Note: You need not obligatorily use the pins defined above
 *
 * By default, the distance returned by the distanceRead()
 * method is in centimeters, to get the distance in inches,
 * pass INC as a parameter.
 * Example: ultrasonic.distanceRead(INC)
 *
 * created 3 Mar 2017
 * by Erick Simões (github: @ErickSimoes | twitter: @AloErickSimoes)
 *
 * This example code is released into the MIT License.
 */
#include "Arduino.h"
#include <Ultrasonic.h>
#include <RunningMedian.h>

#define DISTANCE_BETWEEN_TX_RX (14.2) //cms
#define DISTANCE_OFFSET         (2) //cms
#define TEMPARATURE_OFFSET      (0) //(-91.12+31) //(-91.12 + 40.0 - 4.0 + 3.0)
#define WIND_COMP_OFFSET        (0) //(-54.67) //(-54.57 - 4.0 + 1.78)
#define SOUND_SPEED             (331.2)


Ultrasonic ultrasonic_1(12, 15);	// An ultrasonic sensor HC-04
Ultrasonic ultrasonic_2(14, 13);	// An another ultrasonic sensor HC-04


RunningMedian samples_1 = RunningMedian(61);
RunningMedian samples_2 = RunningMedian(61);


float last_wind_speed_component_mps_1;
unsigned long last_mpss_time_1;

float last_wind_speed_component_mps_2;
unsigned long last_mpss_time_2;
///////////////////

#include <ESP8266WiFi.h>

unsigned long last_time;
String apiWritekey = "K3CC6GPXNVQULRRX";
const char* ssid = "JioFi3_3FA858";
const char* password = "mnajk1h6tz" ;

const char* server = "api.thingspeak.com";
float resolution=3.3/1023;
WiFiClient client;

  // ThingSpeak Settings
  //char thingSpeakAddress[] = "api.thingspeak.com";
  //String writeAPIKey = "K3CC6GPXNVQULRRX";
  // GET https://api.thingspeak.com/update?api_key=K3CC6GPXNVQULRRX&field1=0&field1=0&field1=0
  const int updateThingSpeakInterval = 15 * 1000; // Time interval in milliseconds to update ThingSpeak (number of seconds * 1000 = interval)



///////////////////




#include<Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;



/////////////////

void setup() {
  Serial.begin(115200);
  WiFi.disconnect();
  delay(10);
  WiFi.begin(ssid, password);

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  setup_OTA();
  //
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }
  // Serial.println("");
  // Serial.print("connected to wifi...");
  // Serial.println(ssid);
  // Serial.println();


  // Wire.begin();
  // Wire.beginTransmission(MPU_addr);
  // Wire.write(0x6B);  // PWR_MGMT_1 register
  // Wire.write(0);     // set to zero (wakes up the MPU-6050)
  // Wire.endTransmission(true);
}

void loop() {

  if(loop_OTA())
  {
    return;
  }
  // Sample ping
  unsigned long time_us_raw_1=  ultrasonic_1.timeRead()*2.0; // direct receiving x2.0
  samples_1.add(time_us_raw_1);
  float filtered_time_us_1 = samples_1.getMedian();
  float measured_dist_1 = filtered_time_us_1/CM + DISTANCE_OFFSET; // invalid
  float measured_speed_mps_1 = (((DISTANCE_BETWEEN_TX_RX)/100.0)/(filtered_time_us_1/1000000.0));
  float temparature_dC_1 = ((measured_speed_mps_1 - SOUND_SPEED) / 0.6) + TEMPARATURE_OFFSET;
  float wind_speed_component_mps_1 = measured_speed_mps_1 - SOUND_SPEED + WIND_COMP_OFFSET;
  float wind_speed_component_rate_mpss_1 = (wind_speed_component_mps_1 - last_wind_speed_component_mps_1) / ( millis() - last_mpss_time_1 );

  last_mpss_time_1 = millis();
  last_wind_speed_component_mps_1 = wind_speed_component_mps_1;

  // wait for some time

  unsigned long time_us_raw_2=  ultrasonic_2.timeRead()*2.0; // direct receiving x2.0
  samples_2.add(time_us_raw_2);
  float filtered_time_us_2 = samples_2.getMedian();
  float measured_dist_2 = filtered_time_us_2/CM + DISTANCE_OFFSET; // invalid
  float measured_speed_mps_2 = (((DISTANCE_BETWEEN_TX_RX)/100.0)/(filtered_time_us_2/1000000.0));
  float temparature_dC_2 = ((measured_speed_mps_2 - SOUND_SPEED) / 0.6) + TEMPARATURE_OFFSET;
  float wind_speed_component_mps_2 = measured_speed_mps_2 - SOUND_SPEED + WIND_COMP_OFFSET;
  float wind_speed_component_rate_mpss_2 = (wind_speed_component_mps_2 - last_wind_speed_component_mps_2) / ( millis() - last_mpss_time_2 );

  last_mpss_time_2 = millis();
  last_wind_speed_component_mps_2 = wind_speed_component_mps_2;


  float wind_speed_component = measured_speed_mps_2-measured_speed_mps_1;
  float temparature_dC = ((wind_speed_component - SOUND_SPEED) / 0.6);

  // Sample MPU
  // Wire.beginTransmission(MPU_addr);
  // Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  // Wire.endTransmission(false);
  // Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  // AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  // AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  // AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  // Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  // GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  // GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  // GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  Serial.print("Filtered times: ");
  Serial.print(filtered_time_us_1); // Prints the time on the unit (microseconds)
  Serial.print(", ");
  Serial.print(filtered_time_us_2); // Prints the time on the unit (microseconds)
  Serial.println("us");

  Serial.print("Temparatures ");
  Serial.print(temparature_dC_1); // Prints the time on the unit (microseconds)
  Serial.print(", ");
  Serial.print(temparature_dC_2); // Prints the time on the unit (microseconds)
  Serial.println("°C, temparature: ");
  Serial.print(temparature_dC_2); // Prints the time on the unit (microseconds)
  Serial.print(temparature_dC); // Prints the time on the unit (microseconds)
  Serial.println("°C");
  //Serial.print(", ");
  Serial.print(wind_speed_component);
  Serial.println("m/s");

  Serial.print("Measured speeds: ");
  Serial.print(measured_speed_mps_1 ); // Prints the time on the unit (microseconds)
  Serial.print(", ");
  Serial.print(measured_speed_mps_2 );
  Serial.println("m/s");


  Serial.print("wind speed: ");
  Serial.print(wind_speed_component_mps_1); // Prints the time on the unit (microseconds)
  Serial.print(", ");
  Serial.print(wind_speed_component_mps_2);
  Serial.println("m/s , wind speed: ");
  //Serial.print(", ");
  Serial.print(wind_speed_component);
  Serial.println("m/s");

  Serial.print("wind accelerration: ");
  Serial.print(wind_speed_component_rate_mpss_1);
  Serial.print(", ");
  Serial.print(wind_speed_component_rate_mpss_2); // Prints the time on the unit (microseconds)
  Serial.println("m/s²");

  Serial.print("Sensor 01: ");
  Serial.print(measured_dist_1);
  Serial.print(", ");
  Serial.print(measured_dist_2); // Prints the distance on the default unit (centimeters)
  Serial.println("cm");
  Serial.println();

  // If time is some duration
  // TODO
  /*
  if(millis()-last_time>updateThingSpeakInterval) // && samples.getCount() == samples.getSize())
  {
   last_time = millis();

   if(WiFi.status() == WL_CONNECTED)
   {
     Serial.print("Wifi connection OK ");
     Serial.printf("IP %s\n", WiFi.localIP().toString().c_str());

     if (client.connect(server,80))
     {
       String tsData = apiWritekey;
       tsData +="&field1=";
       tsData += String(filtered_time_us); // filtered time_us
       tsData +="&field2=";
       tsData += String(measured_dist); // distance_between_tx_rx_cm
       tsData +="&field3=";
       tsData += String(temparature_dC); // temparature_dC
       tsData +="&field4=";
       tsData += String(measured_speed_mps);  //measured_speed_mps
       tsData +="&field5=";
       tsData += String(wind_speed_component_mps); //wind_speed_component_mps
       tsData +="&field6=";
       tsData += String(time_us_raw);
       tsData +="&field7=";
       tsData += String(millis()/1000.0); // seconds
       tsData +="&field8=";
       tsData += String(wind_speed_component_rate_mpss);
       tsData += "\r\n\r\n";

       client.print("POST /update HTTP/1.1\n");
       client.print("Host: api.thingspeak.com\n");
       client.print("Connection: close\n");
       client.print("X-THINGSPEAKAPIKEY: "+apiWritekey+"\n");
       client.print("Content-Type: application/x-www-form-urlencoded\n");
       client.print("Content-Length: ");
       client.print(tsData.length());
       client.print("\n\n");
       client.print(tsData);
       Serial.print("ThingSpeak data sent");

     }
     client.stop();
    }
  }
  */

  delay(250);


}
