
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

#include "RemoteDebug.h"
RemoteDebug Debug;


#if RUN_MED==1
#include <RunningMedian.h>
#else
#include <MedianFilter.h>
#endif

#define TEST_PAIR_ONE (2)
#define RUM_MED (2)

//#define DISTANCE_BETWEEN_TX_RX (13.3) //(14.2) //cms
float DISTANCE_BETWEEN_TX_RX = 15.0;
#define DISTANCE_OFFSET         (2) //cms
#define TEMPARATURE_OFFSET      (0) //(-91.12+31) //(-91.12 + 40.0 - 4.0 + 3.0)
#define WIND_COMP_OFFSET        (0) //(-54.67) //(-54.57 - 4.0 + 1.78)
#define SOUND_SPEED             (331.2)


Ultrasonic ultrasonic_1(12, 15);	// An ultrasonic sensor HC-04
Ultrasonic ultrasonic_2(14, 13);	// An another ultrasonic sensor HC-04

#if RUN_MED==1
RunningMedian samples_1 = RunningMedian(31);
RunningMedian samples_2 = RunningMedian(31);
#else
MedianFilter samples_1(121, 0);
MedianFilter samples_2(121, 0);
#endif

float last_wind_speed_component;
unsigned long last_mpss_time;

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
  const int updateThingSpeakInterval = 20 * 1000; // Time interval in milliseconds to update ThingSpeak (number of seconds * 1000 = interval)



///////////////////




#include<Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;



/////////////////

bool whether_post_wifi_connect_setup_done;

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

  pinMode(LED_BUILTIN, OUTPUT);

  whether_post_wifi_connect_setup_done = false;

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


  if(false==whether_post_wifi_connect_setup_done)
  {
    if(WiFi.status() == WL_CONNECTED)
    {
      setup_OTA();
      rd_setup();
      whether_post_wifi_connect_setup_done = true;
    }
  }
  if(true==whether_post_wifi_connect_setup_done)
  {
    if(WiFi.status() == WL_CONNECTED)
    {
      if(loop_OTA())
      {
        return;
      }
      rd_loop();
    }
  }
  // Sample ping
  unsigned long time_us_raw_1=  ultrasonic_1.timeRead()*2.0; // direct receiving x2.0
  delay(60);


  #if TEST_PAIR_ONE != 1
  unsigned long time_us_raw_2=ultrasonic_2.timeRead()*2.0; // direct receiving x2.0
  #else
  unsigned long time_us_raw_2=0;
  #endif

  //delay(60);

  #if RUN_MED==1
  samples_1.add(time_us_raw_1);
  float filtered_time_us_1 = samples_1.getMedian();
  #else
  samples_1.in(time_us_raw_1);
  float filtered_time_us_1 = samples_1.out();
  #endif
  //
  // float measured_dist_1 = filtered_time_us_1/CM + DISTANCE_OFFSET; // invalid
  // float measured_speed_mps_1 = (((DISTANCE_BETWEEN_TX_RX)/100.0)/(filtered_time_us_1/1000000.0));
  // float temparature_dC_1 = ((measured_speed_mps_1 - SOUND_SPEED) / 0.6) + TEMPARATURE_OFFSET;
  // float wind_speed_component_mps_1 = measured_speed_mps_1 - SOUND_SPEED + WIND_COMP_OFFSET;

  // wait for some time


  #if RUN_MED==1
  samples_2.add(time_us_raw_2);
  float filtered_time_us_2 = samples_2.getMedian();
  #else
  samples_2.in(time_us_raw_2);
  float filtered_time_us_2 = samples_2.out();
  #endif

  // float measured_dist_2 = filtered_time_us_2/CM + DISTANCE_OFFSET; // invalid
  // float measured_speed_mps_2 = (((DISTANCE_BETWEEN_TX_RX)/100.0)/(filtered_time_us_2/1000000.0));
  // float temparature_dC_2 = ((measured_speed_mps_2 - SOUND_SPEED) / 0.6) + TEMPARATURE_OFFSET;
  // float wind_speed_component_mps_2 = measured_speed_mps_2 - SOUND_SPEED + WIND_COMP_OFFSET;

  //-------------------------------

  // minus sign is for direction omly
  double speed_1 = (((DISTANCE_BETWEEN_TX_RX)/100.0)*((1000000.0/filtered_time_us_1)));
  double speed_2 = (((DISTANCE_BETWEEN_TX_RX)/100.0)*((1000000.0/filtered_time_us_2)));

  double sound_speed_measured = (speed_1+speed_2);
  double windspeed_component = (speed_1-speed_2);

  float wind_speed_component_rate_mpss = (windspeed_component - last_wind_speed_component) / ( millis() - last_mpss_time );

  last_mpss_time = millis();
  last_wind_speed_component = windspeed_component;

  float temparature_virtual = (sound_speed_measured*sound_speed_measured/402.31466)-273.15;
  float temparature_real = temparature_virtual - 0.135;
  // Measures speed = 331.4 + 0.6Tc m/s
  float temparature = ( sound_speed_measured - SOUND_SPEED ) / 0.6;
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
  DEBUG_V("* Dist: %f cms (VERBOSE)\n", DISTANCE_BETWEEN_TX_RX);
  Serial.print("dist: ");
  Serial.print(DISTANCE_BETWEEN_TX_RX);
  Serial.println(" cms");

  DEBUG_V("* Times: %f µs %f µs (VERBOSE)\n", filtered_time_us_1, filtered_time_us_2);
  Serial.print("Filtered times: ");
  Serial.print(filtered_time_us_1); // Prints the time on the unit (microseconds)
  Serial.print(", ");
  Serial.print(filtered_time_us_2); // Prints the time on the unit (microseconds)
  Serial.println(" µs");

  DEBUG_V("* windspeed_component: %f m/s %f m/s (VERBOSE)\n", windspeed_component, sound_speed_measured);
  Serial.print("windspeed_component: ");
  Serial.print(windspeed_component); // Prints the time on the unit (microseconds)
  Serial.print(", ");
  Serial.print(sound_speed_measured); // Prints the time on the unit (microseconds)
  Serial.println(" m/s");

  DEBUG_V("* Temparatures: %f °C %f °C %f °C (VERBOSE)\n", temparature_virtual, temparature_real, temparature);
  Serial.print("Temparatures: ");
  Serial.print(temparature_virtual); // Prints the time on the unit (microseconds)
  Serial.print(", ");
  Serial.print(temparature_real); // Prints the time on the unit (microseconds)
  Serial.print(", ");
  Serial.print(temparature);
  Serial.println(" °C");

  DEBUG_V("* raw times: %u µs %u µs (VERBOSE)\n\n", time_us_raw_1, time_us_raw_2);
  Serial.print("raw times: ");
  Serial.print(time_us_raw_1 ); // Prints the time on the unit (microseconds)
  Serial.print(", ");
  Serial.print(time_us_raw_2 );
  Serial.println(" µs");
  Serial.println();// If time is some duration
  // TODO

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
       tsData += String(filtered_time_us_1); // filtered time_us
       tsData +="&field2=";
       tsData += String(filtered_time_us_2); // distance_between_tx_rx_cm
       tsData +="&field3=";
       tsData += String(temparature_virtual); // temparature_dC
       tsData +="&field4=";
       tsData += String(temparature_real);  //measured_speed_mps
       tsData +="&field5=";
       tsData += String(windspeed_component); //wind_speed_component_mps
       tsData +="&field6=";
       tsData += String(time_us_raw_1);
       tsData +="&field7=";
       tsData += String(time_us_raw_2);//millis()/1000.0); // seconds
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
     else
     {

     }
     client.stop();
    }
  }

  digitalWrite(LED_BUILTIN, HIGH);

  delay(250-120);

  digitalWrite(LED_BUILTIN, LOW);

  if(Serial.available())
  {
    char c = Serial.read();
    if('+'==c)
    {
      DISTANCE_BETWEEN_TX_RX += 0.1;
    }
    if('-'==c)
    {
      DISTANCE_BETWEEN_TX_RX -= 0.1;
    }
  }


}
