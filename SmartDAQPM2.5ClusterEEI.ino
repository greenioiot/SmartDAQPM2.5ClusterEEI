#include "HardwareSerial_NB_BC95.h"
// Include the header files that contain the icons
#include "NBIOT.h"
#include "ThaiEEI.h"
#include "DIP.h"
#include "BluetoothSerial.h"
#define _TASK_TIMECRITICAL

#include <TaskScheduler.h>
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#include <BME280I2C.h>
#include <Wire.h>

#include <Adafruit_MLX90614.h>
#include "HardwareSerial_NB_BC95.h"
#include <TFT_eSPI.h>
#include "FS.h"
//#include "SD.h"
#include "RTClib.h"
#include "Free_Fonts.h"

// Board Thingcontrol
#define SPI_MISO 19
#define SPI_MOSI 13
#define SPI_SCK 18
#define TFT_DC 5
#define TFT_RST 23
#define CF_OL32 &Orbitron_Light_32

#define title1 "PM2.5" // Text that will be printed on screen in any font
#define title2 "_________" // Text that will be printed on screen in any font

String deviceToken = "oBG4ytLhapv2e9oRyA4o";
String serverIP = "103.27.203.83"; // Your Server IP;
String serverPort = "9956"; // Your Server Port;


int statusLoading = 0;
BluetoothSerial SerialBT;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

TFT_eSPI tft = TFT_eSPI();       // Invoke custom library
#define TFT_GREY 0x5AEB // New colour

String json = "";
String attr = "";
HardwareSerial hwSerial(2);
#define SERIAL1_RXPIN 25
#define SERIAL1_TXPIN 26
BME280I2C bme;    // Default : forced mode, standby time = 1000 ms



HardwareSerial_NB_BC95 AISnb;
//const long intervalTask1 = 13000;  //millisecond
//const long intervalTask2 = 11000;  //millisecond
//const long intervalTask3 = 65000;  //millisecond
//const long intervalTask4 = 60000;  //millisecond
//
//unsigned long previousMillisTask1 = 0;
//unsigned long previousMillisTask2 = 0;
//unsigned long previousMillisTask3 = 0;
//unsigned long previousMillisTask4 = 0;

float temp(NAN), hum(NAN), pres(NAN);

void t1CallgetProbe();
void t2CallshowEnv();
void t3CallsendViaNBIOT();
//TASK
Task t1(20000, TASK_FOREVER, &t1CallgetProbe);
Task t2(22000, TASK_FOREVER, &t2CallshowEnv);
Task t3(30000, TASK_FOREVER, &t3CallsendViaNBIOT);
Scheduler runner;

int xpos = 0;
int ypos = 0;
String imsi = "";
String imei = "";
struct pms7003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm01_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};


struct pms7003data data;

void _initBME280()
{
  while (!Serial) {} // Wait

  pinMode(32, OUTPUT); // on BME280
  digitalWrite(32, HIGH); // on BME280
  delay(200);
  Wire.begin(21, 22);

  while (!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  // bme.chipID(); // Deprecated. See chipModel().
  switch (bme.chipModel())
  {
    case BME280::ChipModel_BME280:
      Serial.println("Found BME280 sensor! Success.");
      break;
    case BME280::ChipModel_BMP280:
      Serial.println("Found BMP280 sensor! No Humidity available.");
      break;
    default:
      Serial.println("Found UNKNOWN sensor! Error!");
  }
}

void _initLCD() {
  tft.fillScreen(TFT_BLACK);
  // TFT

  splash();
  // MLX
  mlx.begin();

}


void t3CallshowPM() {

  xpos = tft.width() / 2; // Half the screen width
  ypos = 20;
  tft.fillScreen(TFT_BLACK);

  //  tft.setTextColor(TFT_WHITE);
  //  tft.setFreeFont(FMB24);
  //  tft.setTextDatum(TC_DATUM); // Centre text on x,y position
  //  tft.drawString(String(data.pm25_env), xpos, ypos, FONT8);

  //  tft.setFreeFont(FMB24);                 // Select the font
  //  tft.setTextDatum(TC_DATUM); // Centre text on x,y position
  //
  //  tft.drawString("__________", xpos, ypos + 2, FONT4);
  //  tft.setTextColor(TFT_WHITE);
  //  tft.drawString("PM2.5", xpos, ypos + tft.fontHeight(FONT4), FONT4);

  //  tft.print(data.pm01_env); tft.println(" ug/m3");
  //  tft.setFreeFont(FSI12);
  //  tft.println(" -------");
  //  tft.print(" PM1.0 ");
  //
  //  tft.println(data.pm25_env); tft.println(" ug");
  //  tft.print(data.pm100_env); tft.println(" ug");
  tft.fillScreen(TFT_BLACK);
  xpos = tft.width() / 2; // Half the screen width
  ypos = 60;
  tft.setTextSize(1);
  tft.setTextColor(TFT_GREEN);
  tft.setFreeFont(FMB24);                              // Select the font
  tft.drawString(String(data.pm25_env), xpos, ypos, GFXFF);  // Draw the text string in the selected GFX free font
  ypos += tft.fontHeight(GFXFF);                      // Get the font height and move ypos down
  tft.setFreeFont(FSB9);
  tft.setTextSize(1);
  tft.drawString("-----------", xpos, ypos, GFXFF);
  ypos += tft.fontHeight(GFXFF);
  tft.drawString("PM2.5 (ug/m3)", xpos, ypos, GFXFF);

}
void t2CallshowEnv() {


  tft.setTextDatum(MC_DATUM);
  xpos = tft.width() / 2; // Half the screen width
  tft.fillScreen(TFT_BLACK);            // Clear screen

  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(3);
  tft.setFreeFont(CF_OL32);
  tft.setTextPadding(280);
  tft.drawNumber(data.pm25_env, xpos, 80);
  tft.setTextSize(1);
  tft.setFreeFont(CF_OL32);                 // Select the font
  if ((data.pm25_env > 50) && (data.pm25_env < 120)) {
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);

  } else if (data.pm25_env > 120) {
    tft.setTextColor(TFT_RED, TFT_BLACK);

  } else {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);

  }
  tft.drawString(title2, xpos, 150, GFXFF);// Print the test text in the custom font
  tft.setTextDatum(BR_DATUM);
  tft.setTextColor(TFT_WHITE);

  tft.drawString(title1, xpos, 220, GFXFF);// Print the test text in the custom font
  // Reset text padding to zero (default)
  tft.setTextPadding(0);
  
}


void t1CallgetProbe() {
  readPMSdata(&hwSerial);
  printBME280Data();

}
void sendAttribute() {
  attr = "";
  attr.concat("{\"Tn\":\"");
  attr.concat(deviceToken);
  attr.concat("\",\"IMSI\":");
  attr.concat("\"");
  attr.concat(imsi);
  attr.concat("\",\"IMEI\":");
  attr.concat("\"");
  attr.concat(imei);
  attr.concat("\"}");
  UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, attr);

}
void t3CallsendViaNBIOT() {
  composeJson();

  Serial.println(json);
  getAQI() ;
  // Send data in String
  UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, json);
  UDPReceive resp = AISnb.waitResponse();
  drawOnline();
}
void splash() {
  int xpos =  0;
  int ypos = 40;
  tft.init();
   // Swap the colour byte order when rendering
  tft.setSwapBytes(true);
  tft.setRotation(1);  // landscape

  tft.fillScreen(TFT_WHITE);
  // Draw the icons
  tft.pushImage(tft.width()/2-dipWidth/2, 1, dipWidth, dipHeight, dip);
    delay(5000);
    tft.fillScreen(TFT_WHITE);  
//  tft.pushImage(32, 32, nbiotWidth, nbiotHeight, nbiot);

  tft.pushImage(tft.width()/2-thaieeiWidth/2, 55, thaieeiWidth, thaieeiHeight, thaieei);
  delay(5000);
  tft.setTextFont(GLCD);
  tft.setRotation(1);
  //  tft.setFreeFont(FSSBO24);
  tft.fillScreen(TFT_BLACK);

  tft.setTextColor(TFT_WHITE);

  tft.setTextDatum(TC_DATUM); // Centre text on x,y position

  xpos = tft.width() / 2; // Half the screen width
  ypos = 50;

  tft.setFreeFont(FSB24);                              // Select the font
  tft.drawString("SmartDAQ", xpos, ypos, GFXFF);  // Draw the text string in the selected GFX free font
  ypos += tft.fontHeight(GFXFF);                      // Get the font height and move ypos down
  tft.setFreeFont(FSB9);
  tft.drawString("PM2.5", xpos, ypos, GFXFF);
  ypos += tft.fontHeight(GFXFF);
  tft.drawString("by", xpos, ypos, GFXFF);
  ypos += tft.fontHeight(GFXFF);
  tft.drawString("Cluster Electronics", xpos, ypos, GFXFF);
  tft.setTextPadding(280);
  tft.setTextColor(TFT_GREEN);
  tft.setTextDatum(MC_DATUM);
  Serial.println("Start...");
  for ( int i = 0; i < 180; i++)
  {
    tft.drawNumber(i, xpos, 180);
    tft.drawString(".", 1 + 2 * i, 200, GFXFF);
    delay(100);
    Serial.println(i);
  }
  Serial.println("end");
}
void setup() {

  ypos = 60;
  _initLCD();
  _initBME280();
  Serial.begin(115200);

  SerialBT.begin("Start SmartDAQPM2.5.."); //Bluetooth device name
  Serial.println("Start SmartDAQPM2.5..");
  SerialBT.print("imsi:");
  SerialBT.println(imsi);
  SerialBT.print("imei:");
  SerialBT.println(imei);


  pinMode(15, OUTPUT); // turn on PMS7003
  digitalWrite(15, HIGH); // turn on PMS7003
  delay(1000);
  pinMode(32, OUTPUT); // on BME280
  digitalWrite(32, HIGH); // on BME280
  delay(1000);
  runner.init();
  Serial.println("Initialized scheduler");

  runner.addTask(t1);
  Serial.println("added t1");
  runner.addTask(t2);
  Serial.println("added t2");
  runner.addTask(t3);
  Serial.println("added t3");

  runner.addTask(t3);
  Serial.println("added t3");
  delay(2000);
  t1.enable();  Serial.println("Enabled t1");
  t2.enable();  Serial.println("Enabled t2");
  t3.enable();  Serial.println("Enabled t3");

  AISnb.debug = true;
  AISnb.setupDevice(serverPort);
  String ip1 = AISnb.getDeviceIP();
  imsi = AISnb.getIMSI();
  imsi.trim();
  imei = AISnb.getIMEI();
  imei.trim();
  pingRESP pingR = AISnb.pingIP(serverIP);

  hwSerial.begin(9600, SERIAL_8N1, SERIAL1_RXPIN, SERIAL1_TXPIN);
}
void drawOnline() {
  tft.setTextColor(TFT_GREEN);

  tft.setTextDatum(TL_DATUM); // Centre text on x,y position

  xpos = tft.width() - 70; // Half the screen width
  ypos = 10;

  tft.setFreeFont(FMB9);                              // Select the font
//  tft.drawString("NB-IoT", xpos, ypos, GFXFF);  // Draw the text string in the selected GFX free font
  tft.pushImage(xpos, ypos, nbiotWidth, nbiotHeight, nbiot);

  delay(3000);
  tft.setTextColor(TFT_BLACK);
  
//  tft.drawString("NB-IoT", xpos, ypos, GFXFF);  // Draw the text string in the selected GFX free font

}
void printBME280Data()
{
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  bme.read(pres, temp, hum, tempUnit, presUnit);
  delay(500);
}

void composeJson() {
  json = "";
  json.concat("{\"Tn\":\"");
  json.concat(deviceToken);
  json.concat("\",\"temp\":");
  json.concat(temp);
  json.concat(",\"hum\":");
  json.concat(hum);
  json.concat(",\"pres\":");
  json.concat(pres);
  json.concat(",\"pm1\":");
  json.concat(data.pm01_env);
  json.concat(",\"pm2.5\":");
  json.concat(data.pm25_env);
  json.concat(",\"pm10\":");
  json.concat(data.pm100_env);

  json.concat(",\"pn03\":");
  json.concat(data.particles_03um);
  json.concat(",\"pn05\":");
  json.concat(data.particles_05um);
  json.concat(",\"pn10\":");
  json.concat(data.particles_10um);
  json.concat(",\"pn25\":");
  json.concat(data.particles_25um);
  json.concat(",\"pn50\":");
  json.concat(data.particles_50um);
  json.concat(",\"pn100\":");
  json.concat(data.particles_100um);
  json.concat("}");

  if (data.pm25_env > 1200)
    ESP.restart();

}
void getAQI() {

  // reading data was successful!
  //  Serial.println();
  //  Serial.println("---------------------------------------");
  //  Serial.println("Concentration Units (standard)");
  //  Serial.print("PM 1.0: "); Serial.print(data.pm10_standard);
  //  Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_standard);
  //  Serial.print("\t\tPM 10: "); Serial.println(data.pm100_standard);
  //  Serial.println("---------------------------------------");
  //  Serial.println("Concentration Units (environmental)");
  if (AISnb.debug) {
    Serial.print("PM1.0:"); Serial.print(data.pm01_env);
    Serial.print("\tPM2.5:"); Serial.print(data.pm25_env);
    Serial.print("\tPM10:"); Serial.println(data.pm100_env);
    Serial.println("---------------------------------------");
    Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(data.particles_03um);
    Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(data.particles_05um);
    Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(data.particles_10um);
    Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(data.particles_25um);
    Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(data.particles_50um);
    Serial.print("Particles > 10.0 um / 0.1L air:"); Serial.println(data.particles_100um);
    Serial.println("---------------------------------------");
  }
}


//Task2code: blinks an LED every 700 ms


boolean readPMSdata(Stream *s) {
  //  Serial.println("readPMSdata");
  if (! s->available()) {
    return false;
  }

  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }

  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  memcpy((void *)&data, (void *)buffer_u16, 30);
  // get checksum ready
  for (uint8_t i = 0; i < 30; i++) {
    sum += buffer[i];
  }
  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}
void loop() {

  runner.execute();

}
