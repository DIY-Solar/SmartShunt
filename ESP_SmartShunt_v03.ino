/* ACS758-100B ESP8266 SmartShunt for LiPoFe4 Battery
 * 
 * Peukert's formula, T = C / In (or C = T * In), 
 * where C is theoretical capacity (in amp-hours, 
 * equal to actual capacity at one amp), I is current (in amps), 
 * T is time (in hours), and n is the ve.shunt.Peukert number for the battery. 
 * That number shows how well the battery holds up under heavy rates of discharge. 
 * Most range from 1.1 to 1.4; 1.2 is about average.
 * 
 * # Example: {'H1': '-264148', 'H2': '-2909', 'H3': '-109417', 'H4': '6', 'H5': '1', 'H6': '-3928992', 'H7': '6200',
# 'H8': '14592', 'H9': '3331', 'H10': '21', 'H11': '0', 'H12': '0', 'H15': '-27', 'H16': '14592', 'H17': '5148',
# 'H18': '5581', 'PID': '0xA389', 'V': '13259', 'VS': '12716', 'I': '-7742', 'P': '-103', 'CE': '-2911', 'SOC': '990',
# 'TTG': '2052', 'Alarm': 'OFF', 'AR': '0', 'BMV': 'SmartShunt 500A/50mV', 'FW': '0407', 'MON': '0'}
#
 * # MAP: KEY: (Category, Description, Unit, Factor/Mapping, Interpret_function)
    MAP = {
        'H1': ("History", "Deepest Discharge", "Ah", 0.001, helper.convert_int_factor),
        'H2': ("History", "Last Discharge", "Ah", 0.001, helper.convert_int_factor),
        'H3': ("History", "Average Discharge", "Ah", 0.001, helper.convert_int_factor),
        'H4': ("History", "Charge Cycles", "", 1, helper.convert_int_factor),
        'H5': ("History", "Full Discharges", "", 1, helper.convert_int_factor),
        'H6': ("History", "Cumulative Ah Drawn", "Ah", 0.001, helper.convert_int_factor),
        'H7': ("History", "Battery Voltage min", "V", 0.001, helper.convert_int_factor),
        'H8': ("History", "Battery Voltage max", "V", 0.001, helper.convert_int_factor),
        'H9': ("History", "Time Since Last Full", "s", 1, helper.convert_int_factor),
        'H10': ("History", "Synchonisations", "", 1, helper.convert_int_factor),
        'H11': ("History", "Alarm Voltage low", "", 1, helper.convert_int_factor),
        'H12': ("History", "Alarm Voltage high", "", 1, helper.convert_int_factor),
        'H15': ("History", "Starter Battery Voltage min", "V", 0.001, helper.convert_int_factor),
        'H16': ("History", "Starter Battery Voltage max", "V", 0.001, helper.convert_int_factor),
        'H17': ("History", "Total Discharged Energy", "kWh", 0.01, helper.convert_int_factor),
        'H18': ("History", "Total Charged Energy", "kWh", 0.01, helper.convert_int_factor),
        'PID': ("Meta", "Product ID", "", PID, helper.convert_map_out),
        'V': ("Latest", "Voltage", "V", 0.001, helper.convert_int_factor),
        'VS': ("Latest", "Starter Battery Voltage", "V", 0.001, helper.convert_int_factor),
        'I': ("Latest", "Current", "A", 0.001, helper.convert_int_factor),
        'P': ("Latest", "Power", "W", 1, helper.convert_int_factor),
        'T': ("Latest", "Battery Temperature", "°C", 1, helper.convert_int_factor),
        'CE': ("Latest", "Used Energy", "Ah", 0.001, helper.convert_int_factor),
        'SOC': ("Battery", "State Of Charge", "%", 0.1, helper.convert_int_factor),
        'TTG': ("Battery", "Time To Go", "min", 1, helper.convert_int_factor),
        'Alarm': ("Latest", "Alarm", "", 0, helper.convert_str_out),
        'AR': ("Latest", "Alarm Reason", "", WARN_AR, helper.convert_warn_ar),
        'BMV': ("Meta", "BMV", "", "", helper.convert_str_out),
        'FW': ("Meta", "Firmware Version", "", "", helper.convert_firmware),
        'MON': ("Meta", "MON ?", "", "", helper.convert_str_out),
    }

Einstellungen:
     0x00: ("set capacity", "Ah", 1, False),
    0x01: ("set charged voltage", "V", 1, False),
    0x02: ("set tail current", "A", 1, False),
    0x03: ("set charged detection time", "sec", 1, False),
    0x04: ("set charge eff. factor", "", 1, False),
    0x05: ("set peukert coefficient", "", 1, False),
    0x06: ("set current threshold", "%", 1, False),
    0x07: ("set time-to-go avg. per.", "sec", 1, False),
    0x08: ("set discharge floor", "V?", 1, False),
 */
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#ifndef STASSID
#define STASSID "SSID"
#define STAPSK  "PWD"
#endif

#define NTP_IP_ADR "192.168.188.1"    //NTP Server IP

#define LedPIN    2    //LED Output
#define RelayPIN  0   //SmartShunt Relay for trigger an action!

//#define DEBUG         //Debug Output

//------------------------------------------------------------------------------
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <ESPAsyncTCP.h>            //https://github.com/me-no-dev/ESPAsyncTCP
#include <ESPAsyncWebServer.h>      //https://github.com/me-no-dev/ESPAsyncWebServer
#include <ArduinoJson.h>
#include <AsyncElegantOTA.h>

#include <NTPClient.h>
#include <WiFiUdp.h>

#if defined(DEBUG)
#include <SoftwareSerial.h>
SoftwareSerial Debug(3, 1); // RX, TX
#endif

//------------------------------------------------------------------------------
#include "VEDirectClient.h"
void Callback(uint16_t id, int32_t value);
VEDirectClient ve(Serial, Callback);

//------------------------------------------------------------------------------
#include "index_page.h"
#include "INA226.h"

//------------------------------------------------------------------------------
AsyncWebServer server(80);

unsigned long previousMillis = 0;
unsigned long lastUpdateMillis = 0;   //Update Shunt Data timer

uint16_t LastMeasurements = 0;    //Number of Last Measurements

//------------------------------------------------------------------------------
float ADSA0 = 0.0;
float ADSA1 = 0.0;

unsigned long Measurementcount = 0; 
uint32_t BatVolt = 0;      //0.01 Volt  Sum of all mesuments!
float BatCurrent = 0;   //0.01 Amper  Sum of all mesurements!

float mVShunt = 75.0;  //Shunt mV
float ShuntCorrection = 0.0; //mV Correction

//------------------------------------------------------------------------------
const char* ssid = STASSID;
const char* password = STAPSK;

//------------------------------------------------------------------------------
const long utcOffsetInSeconds = 3600;

String daysOfTheWeek[7] = {"So", "Mo", "Di", "Mi", "Do", "Fr", "Sa"};

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_IP_ADR, utcOffsetInSeconds);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void getShuntData() {
    Measurementcount++;   //count numer of measurements

    //------------------------------------------------------------------------------
    //get values from INA226
    ina226.readAndClearFlags();
    ADSA0 = ina226.getShuntVoltage_mV() * (-1.0);
    //ve.BatCurrent = (ina226.getCurrent_mA() / 1000.0) * (-1);
    //Watt = (ina226.getBusPower() / 1000.0) * (-1);
    BatVolt  += (ina226.getBusVoltage_V() * 2.0) * 100;   // *100 make number positiv 0.01V
    
    ADSA1 = ina226.overflow;    //ERROR Output!

    BatCurrent += ((100.0 / mVShunt) * (ADSA0 + ShuntCorrection)) * 100;  // *100 to make number positiv 0.01A
  
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
String getUptimeString() {
  uint16_t days;
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;

#define SECS_PER_MIN  60
#define SECS_PER_HOUR 3600
#define SECS_PER_DAY  86400

  time_t uptime = millis() / 1000;

  seconds = uptime % SECS_PER_MIN;
  uptime -= seconds;
  minutes = (uptime % SECS_PER_HOUR) / SECS_PER_MIN;
  uptime -= minutes * SECS_PER_MIN;
  hours = (uptime % SECS_PER_DAY) / SECS_PER_HOUR;
  uptime -= hours * SECS_PER_HOUR;
  days = uptime / SECS_PER_DAY;

  char buffer[20];
  sprintf(buffer, "%4u days %02d:%02d:%02d", days, hours, minutes, seconds);
  return buffer;
}
//------------------------------------------------------------------------------
String getNTPString() {
  char buffer[20];
  sprintf(buffer, "%02d:%02d:%02d", timeClient.getHours(), timeClient.getMinutes(), timeClient.getSeconds());
  return buffer;
}
//------------------------------------------------------------------------------
void indexrequest(AsyncWebServerRequest *request) {
  request->send_P(200, "text/html", index_page);
}
//------------------------------------------------------------------------------
void indexpost(AsyncWebServerRequest *request) {
  /*
  int paramsNr = request->params(); // number of params (e.g., 1)
  Debug.println(paramsNr);
  Debug.println();
*/

  AsyncWebParameter *p = request->getParam(0); // Offset ZeroPointCorrect
  ve.SoC = atof(p->value().c_str()) ;
  ve.shunt.NowAh = (ve.shunt.BatAh / 100.0) * ve.SoC;
  
  p = request->getParam(1); // ve.shunt.Peukert
  ve.shunt.Peukert = atof(p->value().c_str());
  
  p = request->getParam(2); //ALARM LowChargeSoC
  ve.shunt.Entladeboden = atoi(p->value().c_str());

  p = request->getParam(3); //ALARM BatteryLowVoltage
  ve.shunt.AlarmLowVoltage = atof(p->value().c_str()) * 10;
  
  p = request->getParam(4); //MaxChargeVoltage
  ve.shunt.MaxChargeVoltage = atof(p->value().c_str()) * 10;
  
  p = request->getParam(5); //mVShunt
  mVShunt = atof(p->value().c_str());
  
  p = request->getParam(6); //Shunt mV correction
  ShuntCorrection = atof(p->value().c_str());
  
  p = request->getParam(7); //Battery capacity (Ah)
  ve.shunt.BatAh = atof(p->value().c_str());
  
  p = request->getParam(8); //Schweifstrom
  ve.shunt.Tailcurrent = atof(p->value().c_str()) * 10;

  p = request->getParam(9); //ChargeDetectionTime
  ve.shunt.ChargeDetectionTime = atoi(p->value().c_str());

  p = request->getParam(10); //ChargeEfficencyFactor
  ve.shunt.ChargeEfficencyFactor = atoi(p->value().c_str());


  request->send_P(200, "text/html", index_page);
}
//------------------------------------------------------------------------------
void indexreset(AsyncWebServerRequest *request) {
  ve.shunt.DeepestDischarge = ve.shunt.NowAh;
  ve.shunt.MinimumVoltage = ve.BatVolt;
  ve.shunt.MaximumVoltage = ve.BatVolt;
  ve.shunt.ChargedEnergy = 0.0;
  ve.shunt.DischargedEnergy = 0.0;
  ve.shunt.AutomaticSyncs = 0;
  //ve.shunt.ChargeCycles = 0;
  ve.shunt.FullDischarges = 0;
  ve.shunt.LastDischarge = ve.shunt.NowAh;
  ve.shunt.AvgDischarge = ve.shunt.NowAh;
  ve.shunt.TotalAhDrawn = 0.0;
  ve.shunt.NumLowVoltageAlarms = 0;
  ve.shunt.NumHighVoltageAlarms = 0;
  ve.shunt.dischargeAh = 0;
  request->send_P(200, "text/html", index_page);
}

//------------------------------------------------------------------------------
void xmlrequest(AsyncWebServerRequest *request) {
  String XML = F("<?xml version='1.0'?><xml>");
  XML += F("<A0>");
  XML += String(ADSA0,4);
  XML += F("</A0>");
  XML += F("<A1>");
  switch (ve.internalStatus) {
    case IDLE: XML += "Leerlauf"; break;
    case CHARGING: XML += "Laden"; break;
    case DISCHARGING: XML += "Entladen"; break;
    case FULLCHARGED: XML += "Voll"; break;
    case LOWSoC: XML += "SoC niedrig!"; break;
    default: XML += "UNKNOWN"; break;
  }
  XML += F("</A1>");
  XML += F("<capacity>");
  XML += String(ve.shunt.NowAh,4);
  XML += F("</capacity>");
  XML += F("<SoC>");
  XML += String(ve.SoC, 1);
  XML += F("</SoC>");
  XML += F("<VBat>");
  XML += String(ve.BatVolt);
  XML += F("</VBat>");
  XML += F("<current>");
  XML += String(ve.BatCurrent ,2);
  XML += F("</current>");
  XML += F("<power>");
  XML += String(ve.BatVolt * ve.BatCurrent, 0);
  XML += F("</power>");

  XML += F("<peukert>");
  XML += String(ve.shunt.Peukert);
  XML += F("</peukert>");
  XML += F("<LCSoC>");
  XML += String(ve.shunt.Entladeboden);
  XML += F("</LCSoC>");
  XML += F("<BLV>");
  XML += String(ve.shunt.AlarmLowVoltage / 10.0, 1);
  XML += F("</BLV>");
  XML += F("<MCV>");
  XML += String(ve.shunt.MaxChargeVoltage / 10.0, 1);
  XML += F("</MCV>");
  XML += F("<MCC>");
  XML += String(mVShunt);
  XML += F("</MCC>");
  XML += F("<MDC>");
  XML += String(ShuntCorrection);
  XML += F("</MDC>"); 
  XML += F("<cap>");
  XML += String(ve.shunt.BatAh);
  XML += F("</cap>"); 

  XML += F("<TC>");
  XML += String(ve.shunt.Tailcurrent / 10.0, 1);   //Schweifstrom
  XML += F("</TC>");
  XML += F("<CDT>");
  XML += String(ve.shunt.ChargeDetectionTime); //Zeit f. Ladezustand-Erkennung
  XML += F("</CDT>");
  XML += F("<CEF>");
  XML += String(ve.shunt.ChargeEfficencyFactor); //Ladewirkungsgrad
  XML += F("</CEF>");
  
  XML += F("<count>");
  XML += String(LastMeasurements);
  XML += F("</count>");
  XML += F("<ntp>");
  XML += String(daysOfTheWeek[timeClient.getDay()] + " " + getNTPString());
  XML += F("</ntp>");
  XML += F("<upt>");
  XML += getUptimeString();
  XML += F("</upt>");
  XML += F("<freeh>");
  XML += String(ESP.getFreeHeap());
  XML += F("</freeh>");
  XML += F("</xml>");
  request->send(200, "text/xml", XML);
}
//------------------------------------------------------------------------------
void SendJsonSite(AsyncWebServerRequest *request)  {
  String JsonString = "\0";
  StaticJsonDocument<2048> doc;
  //doc["A0"] = ADSA0;
  //doc["A1"] = ADSA1;
  doc["remain_cap"] = ve.shunt.NowAh;
  doc["SoC"] = ve.SoC;
  doc["BatVolt"] = (int32_t)(ve.BatVolt * 1000.0);
  doc["current"] = (int32_t)(ve.BatCurrent * 1000.0) ;
  doc["power"] = (int32_t)(ve.BatVolt * ve.BatCurrent * 10.0);
  doc["capacity"] = ve.shunt.BatAh;
  doc["TimeToGo"] = ve.TimeToGo;
  doc["ChargeCycles"] = ve.shunt.ChargeCycles;
  doc["TimeSinceLastFullCharge"] = ve.shunt.TimeSinceLastFullCharge;
  doc["MinimumVoltage"] = ve.shunt.MinimumVoltage;
  doc["MaximumVoltage"] = ve.shunt.MaximumVoltage;
  doc["ChargedEnergy"] = ve.shunt.ChargedEnergy;
  doc["DeepestDischarge"] = ve.shunt.DeepestDischarge;
  doc["TotalAhDrawn"] = ve.shunt.TotalAhDrawn;
  doc["DeepDischarge"] = ve.shunt.DeepestDischarge;
  doc["LastDischarge"] = ve.shunt.LastDischarge;
  doc["AvgDischarge"] = ve.shunt.AvgDischarge;
  doc["CVL"] = ve.shunt.MaxChargeVoltage;  //Ladespannungsbegrenzung
  doc["Shunt"] = mVShunt;  //Shunt mV
  doc["Correction"] = ShuntCorrection; //Correction value
  doc["Relay"] = ve.RelayState;
  doc["Alarm"] = ve.shunt.AlarmReason;
  doc["LV"] = ve.shunt.RelayLowVoltage / 10.0;
  doc["LVC"] = ve.shunt.RelayLowVoltageClear / 10.0;
  //doc["Store"] = preferences.freeEntries();
  //doc["discharge"] = dischargeAh;  //only for charge cycle calculation
  //doc["time"] = daysOfTheWeek[timeClient.getDay()] + " " + getNTPString();
  //doc["Uptime"] = getUptimeString();
  //doc["FreeHeap"] = ESP.getFreeHeap();
  serializeJson(doc, JsonString);
  request->send(200, "application/json", JsonString);
}
//------------------------------------------------------------------------------
void serverInit() {
  server.on("/", HTTP_GET, indexrequest);
  server.on("/", HTTP_POST, indexpost);
  server.on("/xml", HTTP_PUT, xmlrequest);
  server.on("/status", HTTP_GET, SendJsonSite);
  server.on("/reset", HTTP_GET, indexreset);
  server.onNotFound([](AsyncWebServerRequest * request) {
    request->send(404);
  });
  // Start ElegantOTA
  AsyncElegantOTA.begin(&server);
  
  server.begin();
}
//------------------------------------------------------------------------------
void setup() {
  #if defined(DEBUG)
  Debug.begin(115200);
  Debug.println();
  Debug.println("SmartShunt");
  #endif

  pinMode(RelayPIN, OUTPUT);
  pinMode(LedPIN, OUTPUT);
  digitalWrite(LedPIN, HIGH);
  digitalWrite(RelayPIN, LOW);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    //Debug.println("Connection Failed! Rebooting...");
    delay(1000);
    ESP.restart();
  }

  ArduinoOTA.setHostname("SmartShunt 100/75mV");
  ArduinoOTA.begin();

  #if defined(DEBUG)
  Debug.print("IP address: ");
  Debug.println(WiFi.localIP());
  #endif

  //Starte NTP client:
  timeClient.begin();

  //Starte ADS Kommunikation:
  INA_init();   
  
  //first time data report:
  getShuntData();

  //starte Webserver!
  serverInit();

  //starte VE.Direct
  ve.begin();
  Serial.swap();    //change to VE.Direct Port

  #if defined(DEBUG)
  Debug.println("Startup Ready!");
  #endif
}

//------------------------------------------------------------------------------
void loop() {
  ArduinoOTA.handle();
  timeClient.update();
  
  getShuntData();

  unsigned long currentMillis = millis();
  if ((currentMillis - lastUpdateMillis) >= 10) {
    lastUpdateMillis = currentMillis;
    ve.BatVolt = float((BatVolt / Measurementcount)) / 100.0;         //0.01 Volt  
    ve.BatCurrent = float((BatCurrent / Measurementcount)) / 100.0;   //0.01 Amper 
    ve.update();
    LastMeasurements = Measurementcount;
    Measurementcount = 0;
    BatVolt = 0;
    BatCurrent = 0;
  }
  if ((currentMillis - previousMillis >= 100) && (ve.VeDirectStatus != DISCONNECTED)) {
    digitalWrite(LedPIN, ve.RelayState);   //OFF - false
  }
  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;

    //set Relay:
    digitalWrite(RelayPIN, ve.RelayState);

    if (ve.VeDirectStatus != DISCONNECTED)    //connected?
      digitalWrite(LedPIN, !ve.RelayState);   //ON - true
    else digitalWrite(LedPIN, HIGH);   //OFF  
  }

  if (WiFi.status() != WL_CONNECTED) {
    ESP.restart();
  }
  
}


void Callback(uint16_t id, int32_t value) {
  //toDo
}
