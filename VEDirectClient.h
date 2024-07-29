#ifndef VEDIRECTCLIENT_H_
#define VEDIRECTCLIENT_H_

#include "Arduino.h"
#include <EEPROM.h>

//Emulate Device:
#define FirmwareVersion "0414"
/* Product ID:      
 *     * BMV712 = 0xA381
       * BMV-712 Smart Rev2 = 0xA383
       * SmartShunt 500A/50mV = 0xA389
       */
#define ProductID       "0xA389"    //SmartShunt 500A/50mV 
//serial number notation is LLYYMMSSSSS, where LL=location code, 
//YYWW=production date stamp (year, week) and SSSSS=unique part of the serial number.
#define SerialNr        "HQ2310PG0AS"; 
#define ModelName       "SmartShunt 100/75mV" 

//VE.Direct:
#define VEDirect_kBaud (19200)
#define hexBuffLen 100      // Maximum size of hex frame - max payload 34 byte (=68 char) + safe buffer
#define UpdateTEXTInterval 900 //each 1 sec Update Interval for VE.Direct Text messages
#define UpdateHistoryInterval  10000 //each 10 sec Update the History Data

#define DEFAULT_TailCurrent 20
#define DEFAULT_Peukert 1.04

#define VEDirect_kPingCommand (0x01)
#define VEDirect_kRestartCommand (0x06)
#define VEDirect_kSetCommand (0x08)
#define VEDirect_kGetCommand (0x07)
#define VEDirect_kAsyncCommand (0x0A)

// subset of register definitions from BlueSolar-HEX-protocol-MPPT.pdf
#define VEDirect_kBatterySense   (0x2002)
#define VEDirect_kPanelVoltage   (0xEDBB)
#define VEDirect_kChargeVoltage  (0xEDD5)
#define VEDirect_kPanelCurrent   (0xEDBD)
#define VEDirect_kChargeCurrent  (0xEDD7)
#define VEDirect_kPanelPower     (0xEDBC)

#define VEDirect_kNetworkMode    (0x200E)
#define VEDirect_VoltageSetpoint (0x2001)
#define VEDirect_CurrentLimit    (0x2015)
#define VEDirect_kDeviceState    (0x0201)

#define VEDirect_kExternalControlMode (0x05)

//VE.Direct Status
#define CONNECTED 0x0A
#define DISCONNECTED 0x00

//Internal Battery Status output:
#define IDLE 0x00             //Leerlauf
#define CHARGING 0x01         //Laden
#define DISCHARGING 0x10      //Entladen
#define FULLCHARGED 0x66      //Geladen
#define LOWSoC 0x55           //SoC unter Entladeboden!

/*
 *  hexIsValid
 *  This function compute checksum and validate hex frame
 */
#define ascii2hex(v) (v-48-(v>='A'?7:0))

typedef void (*receiveCallback)(uint16_t id, int32_t value);

class VEDirectClient {
public:

  VEDirectClient(HardwareSerial &serial, receiveCallback receive);

  void begin();
  void update();
  void save();    //writes structure data to FLASH MEMORY

  uint8_t internalStatus = 0xFF;            //internal Status is unknown!
  uint8_t VeDirectStatus = DISCONNECTED;    //unknown!

  //calculated values on each run:
  float BatVolt = 0.0;      //0.01 Volt
  float BatCurrent = 0.0;   //0.01 Amper
  int32_t TimeToGo = 0;   //in seconds
  float SoC = 0.0;
  bool RelayState = false;
  bool FullDischarge = false;   //Number of full discharges: The number of full discharges. A full discharge is counted when the state of charge reaches 0%.
  
  //long term values store inside EEPROM:
  struct shuntvar {  
    float NowAh = 0;          //Amperstundenmessung aktuell (could be also negativ if BatAh is to small!)
    
    float dischargeAh = 0;    //NOT IN USE ANYMORE!!
    
    //History:
    float DeepestDischarge = 99999.0; //0.1Ah
    float LastDischarge = 0.0; //0.1Ah   
    float AvgDischarge = 0.0; //0.1Ah      
    uint32_t ChargeCycles = 0;      //Ladezyklen insgesamt
    uint32_t FullDischarges = 0;    //Anzahl der vollständigen Entladungen
    unsigned long TimeSinceLastFullCharge = 0;   //in seconds
    float MinimumVoltage = 99.9;      //0.01V
    float MaximumVoltage = 0.0;       //0.01V
    float DischargedEnergy = 0.0;     //0.01kWh
    float ChargedEnergy = 0.0;        //0.01kWh
    float TotalAhDrawn = 0.0;         //Ah
    uint16_t AutomaticSyncs = 0;     //Sync if SoC 100%
    uint32_t NumLowVoltageAlarms = 0;     //Low voltage alarms: The number of low voltage alarms.
    uint32_t NumHighVoltageAlarms = 0;    //High voltage alarms: The number of high voltage alarms.
    
    uint16_t BatAh = 110;   //Amperstunden der Batterie
    uint16_t Tailcurrent = 40;   //Schweifstrom 4.0%
    float CurrentThreshold = 0.08;  //Strom f. Ladezustand-Erkennung
    uint16_t ChargeDetectionTime = 3; //Zeit f. Ladezustand-Erkennung
    uint16_t ChargeEfficencyFactor = 99; //Ladewirkungsgrad
    float Peukert = 1.02;     //Peukert-Exponent zwischen 1,00 und 1,50 anpassen
    uint16_t TimeToGoDelta = 1; //minuten
    uint16_t MaxChargeVoltage = 564;  //Ladespannungsbegrenzung
    uint16_t Entladeboden = 25;  //Berechnung der "verbleibdenen Zeit" (Relay Low Soc Set)
    uint16_t EntladebodenClear = 25;  //Relay Low Soc Clear

    //Alarm:
    bool AlarmBuzzer = 0;
    uint32_t AlarmReason = 0;    //No Alarm!
    uint16_t AlarmLowSoc = 0;
    uint16_t AlarmLowSocClear = 0;
    uint16_t AlarmLowVoltage = 0;  
    uint16_t AlarmLowVoltageClear = 0;
    uint16_t AlarmHighVoltage = 0;
    uint16_t AlarmHighVoltageClear = 0;

    //Relay:
    uint16_t RelayEnableTime = 0;
    uint16_t RelayDisableTime = 0;
    uint16_t RelayLowVoltage = 0;
    uint16_t RelayLowVoltageClear = 0;
    uint16_t RelayHighVoltage = 0;
    uint16_t RelayHighVoltageClear = 0;
  } shunt;
 
private:
  HardwareSerial &serialPort;
  receiveCallback rxCallback;
  
  unsigned long timemillis = 0;	//Interval 1 sec.
  unsigned long historymillis = 0;  //Interval 10 sec.
  void makeDataHEX();
  bool checkRegID (uint16_t id);
  void rxData(uint8_t inbyte);

  uint8_t ConvertToASCII(uint8_t hex);
  void VEsendData(uint8_t message[], uint8_t size);
  
  
  uint8_t recdata[hexBuffLen];  //HEX read Buffer
  uint8_t recdatacount = 0;     //HEX Buffer size
  bool start = false;         //HEX Data received?
  uint8_t IntData[1000];      //TEXT Data output
  uint16_t IntDataCount = 0;  //TEXT Data size

  void AddData (uint8_t d);
  void AddLabel (String FieldLabel, String FieldValue); 
  void AddCRC();
  void sendRegular();

  unsigned long MessungMillis = 0;    //Time when last ADC read was!
  unsigned long FullTimeMillis = 0;    //Counter for FullCharge detection wait
  float AvgCurrent = 0.0;   //mean value of current for TimeToGo
  unsigned long AvgCurrentMillis = 0; //time for the mean value
  bool AutoSyncDetect = false;  //Check if there was a sync

  unsigned long RelayMillis = 0;    //timer to delay relay
  bool RelayNextState = false;      //Relay State if timer is ready

};

#endif //  VEDIRECTCLIENT_H_
