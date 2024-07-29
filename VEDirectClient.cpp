#include "VEDirectClient.h"

VEDirectClient::VEDirectClient(HardwareSerial &serial, receiveCallback receive)
    : serialPort{serial}, rxCallback{receive} {};

/*
void mpptCallback(uint16_t id, int32_t value);
VEDirect mppt(Serial1, mpptCallback);

rxCallback(id, ved_getU8(&rxBuffer));
*/	

void VEDirectClient::begin() {
	serialPort.begin(VEDirect_kBaud);

  EEPROM.begin(4096);     //Intitialize EEprom with specified size
  EEPROM.get(0, shunt);   //reads structure data from FLASH MEMORY

  //check if values are correct from EEPROM?
  if (shunt.AlarmReason > 9 || shunt.ChargeEfficencyFactor > 100 || shunt.Peukert > 1.50 || shunt.Entladeboden >100) {
    //Reset EEPROM values!
    shunt.AlarmReason = 0;
    shunt.AlarmLowSoc = 0;
    shunt.AlarmLowSocClear = 0;
    shunt.AlarmLowVoltage = 480;  
    shunt.AlarmLowVoltageClear = 490;
    shunt.AlarmHighVoltage = 580;
    shunt.AlarmHighVoltageClear = 560;
    shunt.RelayEnableTime = 0;
    shunt.RelayDisableTime = 0;
    shunt.RelayLowVoltage = 0;
    shunt.RelayLowVoltageClear = 0;
    shunt.RelayHighVoltage = 0;
    shunt.RelayHighVoltageClear = 0;
    shunt.Entladeboden = 25;  //Berechnung der "verbleibdenen Zeit" (Relay Low Soc Set)
    shunt.EntladebodenClear = 25;  //Relay Low Soc Clear
    shunt.BatAh = 100;   //Amperstunden der Batterie
    shunt.Tailcurrent = 20;   //Schweifstrom 2.0%
    shunt.CurrentThreshold = 0.08;  //Strom f. Ladezustand-Erkennung
    shunt.ChargeDetectionTime = 3; //Zeit f. Ladezustand-Erkennung
    shunt.ChargeEfficencyFactor = 98; //Ladewirkungsgrad
    shunt.Peukert = 1.05;     //Peukert-Exponent zwischen 1,00 und 1,50 anpassen
    shunt.TimeToGoDelta = 1; //minuten
    shunt.MaxChargeVoltage = 564;
    shunt.DeepestDischarge = 0.0; //0.1Ah
    shunt.LastDischarge = 0.0; //0.1Ah   
    shunt.AvgDischarge = 0.0;
    shunt.ChargeCycles = 0;
    shunt.FullDischarges = 0;
    shunt.TimeSinceLastFullCharge = 0;   //in seconds
    shunt.MinimumVoltage = 99.9;      //0.01V
    shunt.MaximumVoltage = 0.0;       //0.01V
    shunt.DischargedEnergy = 0.0; //0.01kWh
    shunt.ChargedEnergy = 0.0; //0.01kWh
    shunt.TotalAhDrawn = 0.0; //Ah
    shunt.AutomaticSyncs = 0;    
    shunt.NumLowVoltageAlarms = 0;
    shunt.NumHighVoltageAlarms = 0;
    shunt.NowAh = shunt.BatAh;          //Amperstundenmessung aktuell
    //write to EEPROM:
    save();
  }

  MessungMillis = millis();   //set time last data
}

void VEDirectClient::update() {
  
    //----------------------------LIVE DATA HANDLE -----------------------------------------
    //Current threshold:
    if ((BatCurrent < shunt.CurrentThreshold) && (BatCurrent > (shunt.CurrentThreshold * (-1))))
      BatCurrent = 0;    //reduce noise when current is low!
      
    float Watt = BatVolt * BatCurrent;

    //------------------------------------------------------------------------------
    //calculate Ah for battery:
    if (BatCurrent  < 0.0) {   //discharge:
      float discharge =  ((millis() - MessungMillis) * pow(BatCurrent  * (-1),shunt.Peukert) ) / 3600000.0; //pow(base, exponent) and
       //only for charge cycle calculation:
      shunt.TotalAhDrawn += discharge;
      shunt.LastDischarge += discharge;   //calculate discharged Ah (last, deepest, avg)
      
      shunt.NowAh +=  discharge * (-1);   // -1 make it again discharge
      //DischargedEnergy in kWh
      shunt.DischargedEnergy += ( ((millis() - MessungMillis)*((Watt / 1000.0) * (-1) )) / 3600000.0);
    }
    if (BatCurrent  > 0.0) {   //charge
      shunt.NowAh += ( (((millis() - MessungMillis)*BatCurrent ) / 3600000.0) / 100 )  * shunt.ChargeEfficencyFactor;
      //ChargedEnergy in kWh
      shunt.ChargedEnergy += (((millis() - MessungMillis)*(Watt/1000.0)) / 3600000.0);
    }
    MessungMillis = millis();   //----------TIMING neue Messung ---------------

    //------------------------------------------------------
    //make internal Status:
    if (BatCurrent > 0.0)
      internalStatus = CHARGING;
    if (BatCurrent < 0.0)
      internalStatus = DISCHARGING;
    if (BatCurrent == 0.0)
      internalStatus = IDLE;  
    if (SoC <= shunt.Entladeboden)
      internalStatus = LOWSoC;  

    //------------------------------------------------------------------------------
    //check full charge?
    //The battery is considered as fully charged once the charge current has dropped to less than this “Tail current” parameter. The “Tail current” parameter is expressed as a percentage of the battery capacity.
    if ( ((BatVolt * 10.0) >= shunt.MaxChargeVoltage) && ((BatCurrent * 1000) <= (((shunt.BatAh / 10.0) * shunt.Tailcurrent)) * 1000) &&  (BatCurrent > 0.0) ) {
      if (((millis() - FullTimeMillis) / 1000) >= (shunt.ChargeDetectionTime * 60)) {    //wait X minutes until detection success!
        if (shunt.TimeSinceLastFullCharge != 0 && AutoSyncDetect == false) {
          //we are now full!
          shunt.NowAh = shunt.BatAh;    //reset remain capacity
          shunt.AutomaticSyncs++;   //Synchronisations: The number of automatic synchronisations. A synchronisation is counted every time the state of charge drops below 90% before a synchronisation occurs.
          AutoSyncDetect = true;   //Sync count
          if (shunt.TimeSinceLastFullCharge > 60)   //only calculate if last Full charge was 1h away!
            shunt.AvgDischarge = (shunt.AvgDischarge + shunt.LastDischarge) / 2;
          shunt.LastDischarge = 0;  //make the discharge values new:
        }
        shunt.TimeSinceLastFullCharge = 0;    //reset timer
        internalStatus = FULLCHARGED;     //set status
      }
    }
    else {
      if ( ((BatVolt * 10.0) < shunt.MaxChargeVoltage) && (BatCurrent > 0.0) ) {  //charging and under MaxChargeVoltage -> free for new sync:
        AutoSyncDetect = false;
        FullTimeMillis = millis();
      }
    }
    //------------------------------------------------------------------------------
    //correct wrong values
    if (shunt.NowAh > shunt.BatAh) {    //alreday over capacity!
      SoC = 100.0;
      shunt.NowAh = shunt.BatAh;
    }
    else if (shunt.NowAh < 0.0)           //already under capacity!
      SoC = 0.0;  
    else SoC = shunt.NowAh / ((float)shunt.BatAh / 100.0);        //make SoC

    //------------------------------------------------------------------------------ 
    //TimeToGo (Restlaufzeit):
    if (AvgCurrent == 0.0)
      AvgCurrent = BatCurrent;
    else AvgCurrent = (AvgCurrent + BatCurrent ) / 2.0;
    if ((millis() - AvgCurrentMillis) > (shunt.TimeToGoDelta * 60000)) {
      if (AvgCurrent >= 0.0)  //charging til 100%
        TimeToGo = ( ((shunt.BatAh-shunt.NowAh) / ((AvgCurrent / 100) * shunt.ChargeEfficencyFactor) )*60)*60;    // h to min to sec
      //turn around:  
      AvgCurrent = AvgCurrent * (-1);
      if (AvgCurrent >= 0.0) {  //discharging to dicharge floor
        float TimeToGoAh = shunt.NowAh - (((float)shunt.BatAh / 100.0) *  (float)shunt.Entladeboden);
        if (TimeToGoAh <= 0.0)
          TimeToGo = 0;   //we are already under shunt.Entladeboden capacity!
        else TimeToGo = ( (TimeToGoAh / pow(AvgCurrent, shunt.Peukert) )*60)*60;    // h to min to sec
      }
      AvgCurrentMillis = millis();
      AvgCurrent = 0.0; //reset
    }

  
  //----------------------------VE.DIRECT DATA RECEIVE -----------------------------------------
	if (serialPort.available()) {
		rxData(serialPort.read());
    VeDirectStatus = CONNECTED;
	}

  //----------------------------Update VE.DIRECT DATA -----------------------------------------
	if ( (millis() - timemillis) >= UpdateTEXTInterval) { //1. sec
		timemillis = millis();
		digitalWrite(2, LOW);   //ON
		sendRegular();  //VE.Direct TEXT replay

    //prüfen VE.DIRECT connection:
    if (VeDirectStatus != DISCONNECTED)
      VeDirectStatus--;     //reduce Status!

    //----------------------------Update VE.DIRECT HISTORY DATA -----------------------------------------
    //Calculate the history data:
    if ( (millis() - historymillis) >= UpdateHistoryInterval) {  //10 sec.
      shunt.TimeSinceLastFullCharge += (millis() - historymillis) / 1000;
      historymillis = millis();

      //Number of full discharges: The number of full discharges. A full discharge is counted when the state of charge reaches 0%.
      if ((shunt.NowAh <= 0.0) && (FullDischarge == false)) {  
        shunt.FullDischarges++;   //was a full Discharge!
        FullDischarge = true;
      }
      else if ((shunt.NowAh > 1.0) && (FullDischarge == true)) 
        FullDischarge = false;

      //Deepest discharge: The battery monitor remembers the deepest discharge and each time the battery is discharged deeper the old value will be overwritten.
      if ((shunt.BatAh - shunt.NowAh) > shunt.DeepestDischarge)    
        shunt.DeepestDischarge = shunt.BatAh - shunt.NowAh;

      //Total charge cycles: The number of charge cycles over the lifetime of the battery monitor. A charge cycle is counted every time the state of charge drops below 65% and then rises above 90%.
      //if (SoC < 65.0)
      //if (SoC > 90.0)
      shunt.ChargeCycles = shunt.TotalAhDrawn / shunt.BatAh;  //calculate  Charge cycle

      //check Bat Voltage:
      if ((BatVolt < shunt.MinimumVoltage) && (BatVolt != 0))  //Min battery voltage: The lowest battery voltage.
        shunt.MinimumVoltage = BatVolt;
      if (BatVolt > shunt.MaximumVoltage)   //Max battery voltage: The highest battery voltage.
        shunt.MaximumVoltage = BatVolt;
  
    //------------------------------------------------------------------------------
    //set Alarm Values
    if (BatVolt <= (shunt.AlarmLowVoltage / 10.0) && shunt.AlarmLowVoltage != 0) {   //Batteriespannung niedrig = 1
      if (bitRead(shunt.AlarmReason,0) == 0)
        shunt.NumLowVoltageAlarms++;    //count Alarm      
      bitWrite(shunt.AlarmReason,0,1);
    }
    if (BatVolt >= (shunt.AlarmLowVoltageClear / 10.0)) //clear
      bitWrite(shunt.AlarmReason,0,0); 
      
    if (BatVolt >= (shunt.AlarmHighVoltage / 10.0) && shunt.AlarmHighVoltage != 0) {   //Max Charge Voltage = 2
      if (bitRead(shunt.AlarmReason,1) == 0)
        shunt.NumHighVoltageAlarms++;    //count Alarm      
      bitWrite(shunt.AlarmReason,1,1);
    }
    if (BatVolt <= (shunt.AlarmHighVoltageClear / 10.0))    //clear
      bitWrite(shunt.AlarmReason,1,0);  
    
    if (SoC <= (shunt.AlarmLowSoc / 10.0))  //Low SoC = 5 
      bitWrite(shunt.AlarmReason,2,1);
    if (SoC >= (shunt.AlarmLowSocClear / 10.0))  //clear 
      bitWrite(shunt.AlarmReason,2,0); 

    //------------------------------------------------------------------------------
    //Update Relay State
    if (shunt.Entladeboden != shunt.EntladebodenClear) {  //Active?
        if ((SoC >= (shunt.EntladebodenClear / 10.0)) && (RelayNextState == true))
          RelayNextState = false;  
    }
    if ((shunt.RelayLowVoltage != 0) && (shunt.RelayLowVoltageClear != 0)) {
        if ((BatVolt >= (shunt.RelayLowVoltageClear / 10.0)) && (RelayNextState == true))
          RelayNextState = false;       
    }
    if ((shunt.RelayHighVoltage != 0) && (shunt.RelayHighVoltageClear != 0)) {
        if ((BatVolt <= (shunt.RelayHighVoltageClear / 10.0)) && (RelayNextState == true))
          RelayNextState = false;       
    }
    if (shunt.Entladeboden != shunt.EntladebodenClear) {  //Active?
       if ((SoC <= (shunt.Entladeboden / 10.0)) && (RelayNextState == false))
          RelayNextState = true;
    }
    if ((shunt.RelayLowVoltage != 0) && (shunt.RelayLowVoltageClear != 0)) {
        if ((BatVolt <= (shunt.RelayLowVoltage / 10.0)) && (RelayNextState == false))
          RelayNextState = true;
    }
    if ((shunt.RelayHighVoltage != 0) && (shunt.RelayHighVoltageClear != 0)) {
      if ((BatVolt >= (shunt.RelayHighVoltage / 10.0)) && (RelayNextState == false))
          RelayNextState = true;
    }
    //Delay the Relay state:
    if (RelayState != RelayNextState) {   //State should change?
      if (RelayNextState == true) { //Check Relay Enable Time?
        if ( (millis() - RelayMillis) >= (shunt.RelayEnableTime * 60000) )
          RelayState = RelayNextState;
      }
      else {                        //Check Relay Disable Time?
        if ( (millis() - RelayMillis) >= (shunt.RelayDisableTime * 60000) )
          RelayState = RelayNextState;
      }
    }
    else RelayMillis = millis();    //get time we have no change!

    save();  //writes structure data to FLASH MEMORY

    } //ENDE HISTORY

	} //ENDE Regular 1 sec

}

//writes structure data to FLASH MEMORY
void VEDirectClient::save() {
  EEPROM.put(0, shunt);  
  EEPROM.commit();   //writes structure data to FLASH MEMORY
}

void VEDirectClient::makeDataHEX() {
  uint8_t counter = 1;
  for (uint8_t i = 1; i < recdatacount; i++) {
    if (i % 2 == 0) {
      recdata[counter] |= recdata[i];
      counter++;
    }
    else recdata[counter] = recdata[i] << 4;  
  }
  recdatacount = counter;
}

bool VEDirectClient::checkRegID (uint16_t id) {
  return ( (recdata[2] == (id >> 8)) && (recdata[1] == (id & 0xFF)) );
}

void VEDirectClient::rxData(uint8_t inbyte) {               // byte of serial data to be passed by the application
    //Stop BIT erkannt
    if (inbyte == 0x0a) {    // '\n'
      //recdatacount = 0xFF;    //FINISH!
      start = false;
      //WebSerial.println();
      makeDataHEX();
    }
    if (start == true) {
     // WebSerial.print(String(ascii2hex(inbyte), HEX));
     // if (recdatacount % 2 == 0)
     //   WebSerial.print(" ");
      recdata[recdatacount] = ascii2hex(inbyte);
      recdatacount++;
    }
    //Start BIT erkannt!
    if (inbyte == 0x3a) {    // ':'
      //start
      start = true;
      recdatacount = 0;
     // WebSerial.print("RX:");
    }
    

  if (start == false && recdatacount != 0) {
    if (recdata[0] == 0x01 && recdata[1] == 0x54) { //PING
        uint8_t data[] = {0x05,0x01,0x44};
        VEsendData(data,3);
    }
    if (recdata[0] == 0x03 && recdata[1] == 0x52) { //Application version (Firmware)
        uint8_t data[] = {0x01,0x01,0x44};
        VEsendData(data,3);
    }
    else if (checkRegID(0x010A)) {   //Serial number (String32)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
      String Nr = SerialNr;
      for (byte i = 0; i < Nr.length(); i++) {
        if (i < 32)
          data[4+i] = Nr[i];     
      }
      VEsendData(data,36);
    }
    else if (checkRegID(0x010B)) {   //Model Name (String32)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
      String Name = ModelName;
      for (byte i = 0; i < Name.length(); i++) {
        if (i < 32)
          data[4+i] = Name[i];     
      }
      VEsendData(data,36);
    }
    else if (checkRegID(0x010C)) {   //Description(String20)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x01};
      VEsendData(data,4);
    }
    else if (checkRegID(0x0120)) {   //Uptime in seconds (Un32)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00, millis()/1000, (millis()/1000) >> 8, (millis()/1000) >> 16, (millis()/1000) >> 24};
      VEsendData(data,8);
    }
    else if (checkRegID(0x0150)) {   //Bluetooth capabilities
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,0x00};  //HAS SUPPORT FOR BLE MODE
      VEsendData(data,5);
    }
    //---------------------------------------------------------------------------
    //Historic data registers:
    else if (checkRegID(0x0300)) {   //Depth of the deepest discharge  (Sn32)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,round(shunt.DeepestDischarge * 10.0),((int32_t)(shunt.DeepestDischarge * 10.0)) >> 8, ((int32_t)(shunt.DeepestDischarge * 10.0)) >> 16, ((int32_t)(shunt.DeepestDischarge * 10.0)) >> 24};   //0.1Ah
      VEsendData(data,8);
    }
    else if (checkRegID(0x0301)) {   //Depth of the last discharge  (Sn32)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,round(shunt.LastDischarge * 10.0),((int32_t)(shunt.LastDischarge * 10.0)) >> 8, ((int32_t)(shunt.LastDischarge * 10.0)) >> 16, ((int32_t)(shunt.DeepestDischarge * 10.0)) >> 24};   //0.1Ah
      VEsendData(data,8);
    }
    else if (checkRegID(0x0302)) {   //Depth of the average discharge  (Sn32)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,round(shunt.AvgDischarge * 10.0),((int32_t)(shunt.AvgDischarge * 10.0)) >> 8, ((int32_t)(shunt.AvgDischarge * 10.0)) >> 16, ((int32_t)(shunt.AvgDischarge * 10.0)) >> 24};   //0.1Ah
      VEsendData(data,8);
    }
    else if (checkRegID(0x0303)) {   //Number of cycles  (Un32)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.ChargeCycles,((int32_t)(shunt.ChargeCycles)) >> 8, ((int32_t)(shunt.ChargeCycles)) >> 16, ((int32_t)(shunt.ChargeCycles)) >> 24};  
      VEsendData(data,8);
    }
    else if (checkRegID(0x0304)) {   //Number of full discharges  (Un32)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.FullDischarges,((int32_t)(shunt.FullDischarges)) >> 8, ((int32_t)(shunt.FullDischarges)) >> 16, ((int32_t)(shunt.FullDischarges)) >> 24};  
      VEsendData(data,8);
    }
    else if (checkRegID(0x0305)) {   //Cumulative Amp Hours  (Sn32)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,round(shunt.TotalAhDrawn * 10.0),((int32_t)(shunt.TotalAhDrawn * 10.0)) >> 8, ((int32_t)(shunt.TotalAhDrawn * 10.0)) >> 16, ((int32_t)(shunt.TotalAhDrawn * 10.0)) >> 24};   //0.1Ah
      VEsendData(data,8);
    }
    else if (checkRegID(0x0306)) {   //Min Voltage  (Sn32)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,round(shunt.MinimumVoltage * 100.0),((int32_t)(shunt.MinimumVoltage * 100.0)) >> 8, ((int32_t)(shunt.MinimumVoltage * 100.0)) >> 16, ((int32_t)(shunt.MinimumVoltage * 100.0)) >> 24};    //0.01V
      VEsendData(data,8);
    }
    else if (checkRegID(0x0307)) {   //Max Voltage  (Sn32)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,round(shunt.MaximumVoltage * 100.0),((int32_t)(shunt.MaximumVoltage * 100.0)) >> 8, ((int32_t)(shunt.MaximumVoltage * 100.0)) >> 16, ((int32_t)(shunt.MaximumVoltage * 100.0)) >> 24};    //0.01V
      VEsendData(data,8);
    }
    else if (checkRegID(0x0308)) {   //Seconds since full charge  (Un32)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.TimeSinceLastFullCharge,((int32_t)(shunt.TimeSinceLastFullCharge)) >> 8, ((int32_t)(shunt.TimeSinceLastFullCharge)) >> 16, ((int32_t)(shunt.TimeSinceLastFullCharge)) >> 24};  
      VEsendData(data,8);
    }
    else if (checkRegID(0x0309)) {   //Number of automaic syncs  (Un32)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.AutomaticSyncs,((int32_t)(shunt.AutomaticSyncs)) >> 8, ((int32_t)(shunt.AutomaticSyncs)) >> 16, ((int32_t)(shunt.AutomaticSyncs)) >> 24};  
      VEsendData(data,8);
    }
    else if (checkRegID(0x030A)) {   //Number of Low Voltage Alarms (Un32)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.NumLowVoltageAlarms,((int32_t)(shunt.NumLowVoltageAlarms)) >> 8, ((int32_t)(shunt.NumLowVoltageAlarms)) >> 16, ((int32_t)(shunt.NumLowVoltageAlarms)) >> 24};  
      VEsendData(data,8);
    }
    else if (checkRegID(0x030B)) {   //Number of High Voltage Alarms  (Un32)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.NumHighVoltageAlarms,((int32_t)(shunt.NumHighVoltageAlarms)) >> 8, ((int32_t)(shunt.NumHighVoltageAlarms)) >> 16, ((int32_t)(shunt.NumHighVoltageAlarms)) >> 24};  
      VEsendData(data,8);
    }
    else if (checkRegID(0x0310)) {   //Amount of discharged energy  (Un32)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,round(shunt.DischargedEnergy * 100.0),((int32_t)(shunt.DischargedEnergy * 100.0)) >> 8, ((int32_t)(shunt.DischargedEnergy * 100.0)) >> 16, ((int32_t)(shunt.DischargedEnergy * 100.0)) >> 24};    //0.01kWh
      VEsendData(data,8);
    }
    else if (checkRegID(0x0311)) {   //Amount of charged energy  (Un32)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,round(shunt.ChargedEnergy * 100.0),((int32_t)(shunt.ChargedEnergy * 100.0)) >> 8, ((int32_t)(shunt.ChargedEnergy * 100.0)) >> 16, ((int32_t)(shunt.ChargedEnergy * 100.0)) >> 24};    //0.01kWh
      VEsendData(data,8);
    }
    //---------------------------------------------------------------------------
    //Monitor related registers:
    else if (checkRegID(0xED8D)) {   //Main Voltage  (Sn16)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,round(BatVolt * 100.0),((int16_t)(BatVolt * 100.0)) >> 8};   //0.01V
      VEsendData(data,6);
    }
    else if (checkRegID(0xED7D)) {   //Aux (starter) Voltage (1)    (Sn16)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x02};   //Not Supported
      VEsendData(data,4);
    }
    else if (checkRegID(0xED8F)) {   //Current   (Sn16)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,round(BatCurrent * 10.0),((int16_t)(BatCurrent * 10.0)) >> 8};   //0.1A
      VEsendData(data,6);
    }
    else if (checkRegID(0xED8C)) {   //Current (2, 3)     (Sn16)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x02};   //Not Supported
      VEsendData(data,4);
    }
    else if (checkRegID(0xED8E)) {   //Power     (Sn16)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,round(BatVolt * BatCurrent * 10.0),((int16_t)(BatVolt * BatCurrent * 10.0)) >> 8};   //W
      VEsendData(data,6);
    }
    else if (checkRegID(0xEEFF)) {   //Consumed Ah     (Sn32)
      float ConsumedAh = shunt.BatAh - shunt.NowAh;
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,round(ConsumedAh * 10.0),((int32_t)(ConsumedAh * 10.0)) >> 8, ((int32_t)(ConsumedAh * 10.0)) >> 16, ((int32_t)(ConsumedAh * 10.0)) >> 24};   //W
      VEsendData(data,8);
    }
    else if (checkRegID(0x0FFF)) {   //SoC     (Un16)
      if (recdata[0] == 0x08) { //save
        SoC = (int16_t)(recdata[4] | (recdata[5] << 8)) / 100.0;
        shunt.NowAh = (shunt.BatAh / 100.0) * SoC;
      }
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,round(SoC * 100),((int16_t)(SoC * 100)) >> 8};   //0.01%
      VEsendData(data,6);
    }
    else if (checkRegID(0x0FFE)) {   //TTG     (Un16)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,TimeToGo / 60,(TimeToGo / 60) >> 8};   //minutes
      VEsendData(data,6);
    }
    else if (checkRegID(0xEEB6)) {   //Synchronization SoC state      (Un8)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,0xA0};   //SoC
      VEsendData(data,5);
    }
    //---------------------------------------------------------------------------
    //Monitor setting registers:
    else if (checkRegID(0x1000)) {   //Battery Capacity (Un16)
      if (recdata[0] == 0x08) //save
        shunt.BatAh = recdata[4] | (recdata[5] << 8);
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.BatAh,shunt.BatAh >> 8};   //110
      VEsendData(data,6);
    }
    else if (checkRegID(0x1001)) {   //Charged Voltage (Un16)
      if (recdata[0] == 0x08) //save
        shunt.MaxChargeVoltage = (int16_t)(recdata[4] | (recdata[5] << 8)) ;
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.MaxChargeVoltage,((int16_t)(shunt.MaxChargeVoltage)) >> 8};   //552
      VEsendData(data,6);
    }
    else if (checkRegID(0x1002)) {   //Tail Current  (Un16)
      if (recdata[0] == 0x08) //save
        shunt.Tailcurrent = (int16_t)(recdata[4] | (recdata[5] << 8));
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.Tailcurrent,((int16_t)(shunt.Tailcurrent)) >> 8};   //40 = 4%
      VEsendData(data,6);
    }
    else if (checkRegID(0x1003)) {   //Charged Detection Time  (Un16)
      if (recdata[0] == 0x08) //save
        shunt.ChargeDetectionTime = (int16_t)(recdata[4] | (recdata[5] << 8));
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.ChargeDetectionTime,shunt.ChargeDetectionTime >> 8};   //min
      VEsendData(data,6);
    }
    else if (checkRegID(0x1004)) {   //Charge Efficiency  (Un16)
      if (recdata[0] == 0x08) //save
        shunt.ChargeEfficencyFactor = (int16_t)(recdata[4] | (recdata[5] << 8));
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.ChargeEfficencyFactor,shunt.ChargeEfficencyFactor >> 8};   //99%
      VEsendData(data,6);
    }
    else if (checkRegID(0x1005)) {   //Peukert Coefficient  (Un16)
      if (recdata[0] == 0x08) //save
        shunt.Peukert = (recdata[4] | (recdata[5] << 8)) / 100.00;
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,round(shunt.Peukert * 100.0),(int16_t)(shunt.Peukert * 100) >> 8};   //1,02%
      VEsendData(data,6);
    }
    else if (checkRegID(0x1006)) {   //Current Threshold  (Un16)
      if (recdata[0] == 0x08) //save
        shunt.CurrentThreshold = (int16_t)(recdata[4] | (recdata[5] << 8)) / 100.00;
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,round(shunt.CurrentThreshold * 100),((int16_t)(shunt.CurrentThreshold * 100.0)) >> 8};   //0,1A
      VEsendData(data,6);
    }
    else if (checkRegID(0x1007)) {   //TTG Delta T  (Un16)
      if (recdata[0] == 0x08) //save
        shunt.TimeToGoDelta = (int16_t)(recdata[4] | (recdata[5] << 8));
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.TimeToGoDelta,((int16_t)(shunt.TimeToGoDelta)) >> 8};   //min
      VEsendData(data,6);
    }
    else if (checkRegID(0x1008)) {   //Discharge Floor (Relay Low Soc Set)  (Un16)
      if (recdata[0] == 0x08) { //save
        shunt.Entladeboden = (int16_t)(recdata[4] | (recdata[5] << 8)) / 10.0;
        shunt.EntladebodenClear = shunt.Entladeboden;   //deaktiviert das Relais
      }
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.Entladeboden * 10,((int16_t)(shunt.Entladeboden * 10.0)) >> 8};   //20.0%
      VEsendData(data,6);
    }
    else if (checkRegID(0x1009)) {   //Relay Low Soc Clear  (Un16)
      if (recdata[0] == 0x08) //save
        shunt.EntladebodenClear = (int16_t)(recdata[4] | (recdata[5] << 8)) / 10.0;
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.EntladebodenClear * 10.0,((int16_t)(shunt.EntladebodenClear * 10.0)) >> 8};   //30.0%
      VEsendData(data,6);
    }
    else if (checkRegID(0x1034)) {   //User Current Zero (read only) 
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,0x00,0x00};   //ADC count
      VEsendData(data,6);
    }
    //---------------------------------------------------------------------------
    //Alarm settings:
    else if  (checkRegID(0xEEFC)) {   //Alarm Buzzer   (Un8)
      if (recdata[0] == 0x08) //save
        shunt.AlarmBuzzer = recdata[4] & 0x01;
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.AlarmBuzzer};   //OFF
      VEsendData(data,5);
    }
    else if (checkRegID(0x0320)) {   //Alarm Low Voltage  (Un16)
      if (recdata[0] == 0x08) //save
        shunt.AlarmLowVoltage = (int16_t)(recdata[4] | (recdata[5] << 8));
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.AlarmLowVoltage,shunt.AlarmLowVoltage >> 8};   //0,1V
      VEsendData(data,6);
    }
    else if (checkRegID(0x0321)) {   //Alarm Low Voltage Clear (Un16)
      if (recdata[0] == 0x08) //save
        shunt.AlarmLowVoltageClear = (int16_t)(recdata[4] | (recdata[5] << 8));
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.AlarmLowVoltageClear,shunt.AlarmLowVoltageClear >> 8};   //0,1V
      VEsendData(data,6);
    }
    else if (checkRegID(0x0322)) {   //Alarm High Voltage  (Un16)
      if (recdata[0] == 0x08) //save
        shunt.AlarmHighVoltage = (int16_t)(recdata[4] | (recdata[5] << 8));
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.AlarmHighVoltage,shunt.AlarmHighVoltage >> 8};   //0,1V
      VEsendData(data,6);
    }
    else if (checkRegID(0x0323)) {   //Alarm High Voltage Clear (Un16)
      if (recdata[0] == 0x08) //save
        shunt.AlarmHighVoltageClear = (int16_t)(recdata[4] | (recdata[5] << 8));
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.AlarmHighVoltageClear,shunt.AlarmHighVoltageClear >> 8};   //0,1V
      VEsendData(data,6);
    }
    else if (checkRegID(0x0328)) {   ////Alarm Low SOC  (Un16)
      if (recdata[0] == 0x08) //save
        shunt.AlarmLowSoc = (int16_t)(recdata[4] | (recdata[5] << 8));
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.AlarmLowSoc,shunt.AlarmLowSoc >> 8};   //0.1%
      VEsendData(data,6);
    }
    else if (checkRegID(0x0329)) {   ////Alarm Low SOC Clear  (Un16)
      if (recdata[0] == 0x08) //save
        shunt.AlarmLowSocClear = (int16_t)(recdata[4] | (recdata[5] << 8));
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.AlarmLowSocClear,shunt.AlarmLowSocClear >> 8};   //0.1%
      VEsendData(data,6);
    }    
    //---------------------------------------------------------------------------
    //Relay settings:
    else if (checkRegID(0x034F)) {   //Relay Mode (Un8)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,0x00};  //0..2 (DFLT, CHRG, REM) 
      VEsendData(data,5);
    }
    else if (checkRegID(0x034D)) {   //Relay Invert(Un8)
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,0x00};  //0..1 (OFF, ON) 
      VEsendData(data,5);
    }
    else if (checkRegID(0x034E)) {   //Relay State/Control
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,0x00};   //0..1 (OPEN, CLSD)
      if (RelayState == true)
        data[4] = 0x01;
      VEsendData(data,5);
    }
    else if (checkRegID(0x100a)) {   //Relay Minimal Enable Time (Un16)
      if (recdata[0] == 0x08) //save
        shunt.RelayEnableTime = (int16_t)(recdata[4] | (recdata[5] << 8));
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.RelayEnableTime,shunt.RelayEnableTime >> 8};  //min
      VEsendData(data,6);
    }
    else if (checkRegID(0x100b)) {   //Relay Disable Time (Un16)
      if (recdata[0] == 0x08) //save
        shunt.RelayDisableTime = (int16_t)(recdata[4] | (recdata[5] << 8));
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.RelayDisableTime,shunt.RelayDisableTime >> 8};  //min
      VEsendData(data,6);
    }
    else if (checkRegID(0x0350)) {   //Relay Low Voltage  (Un16)
      if (recdata[0] == 0x08) //save
        shunt.RelayLowVoltage = (int16_t)(recdata[4] | (recdata[5] << 8));
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.RelayLowVoltage,shunt.RelayLowVoltage >> 8};  //min
      VEsendData(data,6);
    }
    else if (checkRegID(0x0351)) {   //Relay Low Voltage Clear (Un16)
      if (recdata[0] == 0x08) //save
        shunt.RelayLowVoltageClear = (int16_t)(recdata[4] | (recdata[5] << 8));
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.RelayLowVoltageClear,shunt.RelayLowVoltageClear >> 8};  //min
      VEsendData(data,6);
    }
    else if (checkRegID(0x0352)) {   //Relay High Voltage (Un16)
      if (recdata[0] == 0x08) //save
        shunt.RelayHighVoltage = (int16_t)(recdata[4] | (recdata[5] << 8));
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.RelayHighVoltage,shunt.RelayHighVoltage >> 8};  //min
      VEsendData(data,6);
    }
    else if (checkRegID(0x0353)) {   //Relay High Voltage Clear (Un16)
      if (recdata[0] == 0x08) //save
        shunt.RelayHighVoltageClear = (int16_t)(recdata[4] | (recdata[5] << 8));
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00,shunt.RelayHighVoltageClear,shunt.RelayHighVoltageClear >> 8};  //min
      VEsendData(data,6);
    }
    //---------------------------------------------------------------------------
    //Micellaneous:
    else if (checkRegID(0xEEF6)) {   //Setup Lock
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00, 0x00};
      VEsendData(data,5);
    }
    else if (checkRegID(0xEEFB)) {   //Shunt Amps
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00, 0xF4, 0x01}; //Default: 500A
      VEsendData(data,6);
    }
    else if (checkRegID(0xEEFA)) {   //Shunt Volts
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00, 0x32, 0x00}; //Default: 0.050
      VEsendData(data,6);
    }
    else if (checkRegID(0xEEF7)) {   //Temperature Unit
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00, 0x00}; //Default: 0
      VEsendData(data,5);
    }
    else if (checkRegID(0xEEF4)) {   //Temperature coefficient
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00, 0x00, 0x00}; //Default: 0
      VEsendData(data,6);
    }
    else if (checkRegID(0xEEF8)) {   //Aux Input
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00, 0xFF}; //Default: Keins
      VEsendData(data,5);
    }
    else if (checkRegID(0x0FFD)) {   //Start synchronized
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00, 0x00}; //Default: OFF
      VEsendData(data,5);
    }
    else if (checkRegID(0xEC41)) {   //Setting changed timesamp
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00, 0xFF, 0xFF, 0xFF, 0xFF}; //seconds
      VEsendData(data,8);
    }
    else if (checkRegID(0x0090)) {   //Bluetooth mode
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00, 0x00}; //OFF
      VEsendData(data,5);
    }
    else if (checkRegID(0xEEB8)) {   //DC Monitor
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00, 0x00, 0x00}; //OFF
      VEsendData(data,6);
    }
    //---------------------------------------------------------------------------
    //Commands:
    else if (checkRegID(0x1029)) {  //Zero Current (write only) Nullstromkalibrierung
      //how to do with INA226?
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00};//Done
      VEsendData(data,4);
    }
    else if (checkRegID(0x102c)) {  //Syncronisieren SOC 100%
      SoC = 100.0;
      shunt.NowAh = shunt.BatAh;
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00};//Done
      VEsendData(data,4);
      uint8_t So[] = {0x0A,0xFF,0x0F,0x00,round(SoC * 100),((int16_t)(SoC * 100)) >> 8};   //0.01%};   //0.01%
      VEsendData(So,6);
    }
    else if (checkRegID(0x0004)) {  //Restore Defaults (write only) 
      shunt.Peukert = 1.05;
      shunt.Tailcurrent = 40;
      shunt.CurrentThreshold = 0.08;
      shunt.ChargeDetectionTime = 3;
      shunt.ChargeEfficencyFactor = 99;
      shunt.Entladeboden = 20;
      shunt.MaxChargeVoltage = 564;
      shunt.TimeToGoDelta = 1;
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00};//Done
      VEsendData(data,4);
    }
    else if (checkRegID(0x1030)) {  //Clear History (write only) 
      shunt.AutomaticSyncs = 0;
      //shunt.ChargeCycles = 0;
      shunt.FullDischarges = 0;
      shunt.DeepestDischarge = 0.0;
      shunt.LastDischarge = 0.0;
      shunt.AvgDischarge = 0.0;
      shunt.TotalAhDrawn = 0.0;
      shunt.MinimumVoltage = BatVolt;
      shunt.MaximumVoltage = BatVolt;
      shunt.DischargedEnergy = 0.0;
      shunt.ChargedEnergy = 0.0;
      shunt.NumLowVoltageAlarms = 0;
      shunt.NumHighVoltageAlarms = 0;
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x00};//Done
      VEsendData(data,4);
    }
    //---------------------------------------------------------------------------
    //Unknown:
    else {
      uint8_t data[] = {recdata[0],recdata[1],recdata[2],0x01};
      VEsendData(data,4);
    }
    if (recdata[0] == 0x08)  //save
      save();  //writes structure data to FLASH MEMORY
    recdatacount = 0;    
  }
}

uint8_t VEDirectClient::ConvertToASCII(uint8_t hex) {
  if (hex < 10)
    return hex+'0';
  return (hex-10) + 'A';
}

void VEDirectClient::VEsendData(uint8_t message[], uint8_t size) {
  //:7<register id><flags><checksum>\n
  uint8_t checksum = 0x55;
  serialPort.write(0x3a); //:
  
  //WebSerial.print("TX:");
	
  for (int i = 0; i < size; i++) {
    checksum -= message[i];
    if (i != 0) {
      serialPort.write(ConvertToASCII(message[i] >> 4));
      serialPort.write(ConvertToASCII(message[i] & 0x0F));

      //WebSerial.print(String(message[i] >> 4, HEX));
      //WebSerial.print(String(message[i] & 0x0F, HEX));

    }
    else {
      serialPort.write(ConvertToASCII(message[i]));    //Response Code
     
      //WebSerial.print(String(message[i], HEX));
    }
   }
   serialPort.write(ConvertToASCII(checksum >> 4));   // CRC
   serialPort.write(ConvertToASCII(checksum & 0x0F));   // CRC
   
  // WebSerial.println(String(checksum, HEX));
   
   serialPort.write(0x0a);   // '\n' End of Message
}



void VEDirectClient::AddData (uint8_t d) {
  IntData[IntDataCount] = d;
  IntDataCount++;
}

void VEDirectClient::AddLabel (String FieldLabel, String FieldValue) {
  //<Newline><Field-Label><Tab><Field-Value>
  AddData(0x0D);
  AddData(0x0A);
  for (uint8_t i = 0; i < FieldLabel.length();i++)
    AddData(FieldLabel[i]);
  AddData(0x09);
  for (uint8_t i = 0; i < FieldValue.length();i++)
    AddData(FieldValue[i]);  
}

void VEDirectClient::AddCRC() {
  AddLabel("Checksum", "");
  IntData[IntDataCount] = 0;
  for (uint16_t i = 0; i < IntDataCount; i++) 
    IntData[IntDataCount] += IntData[i];
  IntData[IntDataCount] = 256 - IntData[IntDataCount];
  IntDataCount++;
}

void VEDirectClient::sendRegular() {
  IntDataCount = 0;
  AddLabel("PID", ProductID);
  AddLabel("V", String((int32_t)(BatVolt * 1000.0)));       //Main or channel 1 (battery) voltage 
  AddLabel("I", String((int32_t)(BatCurrent * 1000.0)));    //Main or channel 1 battery current
  AddLabel("P", String((int32_t)(BatVolt * BatCurrent * 10.0)));      //Instantaneous power
  AddLabel("CE", String((int32_t)((shunt.BatAh - shunt.NowAh) * 1000.0)));         //Consumed Amp Hours
  AddLabel("SOC", String((uint16_t)(SoC * 10.0)));
  AddLabel("TTG", String((int32_t)(TimeToGo / 60.0)));          //TimeToGo
  if (shunt.AlarmReason == 0)
    AddLabel("Alarm", "OFF");
  else AddLabel("Alarm", "ON");
  if (RelayState)
    AddLabel("Relay", "ON");
  else AddLabel("Relay", "OFF");  
  AddLabel("AR", String(shunt.AlarmReason));          //Alarm reason 
  AddLabel("FW", FirmwareVersion);
  AddLabel("H1", String((int32_t)(shunt.DeepestDischarge * 1000.0)));     //Depth of the deepest discharge mAh
  AddLabel("H2", String((int32_t)(shunt.LastDischarge * 1000.0)));     //Depth of the last discharge mAh
  AddLabel("H3", String((int32_t)((shunt.AvgDischarge) * 1000.0)));     // average discharge mAh (Average discharge: The cumulative Ah drawn divided by the total number of cycles)
  AddLabel("H4", String((int16_t)(shunt.ChargeCycles)));         //Number of charge cycles
  AddLabel("H5", String(shunt.FullDischarges));         //Number of full discharges
  AddLabel("H6", String((int32_t)(shunt.TotalAhDrawn * 1000.0)));      //Cumulative Amp Hours drawn 
  AddLabel("H7", String((int32_t)(shunt.MinimumVoltage * 1000.0)));    //Minimum main (battery) voltage (mV)
  AddLabel("H8", String((int32_t)(shunt.MaximumVoltage * 1000.0)));    //Maximum main (battery) voltage (mV)
  AddLabel("H9", String(shunt.TimeSinceLastFullCharge));                //Number of seconds since last full charge 
  AddLabel("H10", String(shunt.AutomaticSyncs));     //Number of automatic synchronizations
  AddLabel("H11", String(shunt.NumLowVoltageAlarms));     //Number of low main voltage alarms 
  AddLabel("H12", String(shunt.NumHighVoltageAlarms));     //Number of high main voltage alarms
  AddLabel("H17", String((int32_t)(shunt.DischargedEnergy * 100.0))); //Amount of discharged energy (BMV) (0.01kWh)
  AddLabel("H18", String((int32_t)(shunt.ChargedEnergy * 100.0)));  //Amount of charged energy (BMV) (0.01kWh)
  AddLabel("MON", "0");   //DC monitor mode 0 = Battery monitor (BMV)
  AddCRC();

  for (uint16_t i=0; i < IntDataCount; i++) {
    Serial.write(IntData[i]);
  }

  uint8_t data[] = {0x0A,0x4E, 0x03,0x00,0x00};   //0..1 (OPEN, CLSD)
      if (RelayState)
        data[4] = 1;
  VEsendData(data,5);
  
  //uint8_t Main[] = {0x0A,0x8D,0xED,0x00,((int16_t)(BatVolt * 100.0)) & 0xFF,((int16_t)(BatVolt * 100.0)) >> 8};   //Main Voltage
  //VEsendData(Main,6);
  //uint8_t Current[] = {0x0A,0x8F,0xED,0x00,((int16_t)(BatCurrent * 10.0)) & 0xFF,((int16_t)(BatCurrent * 10.0)) >> 8};   //Current
  //VEsendData(Current,6);
  //uint8_t Power[] = {0x0A,0x8E,0xED,0x00,0x01,0x00};   //Power
  //VEsendData(Power,6);
}
