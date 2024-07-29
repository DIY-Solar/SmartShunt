#include <Wire.h>
#include <INA226_WE.h>
#define I2C_ADDRESS 0x40

INA226_WE ina226 = INA226_WE(I2C_ADDRESS);


void INA_init() {
  //Wire.setClock(400000L);   //400 kHz on my ESP boards - doesn't work with INA226!
  Wire.begin();
  
  if(!ina226.init()){
    #if defined(DEBUG)
    Debug.println("INA226 not connected!");
    #endif
  }

  /* Set Number of measurements for shunt and bus voltage which shall be averaged
  * Mode *     * Number of samples *
  AVERAGE_1            1 (default)
  AVERAGE_4            4
  AVERAGE_16          16
  AVERAGE_64          64
  AVERAGE_128        128
  AVERAGE_256        256
  AVERAGE_512        512
  AVERAGE_1024      1024
  */
  ina226.setAverage(AVERAGE_256); 
  
  /* Set conversion time in microseconds
     One set of shunt and bus voltage conversion will take: 
     number of samples to be averaged x conversion time x 2
     
     * Mode *         * conversion time *
     CONV_TIME_140          140 µs
     CONV_TIME_204          204 µs
     CONV_TIME_332          332 µs
     CONV_TIME_588          588 µs
     CONV_TIME_1100         1.1 ms (default)
     CONV_TIME_2116       2.116 ms
     CONV_TIME_4156       4.156 ms
     CONV_TIME_8244       8.244 ms  
  */
  //ina226.setConversionTime(CONV_TIME_2116);

  /* Set measure mode
  POWER_DOWN - INA226 switched off
  TRIGGERED  - measurement on demand
  CONTINUOUS  - continuous measurements (default)
  */
  //ina226.setMeasureMode(CONTINUOUS); // choose mode and uncomment for change of default

  /* Set Resistor and Current Range
     resistor is 5.0 mOhm
     current range is up to 10.0 A
     default was 100 mOhm and about 1.3 A   --> 0.005,10.0
  */
  ina226.setResistorRange(0.00075,100.0); // choose resistor 5 mOhm and gain range up to 10 A

    /* If the current values delivered by the INA226 differ by a constant factor
     from values obtained with calibrated equipment you can define a correction factor.
     Correction factor = current delivered from calibrated equipment / current delivered by INA226
  */
  ina226.setCorrectionFactor(1.00); //1.05

  //if you comment this line the first data might be zero
  ina226.waitUntilConversionCompleted(); 
}

/*
//------------------------------------------------------------------------------
void readData() {
  float shuntVoltage_mV = 0.0;
  float loadVoltage_V = 0.0;
  float busVoltage_V = 0.0;
  float current_mA = 0.0;
  float power_mW = 0.0; 
  
  ina226.readAndClearFlags();
  shuntVoltage_mV = ina226.getShuntVoltage_mV();
  busVoltage_V = ina226.getBusVoltage_V() * 2;
  current_mA = ina226.getCurrent_mA();
  power_mW = ina226.getBusPower();
  loadVoltage_V  = busVoltage_V + (shuntVoltage_mV/1000);

  ADSA0 = shuntVoltage_mV;
  
  Debug.print("Shunt Voltage [mV]: "); Debug.println(shuntVoltage_mV);
  Debug.print("Bus Voltage [V]: "); Debug.println(busVoltage_V);
  Debug.print("Load Voltage [V]: "); Debug.println(loadVoltage_V);
  Debug.print("Current[mA]: "); Debug.println(current_mA);
  Debug.print("Bus Power [mW]: "); Debug.println(power_mW);
  if(!ina226.overflow){
    Debug.println("Values OK - no overflow");
  }
  else{
    Debug.println("Overflow! Choose higher current range");
  }
  Debug.println();
}
*/
