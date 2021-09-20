#include <math.h>
#include <avr/pgmspace.h>
#include <Wire.h>   //Include the Wire library to talk I2C

#include "AnalyserValve.h"
#include "CommandParser.h"

/************************************************************
   GLOBAL VARIABLES
 ************************************************************/
int targetValues[ARRAY_LENGTH];   //Array to hold target values (array lenth must be less than 32 ints or 16 unsigned ints owing to I2C limit)
int measuredValues[ARRAY_LENGTH]; //Array to hold measured values (array lenth must be less than 32 ints or 16 unsigned ints owing to I2C limit)

int slaveCounter = 0;

int duty_cycle = 0;      //Duty cycle of buck converter
boolean hardware;         //Will be set to 1 if hardware ID pin is high (MASTER), else 0 (SLAVE).

CommandParser parser(modeCommand, getCommand, setCommand, commandError);

enum {
  ERR_INVALID_MODE,
  ERR_INVALID_SET,
  ERR_INVALID_GET,
  ERR_HEATER_RANGE,
  ERR_GRID_RANGE,
  ERR_HT_RANGE,
  ERR_HT_TIMEOUT,
  ERR_UNSAFE
};

const char *errorMessages[] = {
  "Invalid mode command",
  "Invalid set command",
  "Invalid get command",
  "Heater voltage out of range",
  "Grid voltage out of range",
  "HT voltage out of range",
  "Timeout setting HT voltage",
  "Unsafe to test"
};

/************************************************************
   SETUP
 ************************************************************/
void setup() {
  Serial.begin(115200); //Setup serial interface

  pinMode(HARDWARE_ID_PIN, INPUT);
  //I2C SDA is on Arduino Nano pin A4 as standard
  //I2C SCL is on Arduino Nano pin A5 as standard. These pins need no further setup.
  //By default, analog input pins also need no setup

  analogReference(EXTERNAL);                //Use external voltage reference for ADC
  TCCR0B = (TCCR0B & 0b11111000) | 0x01;    //Configure Timer0 for internal clock, no prescaling (bottom 3 bits of TCCR0B)
  //Makes Arduino run 6.3 times faster than normal. NB: This affects Arduino delay() function!

  analogWrite(PWM_PIN, 0);                  //Make sure PWM output is zero on startup
  pinMode(LED_BUILTIN, OUTPUT);             //Arduino built-in LED for debugging

  hardware = digitalRead(HARDWARE_ID_PIN); //Identify if this is MASTER (1) or SLAVE (0) Arduino

  if (hardware == MASTER) {
    pinMode(CHARGE1_PIN, OUTPUT);
    pinMode(DISCHARGE1_PIN, OUTPUT);
    pinMode(FIRE1_PIN, OUTPUT);
    pinMode(CHARGE2_PIN, OUTPUT);
    pinMode(DISCHARGE2_PIN, OUTPUT);
    pinMode(FIRE2_PIN, OUTPUT);
    digitalWrite(FIRE1_PIN, LOW);
    digitalWrite(FIRE2_PIN, LOW);
    digitalWrite(CHARGE1_PIN, LOW);
    digitalWrite(CHARGE2_PIN, LOW);
    digitalWrite(DISCHARGE1_PIN, HIGH); //make sure capacitor banks are discharged ready for first sweep
    digitalWrite(DISCHARGE2_PIN, HIGH);
    Wire.begin(MASTER_ADDR);            //Register I2C address
    Wire.onReceive(masterReceiveData);  //Interrupt when I2C data is being received
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else { //else this is SLAVE hardware
    pinMode(LV_DETECT_PIN, INPUT);
    analogWrite(PWM_PIN, 0);            //Set heater to 0V at start-up
    Wire.begin(SLAVE_ADDR);             //Register I2C address
    Wire.onReceive(slaveReceiveData);   //Interrupt when I2C data is being received
    Wire.onRequest(slaveAnswerRequest); //Interrupt when master demands a response
    digitalWrite(LED_BUILTIN, LOW);
  }
}

/************************************************************
   MAIN LOOP
 ************************************************************/
void loop() {

  while (Serial.available() > 0) {
    parser.parseInput(Serial.read());
  }

  if (++slaveCounter > 1000) { // Periodically poll the Slave to get heater values
    requestFromSlave();
    slaveCounter = 0;
  }
} //End of main program loop

// USB command interface functions

/************************************************************
   Callback for Mode commands
 ************************************************************/
void modeCommand(int index) {
  int success = 1;

  switch (index) {
    case 0: // Placeholder for a hard reset
    case 1: // Safe mode
      dischargeHighVoltages();
      Serial.print("OK: Mode(");
      Serial.print(index);
      Serial.println(')');
      break;
    case 2: // Run test
      // success = runTest();
      if (success > 0) {
        Serial.print("OK: ");
        for (int i = 0; i < 10; i++) {
          Serial.print(measuredValues[i]);
          if (i < 9) {
            Serial.print(", ");
          } else {
            Serial.println("");
          }
        }
      }
      break;
    default:
      success = -ERR_INVALID_MODE;
      break;
  }

  if (success < 0) {
    Serial.print("ERR: ");
    Serial.println(errorMessages[-success]);
  }
}

/************************************************************
   Callback for Set commands
 ************************************************************/
void setCommand(int index, int intParam) {
  int success = 1;

  switch (index) {
    case VH: // Heater voltage
      if (intParam < 0 || intParam > 1023) {
        success = -ERR_HEATER_RANGE;
      }
      break;
    case VG1: // Grid 1 voltage
    case VG2: // Grid 2 voltage
      if (intParam < 0 || intParam > 4095) {
        success = -ERR_GRID_RANGE;
      }
      break;
    case HV1: // Anode 1 voltage
    case HV2: // Anode 2 voltage
      if (intParam < 0 || intParam > 4095) {
        success = -ERR_HT_RANGE;
      }
      break;
    default:
      success = -ERR_INVALID_SET;
      break;
  }

  if (success > 0) {
    targetValues[index] = intParam;

    if (index < 2) {
      sendToSlave();
    }

    Serial.print("OK: Set(");
    Serial.print(index);
    Serial.print(") = ");
    Serial.println(intParam);
  } else {
    Serial.print("ERR: ");
    Serial.println(errorMessages[-success]);
  }
}

/************************************************************
   Callback for Get commands
 ************************************************************/
void getCommand(int index) {
  if (index >= 0 && index <= 9) {
    Serial.print("OK: Get(");
    Serial.print(index);
    Serial.print(") = ");
    Serial.println(measuredValues[index]);
  } else {
    Serial.print("ERR: ");
    Serial.println(errorMessages[ERR_INVALID_GET]);
  }
}

/************************************************************
   Callback for syntax erros
 ************************************************************/
void commandError(const char *command) {
  Serial.print("ERR: Unrecognised command - ");
  Serial.println(command);
}

// Master/slave I2C interface functions

/****************************************************************************
  Send target heater value to slave
****************************************************************************/
void sendToSlave() {
  byte byte1;
  byte byte2;
  Wire.beginTransmission(SLAVE_ADDR);
  byte1 = highByte(targetValues[VH]); //Arduino function breaks word into bytes
  byte2 = lowByte(targetValues[VH]);
  Wire.write(byte1);
  Wire.write(byte2);
  Wire.endTransmission();
}

/****************************************************************************
  Upon receiving data, populate the measured values array
****************************************************************************/
void masterReceiveData(int howMany) { //called by ISR when I2C data arrives
  byte byte1;
  byte byte2;
  for (int i = 0; i < (howMany >> 1); i++) { //receive bytes in pairs
    byte1 = Wire.read();
    byte2 = Wire.read();
    measuredValues[i] = word(byte1, byte2); //Arduino function concatenates bytes into word
  }
  while (Wire.available() > 0) { //Ignore any garbage
    Wire.read();
  }
}

/****************************************************************************
  Request measured heater values from slave
****************************************************************************/
void requestFromSlave() {
  byte byte1;
  byte byte2;
  Wire.requestFrom(SLAVE_ADDR, 4); //Request four bytes
  for (int i = 0; i < 2; i++) {
    byte1 = Wire.read();
    byte2 = Wire.read();
    measuredValues[i] = word(byte1, byte2); //Arduino function concatenates bytes into word
  }
}

/****************************************************************************
  Upon receiving data, populate the target values array
****************************************************************************/
void slaveReceiveData(int howMany) {      //called by ISR when I2C data arrives
  for (int i = 0; i < (howMany >> 1); i++) { //receive bytes in pairs
    byte byte1 = Wire.read();
    byte byte2 = Wire.read();
    targetValues[i] = word(byte1, byte2); //Arduino function concatenates bytes into word
  }
  while (Wire.available() > 0) { //Discard any garbage
    Wire.read();
  }
}

/****************************************************************************
  Upon request, send measured heater values to the master
****************************************************************************/
void slaveAnswerRequest() {
  byte byte1;
  byte byte2;
  for (int i = 0; i < 2; i++) { //Send only the first two words in array
    byte1 = highByte(measuredValues[i]); //Arduino function breaks word into bytes
    byte2 = lowByte(measuredValues[i]);
    Wire.write(byte1);
    Wire.write(byte2);
  }
}

// Testing functions

/************************************************************
   Runs a test
 ************************************************************/
int runTest() {
  int status;
  setGridVolts();
  status = chargeHighVoltages();
  if (status > 0) {
    doMeasurement();
    requestFromSlave();
  }

  return status;
}

/************************************************************
   Controls the heater buck regulator
 ************************************************************/
void setHeaterVolts() { //Manages the heater buck-converter
  int Vh_adc = analogRead(VH_PIN);
  int Ih_adc = analogRead(IH_PIN);
  Vh_adc = Vh_adc - (Ih_adc / 8);   //Divide Ih_adc value by 8 and subtract from Vh_adc value to get corrected voltage across heater
  measuredValues[VH] = Vh_adc;        //Update the array with new heater voltage
  measuredValues[IH] = Ih_adc;        //Update the array with new heater current
  if (duty_cycle > 0) {               //Duty cycle is always trying to decrement as a fail-safe measure
    duty_cycle --;                   //but don't let it drop below zero (wrap around)
  }

  if (Ih_adc < 110) {                               //If heater current is less than 2 amps it is safe to proceed.
    measuredValues[VH] = Vh_adc;                  //Update the array with new heater voltage
    measuredValues[IH] = Ih_adc;                  //Update the array with new heater current
    if (measuredValues[VH] < targetValues[VH]) {  //If heater voltage is too low, increment duty cycle
      if (duty_cycle < 254) {                   //But don't let duty cycle exceed 255 (wrap around)
        duty_cycle += 2;
      }
    }
  }

  if (!digitalRead(LV_DETECT_PIN)) {
    duty_cycle = 0;                                //Disable buck converter if heater power supply is off
  }
  analogWrite(PWM_PIN, duty_cycle);                 //Update buck converter with new duty cycle

  /*For debugging*/
  if (measuredValues[VH] == targetValues[VH]) { //If heater voltage is just right
    digitalWrite(LED_BUILTIN, HIGH); //light LED
  }
  else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

/****************************************************************************
  Updates the bias DACs with target values
****************************************************************************/
void setGridVolts() {
  if (measuredValues[VG1] != targetValues[VG1]) {   //If target grid voltage has changed since last time,
    Wire.beginTransmission(DAC1_ADDR);
    Wire.write(64);                               //Command to update the DAC
    Wire.write(targetValues[VG1] >> 4);           //8 most significant bits
    Wire.write((targetValues[VG1] & 15) << 4);    //4 least significant bits
    if (Wire.endTransmission() == 0) {            //If I2C tramission was a success
      measuredValues[VG1] = targetValues[VG1];  //Store the new grid voltage
    }
  }

  if (measuredValues[VG2] != targetValues[VG2]) { //If target grid voltage has changed since last time,
    Wire.beginTransmission(DAC2_ADDR);
    Wire.write(64);                             //Command to update the DAC
    Wire.write(targetValues[VG2] >> 4);         //8 most significant bits
    Wire.write((targetValues[VG2] & 15) << 4);  //4 least significant bits
    if (Wire.endTransmission() == 0) {          //If I2C tramission was a success
      measuredValues[VG2] = targetValues[VG2];//Store the new grid voltage
    }
  }
}

/****************************************************************************
  Charges up the high-voltage capacitor banks to the target values
****************************************************************************/
int chargeHighVoltages() { //Manages the HV supply
  digitalWrite(FIRE1_PIN, LOW);                       //Turn off MOSFETs (fail-safe measure)
  digitalWrite(FIRE2_PIN, LOW);
  digitalWrite(CHARGE1_PIN, LOW);
  digitalWrite(CHARGE2_PIN, LOW);
  digitalWrite(DISCHARGE1_PIN, LOW);
  digitalWrite(DISCHARGE2_PIN, LOW);
  measuredValues[HV1] = analogRead(VA1_PIN);         //Measure the high voltage and store the value
  measuredValues[HV2] = analogRead(VA2_PIN);         //Measure the high voltage and store the value
  //While either storage cap is not charged to the correct voltage, alternately charge each cap
  //NB: Both cannot be charged simultaneously or one may hold down the supply to the other.
  while ((measuredValues[HV1] != targetValues[HV1]) || (measuredValues[HV2] != targetValues[HV2])) {
    int timeout = 0;
    while (measuredValues[HV1] < (targetValues[HV1] - 0)) { //If voltage is too low, charge capacitor
      digitalWrite(CHARGE1_PIN, HIGH);
      measuredValues[HV1] = analogRead(VA1_PIN);    //Keep checking the voltage
      if (timeout++ > HT_TIMEOUT) {
        return -ERR_HT_TIMEOUT;
      }
    }
    digitalWrite(CHARGE1_PIN, LOW); //Done, isolate this storage capacitance. It will begin to discharge slowly.

    timeout = 0;
    measuredValues[HV2] = analogRead(VA2_PIN);          //Measure the high voltage and store the value
    while (measuredValues[HV2] < (targetValues[HV2] - 0)) { //If voltage is too low, charge capacitor
      digitalWrite(CHARGE2_PIN, HIGH);
      measuredValues[HV2] = analogRead(VA2_PIN);    //Keep checking the voltage
    }
    digitalWrite(CHARGE2_PIN, LOW); //Done, isolate this storage capacitance. It will begin to discharge slowly.
    measuredValues[HV1] = analogRead(VA1_PIN);//Check first capacitor bank again
    if (timeout++ > HT_TIMEOUT) {
      return -ERR_HT_TIMEOUT;
    }
  }

  return 1;
}

/****************************************************************************
  Takes a measurement and puts the results in the measuredValues[] array
****************************************************************************/
void doMeasurement(void) {
  int volts1;
  int amps1_hi;
  int amps1_lo;
  int volts2;
  int amps2_hi;
  int amps2_lo;
  digitalWrite(FIRE1_PIN, HIGH);    //Apply high voltage to the DUT
  digitalWrite(FIRE2_PIN, HIGH);

  delayMicroseconds(300);

  //volts1 = analogRead(VA1_PIN);  //Read ADC twice to introduce some delay and avoid glitches
  volts1 = analogRead(VA1_PIN);
  amps2_hi = analogRead(IA2_HI_PIN);
  amps2_hi = analogRead(IA2_HI_PIN);
  amps2_lo = analogRead(IA2_LO_PIN);
  amps2_lo = analogRead(IA2_LO_PIN);
  amps1_lo = analogRead(IA1_LO_PIN);
  amps1_lo = analogRead(IA1_LO_PIN);
  amps1_hi = analogRead(IA1_HI_PIN);
  amps1_hi = analogRead(IA1_HI_PIN);
  volts2 = analogRead(VA2_PIN);
  volts2 = analogRead(VA2_PIN);

  digitalWrite(FIRE1_PIN, LOW);      //Remove high voltage from the DUT
  digitalWrite(FIRE2_PIN, LOW);
  measuredValues[HV1] = volts1;
  measuredValues[IA_HI_1] = amps1_hi;
  measuredValues[IA_LO_1] = amps1_lo;
  measuredValues[HV2] = volts2;
  measuredValues[IA_HI_2] = amps2_hi;
  measuredValues[IA_LO_2] = amps2_lo;
}

/****************************************************************************
  Discharges the capacitor banks
****************************************************************************/
void dischargeHighVoltages(void) {
  digitalWrite(LED_BUILTIN, HIGH); //for debugging
  digitalWrite(FIRE1_PIN, LOW);
  digitalWrite(FIRE2_PIN, LOW);
  digitalWrite(CHARGE1_PIN, LOW);
  digitalWrite(CHARGE2_PIN, LOW);
  digitalWrite(DISCHARGE1_PIN, HIGH);
  digitalWrite(DISCHARGE2_PIN, HIGH);
  while (analogRead(IA1_HI_PIN)) { //wait until no discharge current is detected
  }
  while (analogRead(IA2_HI_PIN)) { //wait until no discharge current is detected
  }
  digitalWrite(LED_BUILTIN, LOW); //for debugging
}
