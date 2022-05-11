#include <math.h>
#include <avr/pgmspace.h>
#include <Wire.h>   //Include the Wire library to talk I2C
//#include <Adafruit_MCP4725.h> Could use the library for the MCP4725s but it's trivial with Wire

#include "AnalyserValve.h"
#include "CommandParser.h"

/************************************************************
   GLOBAL VARIABLES
 ************************************************************/
int targetValues[ARRAY_LENGTH];   //Array to hold target values (array lenth must be less than 32 ints or 16 unsigned ints owing to I2C limit)
int measuredValues[ARRAY_LENGTH]; //Array to hold measured values (array lenth must be less than 32 ints or 16 unsigned ints owing to I2C limit)

int slaveCounter = 0;

double vHeater = 0; // Persistent value used for rolling averaging
double iHeater = 0; // Persistent value used for rolling averaging

#define AVG_FACTOR 0.99

int duty_cycle = 0;      //Duty cycle of buck converter
boolean hardware;         //Will be set to 1 if hardware ID pin is high (MASTER), else 0 (SLAVE).

//Adafruit_MCP4725 dac1;
//Adafruit_MCP4725 dac2;

CommandParser parser(infoCommand, modeCommand, getCommand, setCommand, commandError);

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
  //dac1.begin(DAC1_ADDR);
  //dac2.begin(DAC2_ADDR);

  pinMode(HARDWARE_ID_PIN, INPUT);
  //I2C SDA is on Arduino Nano pin A4 as standard
  //I2C SCL is on Arduino Nano pin A5 as standard. These pins need no further setup.
  //By default, analog input pins also need no setup

  analogReference(EXTERNAL);                //Use external voltage reference for ADC
  TCCR0B = (TCCR0B & 0b11111000) | 0x01;    //Configure Timer0 for internal clock, no prescaling (bottom 3 bits of TCCR0B)
  //Makes Arduino run 6.3 times faster than normal. NB: This affects Arduino delay() function!

  analogWrite(PWM_PIN, 0);                  //Make sure PWM output is zero on startup
  pinMode(LED_BUILTIN, OUTPUT);             //Arduino built-in LED for debugging

  hardware = digitalRead(HARDWARE_ID_PIN) == HIGH; //Identify if this is MASTER (1) or SLAVE (0) Arduino

  for (int i = 0; i < ARRAY_LENGTH; i++) {
    targetValues[i] = 0;
  }

  if (hardware) { // MASTER mode
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
    //digitalWrite(DISCHARGE1_PIN, HIGH); //make sure capacitor banks are discharged ready for first sweep
    //digitalWrite(DISCHARGE2_PIN, HIGH);
    dischargeHighVoltages(1);
    dischargeHighVoltages(2);
    Wire.begin(MASTER_ADDR);            //Register I2C address
    Wire.onReceive(masterReceiveData);  //Interrupt when I2C data is being received
    digitalWrite(LED_BUILTIN, HIGH);
    setGridVolts();
  }
  else { // else this is SLAVE hardware
    pinMode(PWM_PIN, OUTPUT);
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
  if (hardware == MASTER) {
    while (Serial.available() > 0) {
      parser.parseInput(Serial.read());
    }

    if (++slaveCounter > 10000) { // Periodically poll the Slave to get heater values
      requestFromSlave();
      slaveCounter = 0;
    }
  } else { // SLAVE mode
    setHeaterVolts();
  }
} //End of main program loop

// USB command interface functions

/************************************************************
   Callback for Info commands
 ************************************************************/
void infoCommand(int index) {
  int success = 1;

  switch (index) {
    case 0: // H/W Version info
      Serial.print("OK: Info(");
      Serial.print(index);
      Serial.println(') = Rev 1 (Nano)');
      break;
    case 1: // S/W Version info
      Serial.print("OK: Info(");
      Serial.print(index);
      Serial.println(') = 1.0.0');
      break;
    default:
      success = -ERR_INVALID_MODE;
      break;
  }

  if (success < 0) {
    Serial.print("ERR: ");
    Serial.print(errorMessages[-success]);
    Serial.print(" - ");
    printValues();
  }
}

/************************************************************
   Callback for Mode commands
 ************************************************************/
void modeCommand(int index) {
  int success = 1;

  switch (index) {
    case 0: // Safe mode
      dischargeHighVoltages(1);
      dischargeHighVoltages(2);
      for (int i = 0; i < ARRAY_LENGTH; i++) {
        targetValues[i] = 0;
      }
      sendToSlave();
      setGridVolts();
      Serial.print("OK: Mode(");
      Serial.print(index);
      Serial.println(')');
      break;
    case 1: // Discharge high voltages
      dischargeHighVoltages(1);
      dischargeHighVoltages(2);
      Serial.print("OK: Mode(");
      Serial.print(index);
      Serial.print(") ");
      printValues();
      break;
    case 2: // Run test
      success = runTest();
      if (success > 0) {
        Serial.print("OK: Mode(");
        Serial.print(index);
        Serial.print(") ");
        printValues();
      }
      break;
    case 3: // Prepare HV (for debugging)
      success = chargeHighVoltages();
      if (success > 0) {
        Serial.print("OK: Mode(");
        Serial.print(index);
        Serial.print(") ");
        printValues();
      }
      break;
    case 4: // Prepare HV and apply (for debugging)
      success = chargeHighVoltages();
      if (success > 0) {
        digitalWrite(FIRE1_PIN, HIGH);    //Apply high voltage to the DUT
        digitalWrite(FIRE2_PIN, HIGH);
        measureValues();
        Serial.print("OK: Mode(");
        Serial.print(index);
        Serial.print(") ");
        printValues();
      }
      break;
    case 5: // Return all measured values
      measureValues();
      Serial.print("OK: Mode(");
      Serial.print(index);
      Serial.print(") ");
      printValues();
      break;
    default:
      success = -ERR_INVALID_MODE;
      break;
  }

  if (success < 0) {
    Serial.print("ERR: ");
    Serial.print(errorMessages[-success]);
    Serial.print(" - ");
    printValues();
  }
}

void printValues() {
  for (int i = 0; i < 10; i++) {
    Serial.print(measuredValues[i]);
    if (i < 9) {
      Serial.print(", ");
    } else {
      Serial.println("");
    }
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
      if (intParam < 0 || intParam > 1023) {
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
    } else if (index == VG1 || index == VG2) {
      setGridVolts();
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
  measureValues();
  
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
  
  if (duty_cycle > 0) {               //Duty cycle is always trying to decrement as a fail-safe measure
    duty_cycle --;                   //but don't let it drop below zero (wrap around)
  }

  if (Ih_adc < 110) {                               //If heater current is less than 2 amps it is safe to proceed.
    if (Vh_adc < targetValues[VH]) {  //If heater voltage is too low, increment duty cycle
      if (duty_cycle < 254) {                   //But don't let duty cycle exceed 255 (wrap around)
        duty_cycle += 2;
      }
    }
  }

  if (!digitalRead(LV_DETECT_PIN)) {
    duty_cycle = 0;                                //Disable buck converter if heater power supply is off
  }

  analogWrite(PWM_PIN, duty_cycle);                 //Update buck converter with new duty cycle

  vHeater = (vHeater * AVG_FACTOR + ((double) Vh_adc));
  iHeater = (iHeater * AVG_FACTOR + ((double) Ih_adc));

  //measuredValues[VH] = Vh_adc;                  //Update the array with new heater voltage
  //measuredValues[IH] = Ih_adc;                  //Update the array with new heater current
  measuredValues[VH] = vHeater * (1.0 - AVG_FACTOR);                  //Update the array with new *average* heater voltage
  measuredValues[IH] = iHeater * (1.0 - AVG_FACTOR);                  //Update the array with new *average* heater current
}

/****************************************************************************
  Updates the bias DACs with target values
****************************************************************************/
void setGridVolts() {
  byte buf[3];

  Wire.beginTransmission(DAC1_ADDR);          // This DAC programmed in Fast mode
  buf[0] = targetValues[VG1] >> 8;
  buf[1] = targetValues[VG1] & 255;
  Wire.write(buf, 2);
  if (Wire.endTransmission() == 0) {          //If I2C tramission was a success
    measuredValues[VG1] = targetValues[VG1];  //Store the new grid voltage
  }

  Wire.beginTransmission(DAC2_ADDR);
  buf[0] = targetValues[VG2] >> 8;
  buf[1] = targetValues[VG2] & 255;
  Wire.write(buf, 2);
  if (Wire.endTransmission() == 0) {          //If I2C tramission was a success
    measuredValues[VG2] = targetValues[VG2];  //Store the new grid voltage
  }
}

/* void setGridVolts() {
  dac1.setVoltage(targetValues[VG1], false);
  dac2.setVoltage(targetValues[VG2], false);
  } */

/****************************************************************************
  Charges up the high-voltage capacitor banks to the target values
****************************************************************************/
int chargeHighVoltages() { //Manages the HV supply
  digitalWrite(FIRE1_PIN, LOW);                       //Turn off MOSFETs (fail-safe measure)
  digitalWrite(FIRE2_PIN, LOW);
#ifdef WIZARD_MODE
  digitalWrite(CHARGE1_PIN, LOW); // If we're in Wizard mode we treat the charge pins as digital output (i.e. on or off)
  digitalWrite(CHARGE2_PIN, LOW);
#else
  analogWrite(CHARGE1_PIN, 0); // If we're in PWM mode then set the duty cycle for the charge pins to 0
  analogWrite(CHARGE2_PIN, 0);
#endif // WIZARD_MODE
  digitalWrite(DISCHARGE1_PIN, LOW);
  digitalWrite(DISCHARGE2_PIN, LOW);

  measuredValues[HV1] = analogRead(VA1_PIN);         //Measure the high voltage and store the value
  measuredValues[HV2] = analogRead(VA2_PIN);         //Measure the high voltage and store the value
  measuredValues[HV1] = analogRead(VA1_PIN);         //Measure the high voltage and store the value
  measuredValues[HV2] = analogRead(VA2_PIN);         //Measure the high voltage and store the value
  
  if (measuredValues[HV1] > (targetValues[HV1] + OVERVOLTAGE)) {     //Check if our start condition is overvoltage
    dischargeHighVoltages(1);                        //If so, discharge it manually as it may take a while to settle through leakage
    measuredValues[HV1] = analogRead(VA1_PIN);       //Measure the high voltage and store the value
  }
  if (measuredValues[HV2] > (targetValues[HV2] + OVERVOLTAGE)) {     //Check if our start condition is overvoltage
    dischargeHighVoltages(2);                        //If so, discharge it manually as it may take a while to settle through leakage
    measuredValues[HV2] = analogRead(VA2_PIN);       //Measure the high voltage and store the value
  }

  //While either storage cap is not charged to the correct voltage, alternately charge each cap
  //NB: Both cannot be charged simultaneously or one may hold down the supply to the other.
  int timeout1 = 0;

  //while ((measuredValues[HV1] != targetValues[HV1]) || (measuredValues[HV2] != targetValues[HV2])) {
  while (!checkAnodeVoltage(measuredValues[HV1], targetValues[HV1]) || !checkAnodeVoltage(measuredValues[HV2], targetValues[HV2])) {    
    int timeout2 = 0;
    while (measuredValues[HV1] < (targetValues[HV1] + OVERVOLTAGE)) { //If voltage is too low, charge capacitor
#ifdef WIZARD_MODE
      digitalWrite(CHARGE1_PIN, HIGH); // If we're in Wizard mode we treat the charge pins as digital output (i.e. on or off)
#else
      int duty = setDuty(measuredValues[HV1], targetValues[HV1]);
      analogWrite(CHARGE1_PIN, duty); // If we're in PWM mode then set the duty cycle according to the (inverse) voltage gap
#endif // WIZARD_MODE
      measuredValues[HV1] = analogRead(VA1_PIN);    //Keep checking the voltage
      if (timeout2++ > HT_TIMEOUT) {
        chargeOff();
        return -ERR_HT_TIMEOUT;
      }
    }

    chargeOff();
                                                                                                                 
    timeout2 = 0;
    measuredValues[HV2] = analogRead(VA2_PIN);          //Measure the high voltage and store the value
    while (measuredValues[HV2] < (targetValues[HV2] + OVERVOLTAGE)) { //If voltage is too low, charge capacitor
#ifdef WIZARD_MODE
      digitalWrite(CHARGE2_PIN, HIGH); // If we're in Wizard mode we treat the charge pins as digital output (i.e. on or off)
#else
      int duty = setDuty(measuredValues[HV2], targetValues[HV2]);
      analogWrite(CHARGE2_PIN, duty); // If we're in PWM mode then set the duty cycle according to the (inverse) voltage gap
#endif // WIZARD_MODE
      measuredValues[HV2] = analogRead(VA2_PIN);    //Keep checking the voltage
      if (timeout2++ > HT_TIMEOUT) {
        chargeOff();
        return -ERR_HT_TIMEOUT;
      }
    }
    chargeOff();
    
    measuredValues[HV1] = analogRead(VA1_PIN);//Check first capacitor bank again
    
    if (timeout1++ > HT_TIMEOUT) {
      chargeOff();
      return -ERR_HT_TIMEOUT;
    }
  }

  return 1;
}

void chargeOff() {
#ifdef WIZARD_MODE
    digitalWrite(CHARGE1_PIN, LOW);
    digitalWrite(CHARGE2_PIN, LOW);
#else
    analogWrite(CHARGE1_PIN, 0);
    analogWrite(CHARGE2_PIN, 0);
#endif
}

bool checkAnodeVoltage(int measured, int target) {
  // At lower voltages, we could narrow the tolerance band to improve accuracy
  int gap = measured - target;
  return gap >= 0 && gap < (2 * OVERVOLTAGE); // This allows us to be on the money or a little over
}

int setDuty(int measured, int target) {
  int measuredGap = 1023 - measured; // Rather than gap to target, we take the gap to absolute max (as this is the voltage across the charging resistor)
  if (measuredGap >= THRESHOLD) { // If the gap is over the threshold the the duty cycle is minimum to be nice to the resistors 
    return HV_DUTY_MIN;
  }
  double gap = ((double) (THRESHOLD - measuredGap)) / THRESHOLD; // Find out how far below the threshold we are and increase the duty cycle in proportion
  int duty = (int) (gap * CHARGING_SPEED) + HV_DUTY_MIN;
  if (duty < HV_DUTY_MIN) {
    duty = HV_DUTY_MIN;
  }
  if (duty > 255) {
    duty = 255;
  }

  return duty;
}

/****************************************************************************
  Takes a measurement and puts the results in the measuredValues[] array
****************************************************************************/
void doMeasurement(void) {
  digitalWrite(FIRE1_PIN, HIGH);    //Apply high voltage to the DUT
  digitalWrite(FIRE2_PIN, HIGH);

  measureValues();
  
  digitalWrite(FIRE1_PIN, LOW);      //Remove high voltage from the DUT
  digitalWrite(FIRE2_PIN, LOW);
}

void measureValues() {
  measuredValues[HV1] = analogRead(VA1_PIN); // Extra read for delay on switch closure - probably superfluous here
  measuredValues[HV1] = analogRead(VA1_PIN);
  measuredValues[IA_HI_1] = analogRead(IA1_HI_PIN);
  measuredValues[IA_HI_1] = analogRead(IA1_HI_PIN);
  measuredValues[IA_LO_1] = analogRead(IA1_LO_PIN);
  measuredValues[IA_LO_1] = analogRead(IA1_LO_PIN);
  
  measuredValues[HV2] = analogRead(VA2_PIN);
  measuredValues[IA_HI_2] = analogRead(IA2_HI_PIN);
  measuredValues[IA_HI_2] = analogRead(IA2_HI_PIN);
  measuredValues[IA_LO_2] = analogRead(IA2_LO_PIN);
  measuredValues[IA_LO_2] = analogRead(IA2_LO_PIN);
}

/****************************************************************************
  Discharges the capacitor banks
****************************************************************************/
void dischargeHighVoltages(int bank) {
  if (bank == 1) {
    digitalWrite(FIRE1_PIN, LOW);
#ifdef WIZARD_MODE
    digitalWrite(CHARGE1_PIN, LOW); // If we're in Wizard mode we treat the charge pins as digital output (i.e. on or off)
#else
    analogWrite(CHARGE1_PIN, 0); // If we're in PWM mode then set the duty cycle for the charge pins to 0
#endif // WIZARD_MODE
    digitalWrite(DISCHARGE1_PIN, HIGH);
    measuredValues[IA_HI_1] = analogRead(IA1_HI_PIN); // Needs some delay for the switch to close
    measuredValues[IA_HI_1] = analogRead(IA1_HI_PIN);
    measuredValues[IA_LO_1] = analogRead(IA1_LO_PIN);
    while (analogRead(IA1_HI_PIN) > 0) { //wait until no discharge current is detected
    //while (analogRead(VA1_PIN) > 0) { //wait until no appreciable voltage is detected
    }
    digitalWrite(DISCHARGE1_PIN, LOW);
  } else if (bank == 2) {
    digitalWrite(FIRE2_PIN, LOW);
#ifdef WIZARD_MODE
    digitalWrite(CHARGE2_PIN, LOW);
#else
    analogWrite(CHARGE2_PIN, 0);
#endif // WIZARD_MODE
    digitalWrite(DISCHARGE2_PIN, HIGH);
    measuredValues[IA_HI_2] = analogRead(IA2_HI_PIN);
    measuredValues[IA_HI_2] = analogRead(IA2_HI_PIN);
    measuredValues[IA_LO_2] = analogRead(IA2_LO_PIN);
    while (analogRead(IA2_HI_PIN) > 0) { //wait until no discharge current is detected
    //while (analogRead(VA2_PIN) > 0) { //wait until no appreciable voltage is detected
    }
    digitalWrite(DISCHARGE2_PIN, LOW);
  }
}
