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

double vHeater = 0; // Persistent value used for rolling averaging
double iHeater = 0; // Persistent value used for rolling averaging

#define AVG_FACTOR 0.99

int duty_cycle = 0;      //Duty cycle of buck converter

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

//#define T1_COUNTER 65411;   // preload timer 65536-16MHz/256/500Hz
#define T1_COUNTER 64911;   // preload timer 65536-16MHz/256/100Hz
//#define T1_COUNTER 34286;   // preload timer 65536-16MHz/256/2Hz

/************************************************************
   SETUP
 ************************************************************/
void setup() {
  Serial.begin(115200); //Setup serial interface

  //I2C SDA is on Arduino Nano pin A4 as standard
  //I2C SCL is on Arduino Nano pin A5 as standard. These pins need no further setup.
  //By default, analog input pins also need no setup

  analogReference(EXTERNAL);                //Use external voltage reference for ADC
  TCCR3B = (TCCR3B & 0b11111000) | 0x01;    //Configure Timers 3 & 4 for internal clock, no prescaling (bottom 3 bits of TCCRxB) for higher PWM frequency
  TCCR4B = (TCCR4B & 0b11111000) | 0x01;

/* Set up timer 1 if we want to use interrupts for heater control
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = T1_COUNTER;       // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
*/

  pinMode(LED_BUILTIN, OUTPUT);             //Arduino built-in LED for debugging

  for (int i = 0; i < ARRAY_LENGTH; i++) {
    targetValues[i] = 0;
  }

  digitalWrite(LED_BUILTIN, HIGH);

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

  dischargeHighVoltages(1);
  dischargeHighVoltages(2);

  Wire.begin(MASTER_ADDR); 
    
  setGridVolts();

  pinMode(PWM_PIN, OUTPUT);
  analogWrite(PWM_PIN, 0);            

  digitalWrite(LED_BUILTIN, LOW);
}

/************************************************************
   MAIN LOOP
 ************************************************************/
void loop() {
  while (Serial.available() > 0) {
    parser.parseInput(Serial.read());
  }

  setHeaterVolts(); // Only if we're not using timer interrupts 
} //End of main program loop

ISR(TIMER1_OVF_vect)        // interrupt service routine 
{
  TCNT1 = T1_COUNTER;       // preload timer
  setHeaterVolts();
}

// USB command interface functions

/************************************************************
   Callback for Info commands
 ************************************************************/
void infoCommand(int index) {
  int success = 1;

  switch (index) {
    case 0: // H/W Version info
      Serial.println("OK: Info(0) = Rev 2 (Mega Pro)");
      break;
    case 1: // S/W Version info
      Serial.print("OK: Info(");
      Serial.print(index);
      Serial.println(") = 0.0.1");
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
      setGridVolts();
      duty_cycle = 0;
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
  for (int i = 0; i < ARRAY_LENGTH; i++) {
    Serial.print(measuredValues[i]);
    if (i < ARRAY_LENGTH - 1) {
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
      //sendToSlave();
    } else if (index == VG1 || index == VG2) {
      setGridVolts();
    }

    Serial.print("OK: Set(");
    Serial.print(index);
    Serial.print(") = ");
    Serial.println(intParam);
  } else {
    Serial.print("ERR: Set(");
    Serial.print(index);
    Serial.print(") = ");
    Serial.println(intParam);
    Serial.print(" ");
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
  }

  return status;
}

/************************************************************
   Controls the heater buck regulator
 ************************************************************/
void setHeaterVolts() { //Manages the heater buck-converter
  int Vh_adc = analogRead(VH_PIN);
  int Ih_adc = analogRead(IH_PIN);    //This is the *voltage* developed across the heater current sensing resistor
  
  Vh_adc = Vh_adc - (Ih_adc * 470 / 3770);     //Divide Ih_adc value by 8 and subtract from Vh_adc value to get corrected voltage across heater
                                      //NB: Factor of 8 is determined by the voltage scaling of the heater voltage sense (470R/3K3 resistive divider)
  
  duty_cycle += sgn(targetValues[VH] + 0 - Vh_adc);  // (offset is empirical)

  if (duty_cycle < 0) {               //Ensure that the duty cycle is between 0 and 255
    duty_cycle = 0;
  } else if (duty_cycle > 255) {
    duty_cycle = 255;
  }

  if (Ih_adc > 55) {                  //If heater current is more than 2 amps the device has a problem and we should turn the heaters off
    duty_cycle = 0;
  }

  analogWrite(PWM_PIN, duty_cycle);   //Update buck converter with new duty cycle

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

/****************************************************************************
  Charges up the high-voltage capacitor banks to the target values
****************************************************************************/
int chargeHighVoltages() { //Manages the HV supply
  digitalWrite(FIRE1_PIN, LOW);                       //Turn off MOSFETs (fail-safe measure)
  digitalWrite(FIRE2_PIN, LOW);
  analogWrite(CHARGE1_PIN, 0);
  analogWrite(CHARGE2_PIN, 0);
  //digitalWrite(DISCHARGE1_PIN, LOW);
  //digitalWrite(DISCHARGE2_PIN, LOW);
  analogWrite(DISCHARGE1_PIN, 0);
  analogWrite(DISCHARGE2_PIN, 0);

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
    setHeaterVolts(); //Keep the heater happy

    int timeout2 = 0;
    while (measuredValues[HV1] < (targetValues[HV1] + OVERVOLTAGE)) { //If voltage is too low, charge capacitor
      setHeaterVolts(); //Keep the heater happy

      int duty = setDuty(measuredValues[HV1], targetValues[HV1]);
      analogWrite(CHARGE1_PIN, duty); // If we're in PWM mode then set the duty cycle according to the (inverse) voltage gap
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
      setHeaterVolts(); //Keep the heater happy

      int duty = setDuty(measuredValues[HV2], targetValues[HV2]);
      analogWrite(CHARGE2_PIN, duty); // If we're in PWM mode then set the duty cycle according to the (inverse) voltage gap
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
    analogWrite(CHARGE1_PIN, 0);
    analogWrite(CHARGE2_PIN, 0);
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

  noInterrupts();                   //We don't want the measurement to be affected by servicing the heater

  digitalWrite(FIRE1_PIN, HIGH);    //Apply high voltage to the DUT
  digitalWrite(FIRE2_PIN, HIGH);

  measureValues();
  
  digitalWrite(FIRE1_PIN, LOW);      //Remove high voltage from the DUT
  digitalWrite(FIRE2_PIN, LOW);

  interrupts();
}

void measureValues() {
  measuredValues[HV1] = analogRead(VA1_PIN); // Extra read for delay on switch closure - probably superfluous here
  measuredValues[HV1] = analogRead(VA1_PIN);
  measuredValues[IA_HI_1] = analogRead(IA1_HI_PIN);
  measuredValues[IA_HI_1] = analogRead(IA1_HI_PIN);
  measuredValues[IA_LO_1] = analogRead(IA1_LO_PIN);
  measuredValues[IA_LO_1] = analogRead(IA1_LO_PIN);
  measuredValues[IA_XHI_1] = analogRead(IA1_XHI_PIN);
  measuredValues[IA_XHI_1] = analogRead(IA1_XHI_PIN);
  
  measuredValues[HV2] = analogRead(VA2_PIN);
  measuredValues[IA_HI_2] = analogRead(IA2_HI_PIN);
  measuredValues[IA_HI_2] = analogRead(IA2_HI_PIN);
  measuredValues[IA_LO_2] = analogRead(IA2_LO_PIN);
  measuredValues[IA_LO_2] = analogRead(IA2_LO_PIN);
  measuredValues[IA_XHI_2] = analogRead(IA2_XHI_PIN);
  measuredValues[IA_XHI_2] = analogRead(IA2_XHI_PIN);
}

/****************************************************************************
  Discharges the capacitor banks
****************************************************************************/
void dischargeHighVoltages(int bank) {
  if (bank == 1) {
    digitalWrite(FIRE1_PIN, LOW);
    analogWrite(CHARGE1_PIN, 0); // If we're in PWM mode then set the duty cycle for the charge pins to 0
    //digitalWrite(DISCHARGE1_PIN, HIGH);
    analogWrite(DISCHARGE1_PIN, 128); //to be kind to the discharge resistor!
    measuredValues[IA_HI_1] = analogRead(IA1_HI_PIN); // Needs some delay for the switch to close
    measuredValues[IA_HI_1] = analogRead(IA1_HI_PIN);
    measuredValues[IA_LO_1] = analogRead(IA1_LO_PIN);
    while (analogRead(IA1_HI_PIN) > 0) { //wait until no discharge current is detected
    //while (analogRead(VA1_PIN) > 0) { //wait until no appreciable voltage is detected
      setHeaterVolts(); //Keep the heater happy
    }
    digitalWrite(DISCHARGE1_PIN, LOW);
  } else if (bank == 2) {
    digitalWrite(FIRE2_PIN, LOW);
    analogWrite(CHARGE2_PIN, 0);
    //digitalWrite(DISCHARGE2_PIN, HIGH);
    analogWrite(DISCHARGE2_PIN, 128); //to be kind to the discharge resistor!
    measuredValues[IA_HI_2] = analogRead(IA2_HI_PIN);
    measuredValues[IA_HI_2] = analogRead(IA2_HI_PIN);
    measuredValues[IA_LO_2] = analogRead(IA2_LO_PIN);
    while (analogRead(IA2_HI_PIN) > 0) { //wait until no discharge current is detected
    //while (analogRead(VA2_PIN) > 0) { //wait until no appreciable voltage is detected
      setHeaterVolts(); //Keep the heater happy
    }
    digitalWrite(DISCHARGE2_PIN, LOW);
  }
}

int sgn(int value) {
  if (value == 0) {
    return 0;
  }

  if (value > 0) {
    return 1;
  }

  if (value < 0) {
    return -1;
  }
}
