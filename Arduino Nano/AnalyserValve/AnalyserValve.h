#define DIODE_MODE 1
#define TRIODE_MODE 2
#define PENTODE_MODE 3
#define MIN_HV_INCREMENT 5 //Minimum increment between each high-voltage datapoint in a sweep, in volts

#define MASTER 1            //For when hardware ID pin is high
#define SLAVE 0             //For when hardware ID pin is low

#define MASTER_ADDR 0x0A      //I2C Address of the MASTER Arduino
#define SLAVE_ADDR 0x0B       //I2C Address of the SLAVE Arduino
#define DAC1_ADDR 0x60  //I2C Address of one MCP4725, by default pin A0 is pulled to GND.
#define DAC2_ADDR 0x61   //I2C Address of other MCP4725; its pin A0 must be pulled HIGH to make address 0x61 (also remove pull-up resistors). Note that the breakout board uses the MCP4725A0.
//NB: I2C can send 32 ints per transmission.

//#define WIZARD_MODE

#define HARDWARE_ID_PIN 12   //High for master, low for slave
#define VH_PIN A2            //Heater-voltage sense from buck regulator
#define IH_PIN A3            //Heater-current sense from buck regulator
#define PWM_PIN 6            //PWM drive signal to buck regulator MOSFET
//#define CHARGE1_PIN 8        //Drive to high-voltage charge MOSFET
#define CHARGE1_PIN 9        //Drive to high-voltage charge MOSFET - moved to pin D9 in order to enable PWM
#define DISCHARGE1_PIN 11    //Drive to high-voltage discharge MOSFET 
#define FIRE1_PIN 7          //Drive to 'apply high voltage' MOSFET 
#define VA1_PIN A6           //High voltage sense 1
#define VA2_PIN A7           //High voltage sense 2
#define IA1_HI_PIN A1        //Small anode-current sense (large sense resistance) 
#define IA1_LO_PIN A0        //Large anode-current sense (small sense resistance)
#define IA2_HI_PIN A3        //Small anode-current sense (large sense resistance) 
#define IA2_LO_PIN A2        //Large anode-current sense (small sense resistance)
#define DISCHARGE2_PIN 4
#define FIRE2_PIN 2
#define CHARGE2_PIN 3       
#define LV_DETECT_PIN 9  //Pin to detect if heater power supply is present

#define ARRAY_LENGTH 10         

//INDICES FOR BOTH ARRAYS
#define VH       0   //Heater voltage  [example: 12.6V = adc391   6.3V = adc195] 
#define IH       1   //Heater current
#define VG1      2   //Grid voltage 1  [example: 1V = dac60, 10V=dac605, 20V=dac1210, 30V=dac1815, 40V=dac2420, 50V=dac3020]
#define HV1      3   //Anode voltage 1 [example: 600V=3.97V=adc992   300V=1.98V=adc496    200V=0.76V=330   100V=0.381V=V=adc165]
#define IA_HI_1  4   //Anode current hi 1
#define IA_LO_1  5   //Anode current lo 1
#define VG2      6   //Grid voltage 2
#define HV2      7   //Anode voltage 2
#define IA_HI_2  8   //Anode current hi 2
#define IA_LO_2  9   //Anode current lo 2

#define HT_TIMEOUT 64000
#define CHARGING_SPEED 32
#define HV_DUTY_MIN 56 // Don't set too low or pentode tests will struggle
#define THRESHOLD 600 // 600 corresponds to around 360v - once over this we start increasing the duty cycle from min value
#define OVERVOLTAGE 5 // Plan to overshoot HV a tad to account for leakage (we aim for this value and measure if we're within twice this)

/************************************************************   
*FUNCTION PROTOTYPES
************************************************************/
void flushSerialBuffer();
void infoCommand(int index);
void modeCommand(int index);
void getCommand(int index);
void setCommand(int index, int intParam);
void commandError(const char *command);
void printValues();

void sendToSlave();
void masterReceiveData(int howMany);
void requestFromSlave();
void slaveReceiveData(int howMany);
void slaveAnswerRequest();

int runTest();
void setHeaterVolts();
void setGridVolts();
int chargeHighVoltages();
bool checkAnodeVoltage(int measured, int target);
int setDuty(int measured, int target);
void dischargeHighVoltages(int bank);
void chargeOff();
void doMeasurement();
void measureValues();
