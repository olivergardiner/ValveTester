# ValveTester
QT app for using the Valve Wizard's arduino based valve tester

Project home page: https://whitecottage.org.uk/valve-wizard-valve-analyser/

This project contains both the Arduino code and a QT application that will run the analyser and plot characteristic curves for various types of valve. While Merlin's original code provides a text oriented UI direct from the Arduino, this is limiting for programmatic use. Instead, the code presented here implements a line-oriented protocol for issuing commands to the Arduino and receiving responses back that allow the paramters of the test, and the testing itself, to be controlled.

Conceptually, the Arduino protocol is similar (although also quite different to) to G Code. It currently supports three different types of command:

S (Set) commands, which take the form S# <VALUE> - these are for setting test parameters
G (Get) commands, which take the form G# - these are for getting test parameters / measured values
M (Mode) commands, which take the form M# - these are for managing the mode of the Arduino (inclulding test mode)

In all cases above, '#' represents an integer value to identify the specific sub-type of the command.

A response should always be expected - if the Arduino doesn't send one it means that something has gone wrong! There are two types of response: error and ok. If the response begins with "Err:" it is an error response and if it begins with "Ok:" it is an ok response - in which case further information (e.g. a fetched value) will also be returned.

The operation of the Valve Analyser is entirely based around 10 test parameters. While the implementation is transparent to any client connected to the Arduino, these parameters are stored in two internal arrays (target and measured) whose indeces are defined as follows:

0		Heater voltage (target and measured)
1		Heater current (measured only)
2		Grid 1 voltage (target only)
3		Anode 1 voltage (target and measured)
4		Anode 1 current, high sensitivity (measured only)
5		Anode 1 current, low sensitivity (measured only)
6		Grid 2 voltage (target only)
7		Anode 2/Screen voltage (target and measured)
8		Anode 2/Screen current, high sensitivity (measured only)
9		Anode 2/Screen current, low sensitivity (measured only)

All of these parameters are specified or measured as integers which represent the values supplied to or read from the internal ADCs and DACs.

The full list of commands and their responses is listed below:

Mode commands
=============
M0		Safe mode - discharges the capacitor banks, turns off the heater and grid, and sets all target values to 0
M1		Disharges the capacitor banks - important between sweeps of the Anode or Grid
M2		Test mode - executes a test according to the set of target values and returns the set of measured values
M3		Debug mode 1 - charges the capacitor banks (allows the charging process to be validated on the hardware)
M4		Debug mode 2 - charges the capacitor banks and applies them to the device under test
M5		Measure and display all measured values

When a test is run using the M commands, the response will also return a comma separated list of values corresponding to the measured values of each parameter (noting that the Grid voltages are not actually measured and the value returned is the value used to programme the corresponding DAC).

Set/Get commands
================
The main (only, for the time being) Set and Get and commands are derived from the internal array of target/measured values, hence S0 <Value> will set the heater voltage and G0 will return the heater voltage, whereas S3 and G3 commands will set and query the Anode 1 voltage respectively. For completeness, the S# and G# commands are:

S<0-9> <value>		Sets the associated test parameter to <Value>
G<0-9>				Gets the measured value of the associated test parameter

For parameters that are only measured (e.g. parameter 1, Heater current) the corresponding Set command will receive an ok response but have no effect. For parameters that are not directly measured (i.e. the Grid votages), the value returned by the corresponding Get command is simply the specified target value. 

Value specification
===================
As mentioned earlier, the values used for the test parameters are taken from the ADCs or sent to the DACs on the hardware, but these require an appropriate conversion to map them to their corresponding voltages and currents. Both the ADCs and the DACs measure their full scale values against an externally provided voltage reference - 4.096v for the grids, anode and screen and (according to the original circuit) 2.048v for the heater. The software can accommodate (via preferences) hardware that applies a reference voltage of 4.096v for all measurements as this provides access to slightly higher heater voltages. Taking the different parameters in turn, the conversion factors required are described below:

* Heater voltage
The heater voltage is sensed using an Arduino analogue input, which is implemented as a 10 bit ADC. That means that the full scale measurement is 1023 and, for the heaters with the default voltage reference, this value corresponds to 2.096v on the analogue input pin. The heater *voltage* is sensed through a potential divider composed of 470R and 3K3 resistors, resulting in a scaling factor of 470 / 3770. If the fraction of full scale (i.e. the measured/target value divided by 1023) is X, then the resulting heater voltage Vh can be expressed as: X * 2.048 * 3770 / 470. Rearranging this, we can see that X = Vh * 470 / (3770 * 2.096) and that the corresponding ADC value is X * 1023. For example, if we wished to set the heater to 6.3v, the corresponding ADC value would be 392 (with a voltage reference of 2.048v). 

Note that with the voltage reference set to 2.048, the maximum heater voltage that can be measured (and, therefore, regulated) is 2.048 * 3770 / 470 = 16.4v. Raising the voltage reference to 4.096v would provide access to voltages closer to the raw heater supply of 24v but has other drawbacks (see below) and is unlikely to be a practical limitation for most people (who will doubtless be mostly using 6.3v or 12.6v heaters).

Regardless of voltage reference, it is very important that the target heater ADC value cannot be set to the maximum of 1023 as this would prevent the feedback in the buck regulator from working properly (in essence the, the regulation loop would not be able to know if the heater voltage was exactly the desired value or any higher voltage). For this reason, the Arduino limits the maximum allowable value to 1000.

* Heater current
The heater current (in common with the other current measurements) is achieved by measuring the voltage drop across a suitable sense resistor - in this case a 0.22R resistor. For this measurement, no scaling is applied and the analogue input is directly connected to the top of the sense resistor giving a maximum measurable heater current of 2.096v / 0.22 or 9.5A! In practice we would never get close to that value but gives plenty of scope for handling the meager requirements of an EF86 (200mA) through to the big bottle demands of a KT88 (1.8A). What this does show, however, is the tradeoff between the two possible reference voltages. The lower voltage reference of 2.048v increases the efffective resolution at lower currents. For example, a 200mA heater will register as an ADC value of 1023 * 0.2 * 0.22 / 2.048 = 22. This equates to a resolution of nearly 5% of nominal current (the absolute resolution, i.e. the current per increment of the ADC value, is 2.048 / 0.22 / 1023 = 9.1mA. If we increase the voltage reference to 4.096v we gain access to heater voltages from 16v to 24v but we halve the current sensing resolution (the current increment increases to 18.2mA or nearly 10% of the nominal current of an EF86.

For the strict accuracy, the measured value of heater voltage accounts for the voltage dropped by the current sensing resistor, although the low value of this resistor means that its effect is minimal.

* Grid voltage
The Grid voltages are simply set by programming a 14 bit DAC (with a 4.096 full scale reference) whichis then followed by an inverting amplifier (Grid voltages should be negative). The gain of the inverting amplifier is set to 16.5 and, hence, programmed Grid values from 0 to 4095 correspond to negative Grid voltages of 0 to 4.096 * 16.5, i.e.67.5v.

* Anode voltage
The principles for measuring the Anodes/Screens are the same as for the heater but the very high voltages involved require some slight adjustments. Clearly, for the Anode/Screen voltage, the most important adjustment is simply the scaling factor used to translate the actual measured voltage to something within the range 0 to 4.096v that can be measured by the ADCs. This is achieved by building a potential divider with three 470K resistors in the upper half and two 4K7 in the bottom half - this creates a scaling factor of 9400 / 1419400 which, in turn, means that the Anode voltage can be varied from 0v up to a theoretical maximum of around 620v. In practice, the HT supply may not make this achievable - any attempt to set an unachievable Anode/Screen voltage will result in a timeout error as the analyser will simply keep trying to charge the caacitor banks. I followed Merlin's advice and used the isolating transformer from a shaver socket and this actually gives me an unloaded HT supply of around 720v when the nominal 240v is rectified and doubled.

* Anode current
The Anode/Screen current measurement is the most complicated because of the potential for high currents to tax larger sensing resistors. These currents are measured using "low side" sensing on the cathodes of the capcitor banks and the "high resolution" sense resistance is made up of two 15R resistors (i.e. 30R) in series with three 10R resistors in parallel (i.e. 3R333). The combination of resistors in series or parallel both improves the accuracy of the overall resistance (as errors tend to cancel, statistically) and power handling. Even so, a current of 130mA will dissipate 0.25W in a 15R resistor, so we need a safety limit to prevent anything too excessive and this is provided by a string of three rectifier diodes (giving a total forward drop of around 2.1v) that bypass the two 15R resistors (hence limiting the maximum current through the resistors to around 70mA. Once the diodes are conducting, the voltage across all of the sensing resistors is no longer an accurate measure of current but the diodes only bypass the 15R resistors and all of the current still flows through the three parallel 10R resistors. Thus, for higher currents (which will generate higher voltage drops and hence not suffer from low relative resolution) we can still sense the current accurately, albeit with a different conversion factor. Note that the three 10R 0.25W resistors in parallel are equivalent to a 3R333 0.75W resistor and so can happily tolerate much higher currents than their larger counterparts.

For high resolution (low current) sensing, we want to measure the voltage dropped across the total sensing resistance, i.e. 30R in series with 3R333 or 33R333. The sensed voltage is passed through an amplifier with a gain of -2 and so, given the 4.096v reference for full scale, the maximum current that can be sensed this way (i.e. the current corresponding to an ADC value of 1023) is 4.096 / 2 / 33.333 = 61.4mA. However, we know that the current for this sensing method is limited to around 70mA and we don't want the sensed current to be inaccurate as the diodes begin to forward conduct, so it would be prudent to distrust the low current sensing well before full scale. In the client code, it is assumed that the low current sensing is inaccurate once it exceeds an ADC value of 900.

Once the Anode/Screen current has reached a value that makes low current sensing unwise, the high current sensing can be used instead. With a sense resistance of 3R333 in stead of 33R333, the scale factor simply changes by a factor of 10 so a full scale measurement now corresponds to a current of 614mA. Such a current would actually blow our power budget for the 10R resistors but finding a working device that would deliver such a current would be a challenge and the capacitor banks can only deliver a finite charge, thus providing a natural back stop.


