/*
 CapacitiveSense.h - Capacitive Sensing Library for 'duino / Wiring
 https://github.com/PaulStoffregen/CapacitiveSensor
 http://www.pjrc.com/teensy/td_libs_CapacitiveSensor.html
 http://playground.arduino.cc/Main/CapacitiveSensor
 Copyright (c) 2009 Paul Bagder
 Updates for other hardare by Paul Stoffregen, 2010-2016
 vim: set ts=4:

 Permission is hereby granted, free of charge, to any person obtaining a
 copy of this software and associated documentation files (the "Software"),
 to deal in the Software without restriction, including without limitation
 the rights to use, copy, modify, merge, publish, distribute, sublicense,
 and/or sell copies of the Software, and to permit persons to whom the
 Software is furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 DEALINGS IN THE SOFTWARE.
*/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#include "WConstants.h"
#endif

#include "CapacitiveSensor.h"

// Constructor /////////////////////////////////////////////////////////////////
// Function that handles the creation and setup of instances

CapacitiveSensor::CapacitiveSensor(uint8_t sendPin1, uint8_t sendPin2, uint8_t receivePin, uint8_t shieldPin, uint8_t recPinSh1, uint8_t recPinSh2)
{
	// initialize this instance's variables
	// Serial.begin(9600);		// for debugging
	error = 1;
	loopTimingFactor = 310;		// determined empirically -  a hack

	CS_Timeout_Millis = (2000 * (float)loopTimingFactor * (float)F_CPU) / 16000000;
	CS_AutocaL_Millis = 20000;

	// Serial.print("timwOut =  ");
	// Serial.println(CS_Timeout_Millis);

	// get pin mapping and port for send Pin - from PinMode function in core

#ifdef NUM_DIGITAL_PINS
	if (sendPin1 >= NUM_DIGITAL_PINS) error = -1;
 if (sendPin2 >= NUM_DIGITAL_PINS) error = -1;
	if (receivePin >= NUM_DIGITAL_PINS) error = -1;
 if (shieldPin >= NUM_DIGITAL_PINS) error = -1;
 if (recPinSh1 >= NUM_DIGITAL_PINS) error = -1;
 if (recPinSh2 >= NUM_DIGITAL_PINS) error = -1;
#endif

	pinMode(sendPin1, OUTPUT);						// sendpin1 to OUTPUT
  pinMode(sendPin2, OUTPUT);            // sendPin2 to OUTPUT
	pinMode(receivePin, INPUT);						// receivePin to INPUT
  pinMode(recPinSh1, INPUT);            // recPinSh1 to INPUT
  pinMode(recPinSh2, INPUT);            // recPinSh2 to INPUT
  pinMode(shieldPin, OUTPUT);           // shieldPin to OUTPUT
	digitalWrite(sendPin1, LOW);
  digitalWrite(sendPin2, LOW);
  digitalWrite(shieldPin, LOW);

	s1Bit = PIN_TO_BITMASK(sendPin1);					// get send1 pin's ports and bitmask
	s1Reg = PIN_TO_BASEREG(sendPin1);					// get pointer to output register

  s2Bit = PIN_TO_BITMASK(sendPin2);          // get send2 pin's ports and bitmask
  s2Reg = PIN_TO_BASEREG(sendPin2);          // get pointer to output register
  
	rBit = PIN_TO_BITMASK(receivePin);				// get receive pin's ports and bitmask
	rReg = PIN_TO_BASEREG(receivePin);

  rsh1Bit = PIN_TO_BITMASK(recPinSh1);        // get receive pin's ports and bitmask
  rsh1Reg = PIN_TO_BASEREG(recPinSh2);

  rsh2Bit = PIN_TO_BITMASK(recPinSh2);        // get receive pin's ports and bitmask
  rsh2Reg = PIN_TO_BASEREG(recPinSh2);

  shBit = PIN_TO_BITMASK(shieldPin);        // get shield pin's ports and bitmask
  shReg = PIN_TO_BASEREG(shieldPin);

	// get pin mapping and port for receive Pin - from digital pin functions in Wiring.c
	leastTotal = 0x0FFFFFFFL;   // input large value for autocalibrate begin
	lastCal = millis();         // set millis for start
}

// Public Methods //////////////////////////////////////////////////////////////
// Functions available in Wiring sketches, this library, and other libraries

long CapacitiveSensor::capacitiveSensor(uint8_t samples)
{
	total = 0;
	if (samples == 0) return 0;
	if (error < 0) return -1;            // bad pin


	for (uint8_t i = 0; i < samples; i++) {    // loop for samples parameter - simple lowpass filter
  /*digitalWrite(3, HIGH);
  delayMicroseconds(80);
  digitalWrite(3, LOW);*/
		if (SenseOneCycle() < 0)  return -2;   // variable over timeout
}

		// only calibrate if time is greater than CS_AutocaL_Millis and total is less than 10% of baseline
		// this is an attempt to keep from calibrating when the sensor is seeing a "touched" signal

		if ( (millis() - lastCal > CS_AutocaL_Millis) && abs(total  - leastTotal) < (int)(.10 * (float)leastTotal) ) {

			// Serial.println();               // debugging
			// Serial.println("auto-calibrate");
			// Serial.println();
			// delay(2000); */

			leastTotal = 0x0FFFFFFFL;          // reset for "autocalibrate"
			lastCal = millis();
		}
		/*else{                                // debugging
			Serial.print("  total =  ");
			Serial.print(total);

			Serial.print("   leastTotal  =  ");
			Serial.println(leastTotal);

			Serial.print("total - leastTotal =  ");
			x = total - leastTotal ;
			Serial.print(x);
			Serial.print("     .1 * leastTotal = ");
			x = (int)(.1 * (float)leastTotal);
			Serial.println(x);
		} */

	// routine to subtract baseline (non-sensed capacitance) from sensor return
	if (total < leastTotal) leastTotal = total;                 // set floor value to subtract from sensed value
	return(total - leastTotal);

}

long CapacitiveSensor::capacitiveSensorRaw(uint8_t samples)
{
	total = 0;
	if (samples == 0) return 0;
	if (error < 0) return -1;                  // bad pin - this appears not to work

	for (uint8_t i = 0; i < samples; i++) {    // loop for samples parameter - simple lowpass filter
		if (SenseOneCycle() < 0)  return -2;   // variable over timeout
	}

	return total;
}


void CapacitiveSensor::reset_CS_AutoCal(void){
	leastTotal = 0x0FFFFFFFL;
}

void CapacitiveSensor::set_CS_AutocaL_Millis(unsigned long autoCal_millis){
	CS_AutocaL_Millis = autoCal_millis;
}

void CapacitiveSensor::set_CS_Timeout_Millis(unsigned long timeout_millis){
	CS_Timeout_Millis = (timeout_millis * (float)loopTimingFactor * (float)F_CPU) / 16000000;  // floats to deal with large numbers
}

int CapacitiveSensor::get_SenseOneCycle(void){
   return SenseOneCycle();
  }

// Private Methods /////////////////////////////////////////////////////////////
// Functions only available to other functions in this library

int CapacitiveSensor::SenseOneCycle(void)
{
    noInterrupts();
	DIRECT_WRITE_LOW(s1Reg, s1Bit);	        // sendPin1 Register low
  DIRECT_WRITE_LOW(s2Reg, s2Bit);          // sendPin2 Register low
  DIRECT_WRITE_HIGH(shReg, shBit);      // shieldPin1 Register High
	DIRECT_MODE_INPUT(rReg, rBit);	      // receivePin to input (pullups are off)
  DIRECT_MODE_INPUT(rsh1Reg, rsh1Bit);  // recPinSh1 to input (pullups are off)
  DIRECT_MODE_INPUT(rsh2Reg, rsh2Bit);  // recPinSh2 to input (pullups are off)
	DIRECT_MODE_OUTPUT(rReg, rBit);       // receivePin to OUTPUT
  DIRECT_MODE_OUTPUT(rsh1Reg, rsh1Bit); // recPinSh1 to OUTPUT
  DIRECT_MODE_OUTPUT(rsh2Reg, rsh2Bit); // recPinSh2 to OUTPUT
	DIRECT_WRITE_LOW(rReg, rBit);       	// pin is now LOW AND OUTPUT
  DIRECT_WRITE_LOW(rsh1Reg, rsh1Bit);   // pin is now LOW AND OUTPUT
  DIRECT_WRITE_HIGH(rsh2Reg, rsh2Bit);   // pin is now LOW AND OUTPUT
	delayMicroseconds(10);
	DIRECT_MODE_INPUT(rReg, rBit);	      // receivePin to input (pullups are off)
  DIRECT_MODE_INPUT(rsh1Reg, rsh1Bit);  // recPinSh1 to input (pullups are off)
  DIRECT_MODE_INPUT(rsh2Reg, rsh2Bit);   // recPinSh2 to input (pullups are off)
	DIRECT_WRITE_HIGH(s1Reg, s1Bit);	      // sendPin1 High
 DIRECT_WRITE_HIGH(s2Reg, s2Bit);       // sendPin2 High
  DIRECT_WRITE_LOW(shReg, shBit);       // shieldPin Low
    interrupts();

	while ( !DIRECT_READ(rReg, rBit) && (total < CS_Timeout_Millis) ) {  // while receive pin is LOW AND total is positive value
		total++;
	}
	//Serial.print("SenseOneCycle(1): ");
	//Serial.println(total);

	if (total > CS_Timeout_Millis) {
		return -2;         //  total variable over timeout
	}

	// set receive pin HIGH briefly to charge up fully - because the while loop above will exit when pin is ~ 2.5V
    noInterrupts();
	DIRECT_WRITE_HIGH(rReg, rBit);
  DIRECT_WRITE_HIGH(rsh1Reg, rsh1Bit);
  DIRECT_WRITE_LOW(rsh2Reg, rsh2Bit);
	DIRECT_MODE_OUTPUT(rReg, rBit);  // receivePin to OUTPUT - pin is now HIGH AND OUTPUT
  DIRECT_MODE_OUTPUT(rsh1Reg, rsh1Bit);  // receivePin to OUTPUT - pin is now HIGH AND OUTPUT
  DIRECT_MODE_OUTPUT(rsh2Reg, rsh2Bit);  // receivePin to OUTPUT - pin is now LOW AND OUTPUT
	DIRECT_WRITE_HIGH(rReg, rBit);
  DIRECT_WRITE_HIGH(rsh1Reg, rsh1Bit);
  DIRECT_WRITE_LOW(rsh2Reg, rsh2Bit);
	DIRECT_MODE_INPUT(rReg, rBit);	// receivePin to INPUT (pullup is off)
  DIRECT_MODE_INPUT(rsh1Reg, rsh1Bit);  // recPinSh1 to input (pullups are off)
  DIRECT_MODE_INPUT(rsh2Reg, rsh2Bit);  // recPinSh2 to input (pullups are off)
	DIRECT_WRITE_LOW(s1Reg, s1Bit);	// sendPin1 LOW
  DIRECT_WRITE_LOW(s2Reg, s2Bit);  // sendPin2 LOW
  DIRECT_WRITE_HIGH(shReg, shBit); //shieldPin LOW
    interrupts();

#ifdef FIVE_VOLT_TOLERANCE_WORKAROUND
	DIRECT_MODE_OUTPUT(rReg, rBit);
  DIRECT_MODE_OUTPUT(rsh1Reg, rsh1Bit);
  DIRECT_MODE_OUTPUT(rsh2Reg, rsh2Bit);
	DIRECT_WRITE_LOW(rReg, rBit);
  DIRECT_WRITE_LOW(rsh1Reg, rsh1Bit);
  DIRECT_WRITE_LOW(rsh2Reg, rsh2Bit);
	delayMicroseconds(10);
	DIRECT_MODE_INPUT(rReg, rBit);	// receivePin to INPUT (pullup is off)
  DIRECT_MODE_INPUT(rsh1Reg, rsh1Bit);  // recPinSh1 to input (pullups are off)
  DIRECT_MODE_INPUT(rsh2Reg, rsh2Bit);  // recPinSh2 to input (pullups are off)
#else
	while ( DIRECT_READ(rReg, rBit) && (total < CS_Timeout_Millis) ) {  // while receive pin is HIGH  AND total is less than timeout
		total++;
	}
#endif
	//Serial.print("SenseOneCycle(2): ");
	//Serial.println(total);

	if (total >= CS_Timeout_Millis) {
		return -2;     // total variable over timeout
	} else {
		return 1;
	}
}
