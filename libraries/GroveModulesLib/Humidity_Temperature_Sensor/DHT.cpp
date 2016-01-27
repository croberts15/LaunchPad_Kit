/* DHT library 

MIT license
written by Adafruit Industries
*/

#include "DHT.h"

#if defined(__LM4F120H5QR__) || defined(__TM4C123GH6PM__) || defined(__TM4C1294NCPDT__) || defined(__TM4C1294XNCZAD__)
#define tiva
#elif defined(__MSP430F5529__) || defined(__MSP430FR6989)
#define msp430
#elif defined(__MSP432P401R__)
#include <ti/drivers/GPIO.h>
#define msp432
#else 
#error Platform not supported
#endif

DHT::DHT(uint8_t pin, uint8_t type) {
  _pin = pin;
  _type = type;
  firstreading = true;
}

void DHT::begin(void) {
  // set up the pins!
  pinMode(_pin, INPUT);
  digitalWrite(_pin, HIGH);
  _lastreadtime = 0;
}

//boolean S == Scale.  True == Farenheit; False == Celcius
float DHT::readTemperature(bool S) {
  float f;

  if (read()) {
    switch (_type) {
    case DHT11:
      f = data[2];
      if(S)
      	f = convertCtoF(f);
      	
      return f;
    case DHT22:
    case DHT21:
      f = data[2] & 0x7F;
      f *= 256;
      f += data[3];
      f /= 10;
      if (data[2] & 0x80)
	f *= -1;
      if(S)
	f = convertCtoF(f);

      return f;
    }
  }
  Serial.print("Read fail");
  return 0;
}

float DHT::convertCtoF(float c) {
	return c * 9 / 5 + 32;
}

float DHT::readHumidity(void) {
  float f;
  if (read()) {
    switch (_type) {
    case DHT11:
      f = data[0];
      return f;
    case DHT22:
    case DHT21:
      f = data[0];
      f *= 256;
      f += data[1];
      f /= 10;
      return f;
    }
  }
  Serial.print("Read fail");
  return 0;
}


boolean DHT::read(void) {
  uint8_t laststate = HIGH;
  uint16_t counter = 0;
  uint8_t j = 0, i;
  unsigned long currenttime;  
  
  #ifdef msp432
  int limit = microsecondsToClockCycles(1)/3; //1/3 Clock cycle due to driverlib overhead
  #endif
  
  #if defined (msp430) || defined (tiva) 
  int limit = microsecondsToClockCycles(3); //Needs to have the energia.h file for LM4f fixed for this function, 80MHz hard coded
  uint8_t bit = digitalPinToBitMask(_pin);
  uint8_t port = digitalPinToPort(_pin);
  #endif
  
  #ifdef msp430
  volatile uint8_t* portIn = (portInputRegister(port)); //For MSP430
  #endif
  
  #ifdef tiva
  uint32_t portBase = (uint32_t) portBASERegister(port); // For Tiva
  #endif
  
  // pull the pin high and wait 250 milliseconds
  digitalWrite(_pin, HIGH);
  delay(250);

  currenttime = millis();
  if (currenttime < _lastreadtime) {
    // ie there was a rollover
    _lastreadtime = 0;
  }
  if (!firstreading && ((currenttime - _lastreadtime) < 2000)) {
    return true; // return last correct measurement
    //delay(2000 - (currenttime - _lastreadtime));
  }
  firstreading = false;
  /*
    Serial.print("Currtime: "); Serial.print(currenttime);
    Serial.print(" Lasttime: "); Serial.print(_lastreadtime);
  */
  _lastreadtime = millis();

  data[0] = data[1] = data[2] = data[3] = data[4] = 0;
  
  // now pull it low for ~20 milliseconds
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  delay(20);
  //noInterrupts();
  digitalWrite(_pin, HIGH);
  delayMicroseconds(40);
  pinMode(_pin, INPUT);

  // read in timings
  for ( i=0; i< MAXTIMINGS; i++) {
    counter = 0;
	
	#ifdef msp432
	while ((GPIO_read(_pin)) == laststate && counter <= 1000) { //MSP432
      counter++;
    }
	laststate = (GPIO_read(_pin)); //For MSP432 MT
	#endif
	#ifdef tiva
    while ((HWREG(portBase + (0 + (bit << 2)))) == laststate && counter <= 1000) { //For Tiva Series
      counter++;
    }
	laststate = HWREG(portBase + (0 + (bit << 2))); //For Tiva series
	#endif
	
	#ifdef msp430
	while (((*portIn & bit) == laststate) && counter <= 1000) { //For MSP430
	    counter++;
    }
    laststate = (*portIn & bit); //For MSP430
	#endif
	
    if (counter >= 1000) break;

    // ignore first 3 transitions
    if ((i >= 4) && (i%2 == 0)) {
      // shove each bit into the storage bytes
      data[j/8] <<= 1;
      if (counter >= limit)
        data[j/8] |= 1;
      j++;
    }

  }

 //interrupts();
  
  /*
  Serial.println(j, DEC);
  Serial.print(data[0], HEX); Serial.print(", ");
  Serial.print(data[1], HEX); Serial.print(", ");
  Serial.print(data[2], HEX); Serial.print(", ");
  Serial.print(data[3], HEX); Serial.print(", ");
  Serial.print(data[4], HEX); Serial.print(" =? ");
  Serial.println(data[0] + data[1] + data[2] + data[3], HEX);
  */

  // check we read 40 bits and that the checksum matches
  if ((j >= 40) && 
      (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) ) {
    return true;
  }
  

  return false;

}
