#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
// Adafruit_ADS1015 ads;    

#define NUMSAMPLES 10

// ADS1115 Calibration at 10v *1000
// Ex: 10 / 18107 = 0,000552273
const float adc0_calibration = 0.1780487;
const float adc1_calibration = 0.2134769;
const float adc2_calibration = 0.4977307;
const float adc3_calibration = 0.5890515;


int samplesadc0[NUMSAMPLES];
int samplesadc1[NUMSAMPLES];
int samplesadc2[NUMSAMPLES];
int samplesadc3[NUMSAMPLES];

void setup(void) 
{
  Serial.begin(19200);
  Serial.println("Hello!");
  
  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");
  
  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  ads.setGain(GAIN_TWOTHIRDS); 
  ads.begin();
}

void loop(void) 
{
  int16_t adc0, adc1, adc2, adc3;
  float adc0V, adc1V, adc2V, adc3V;

  uint8_t i;
  float average0;
  float average1;
  float average2;
  float average3;
 
  // take N samples in a row, with a slight delay
  for (i=0; i< NUMSAMPLES; i++) {
   samplesadc0[i] = ads.readADC_SingleEnded(0);
   samplesadc1[i] = ads.readADC_SingleEnded(1);
   samplesadc2[i] = ads.readADC_SingleEnded(2);
   samplesadc3[i] = ads.readADC_SingleEnded(3);
  }

// average all the samples out
  average0 = 0;
  average1 = 0;
  average2 = 0;
  average3 = 0;
  
  for (i=0; i< NUMSAMPLES; i++) {
     average0 += samplesadc0[i];
     average1 += samplesadc1[i];
     average2 += samplesadc2[i];
     average3 += samplesadc3[i];
  }
  average0 /= NUMSAMPLES;
    average1 /= NUMSAMPLES;
      average2 /= NUMSAMPLES;
        average3 /= NUMSAMPLES;
  
  adc0V = (average0*adc0_calibration)/1000;
  adc1V = (average1*adc1_calibration)/1000;
  adc2V = (average2*adc2_calibration)/1000;
  adc3V = (average3*adc3_calibration)/1000;
  
  Serial.print("AIN0: ");  Serial.print(average0); Serial.print(" \tV: ");Serial.println(adc0V,5);
  Serial.print("AIN1: ");  Serial.print(average1); Serial.print(" \tV: ");Serial.println(adc1V,5);
  Serial.print("AIN2: ");  Serial.print(average2); Serial.print(" \tV: ");Serial.println(adc2V,5);
  Serial.print("AIN3: ");  Serial.print(average3); Serial.print(" \tV: ");Serial.println(adc3V,5);

  Serial.println(" ");
  
  delay(3000);
}
