/***************************************************************************************************/
/*
   This is an Arduino library for TE Connectivity HTU20D/HTU21D, Sensirion SHT20/SHT21/SHT25, 
   Silicone Labs Si7006/Si7013/Si7020/Si7021 Digital Humidity & Temperature Sensor

   written by : enjoyneering
   sourse code: https://github.com/enjoyneering/

   Sensors features:
   - HTU2xD +1.5v..+3.6v, SHT2x +2.1v..+3.6v, Si70xx +1.9v..+3.6v
   - HTU2xD 0.14uA..0.500uA, SHT2x 0.04uA..0.330uA, Si70xx 0.6uA..180uA
   - integrated resistive heating element HTU2xD/SHT2x 1.83mA, Si70xx 3.1mA..94.2mA
   - temperature range HTU2xD/SHT2x -40C..+125C, Si702x (G-grade) -40C..+80C
   - humidity range 0%..100%
   - typical accuracy T +-0.3C, RH +-2%
   - typical resolution T 0.01C at 14-bits, RH 0.04% at 12-bits
   - maximum T measurement time Si70xx 11msec, HTU2xD 50ms, HTU2xD 85ms
   - maximum RH measurement time HTU2xD 16ms, Si70xx 23msec, HTU2xD 25ms
   - I2C bus speed 100KHz..400KHz
   - response time 8..30sec*
   - recommended to route VDD or GND between I2C lines to reduce crosstalk between SCL & SDA
   - power supply pins must be decoupled with 100nF capacitor
     *measurement with high frequency leads to heating of the sensor,
      measurements must be >= 0.5 seconds apart to keep self-heating below 0.1C

   This device uses I2C bus to communicate, specials pins are required to interface
   Board:                                    SDA              SCL              Level
   Uno, Mini, Pro, ATmega168, ATmega328..... A4               A5               5v
   Mega2560................................. 20               21               5v
   Due, SAM3X8E............................. 20               21               3.3v
   Leonardo, Micro, ATmega32U4.............. 2                3                5v
   Digistump, Trinket, ATtiny85............. PB0              PB2              5v
   Blue Pill, STM32F103xxxx boards.......... PB9/PB7*         PB8/PB6*         3.3v/5v
   ESP8266 ESP-01........................... GPIO0**          GPIO2**          3.3v/5v
   NodeMCU 1.0, WeMos D1 Mini............... GPIO4/D2         GPIO5/D1         3.3v/5v
   ESP32.................................... GPIO21/D21       GPIO22/D22       3.3v
                                             *hardware I2C Wire mapped to Wire1 in stm32duino
                                              see https://github.com/stm32duino/wiki/wiki/API#i2c
                                            **most boards has 10K..12K pullup-up resistor
                                              on GPIO0/D3, GPIO2/D4/LED & pullup-down on
                                              GPIO15/D8 for flash & boot

   Frameworks & Libraries:
   ATtiny  Core  - https://github.com/SpenceKonde/ATTinyCore
   ESP32   Core  - https://github.com/espressif/arduino-esp32
   ESP8266 Core  - https://github.com/esp8266/Arduino
   STM32   Core  - https://github.com/stm32duino/Arduino_Core_STM32


   GNU GPL license, all text above must be included in any redistribution,
   see link for details  - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/

#include <Wire.h>
#include <HTU2xD_SHT2x_Si70xx.h>
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#endif

// const PinMap PinMap_I2C_SDA[] = {
//   {PB_9,  I2C1, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, AFIO_I2C1_ENABLE)},
//   {NC,    NP,    0}
// };
// const PinMap PinMap_I2C_SCL[] = {
//   {PB_8,  I2C1, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, AFIO_I2C1_ENABLE)},
//   {NC,    NP,    0}
// };

float htValue; //to store T/RH result

/*
 sensor type:
  - HTU2xD_SENSOR, SHT2x_SENSOR  //interchangeable with each other, since they have same address & features
  - SI700x_SENSOR, SI702x_SENSOR //interchangeable with each other, since they have same address & features
  - SI701x_SENSOR                //not interchangeable with SI700x_SENSOR/SI702x_SENSOR, different address

 resolution:
  - HUMD_08BIT_TEMP_12BIT        //RH 8-bit  / T 12-bit
  - HUMD_10BIT_TEMP_13BIT        //RH 10-bit / T 13-bit
  - HUMD_11BIT_TEMP_11BIT        //RH 11-bit / T 11-bit
  - HUMD_12BIT_TEMP_14BIT        //RH 12-bit / T 14-bit
*/

HTU2xD_SHT2x_SI70xx ht2x(HTU2xD_SENSOR, HUMD_11BIT_TEMP_11BIT); //sensor type, resolution

//                     RX    TX
HardwareSerial Serial1(PA10, PA9);

/**************************************************************************/
/*
    setup()

    Main setup
*/
/**************************************************************************/
void setup()
{
  #if defined(ESP8266)
  WiFi.persistent(false);  //disable saving wifi config into SDK flash area
  WiFi.forceSleepBegin();  //disable AP & station by calling "WiFi.mode(WIFI_OFF)" & put modem to sleep
  #endif

  Serial1.begin(115200);
  Serial1.println();
  
  Wire.setSDA(PB9);
  Wire.setSCL(PB8);

  while (ht2x.begin() != true) //reset sensor, set heater off, set resolution, check power (sensor doesn't operate correctly if VDD < +2.25v)
  {
    Serial1.println(F("HTU2xD/SHT2x not connected, fail or VDD < +2.25v")); //(F()) save string to flash & keeps dynamic memory free

    delay(5000);
  }

  Serial1.println(F("HTU2xD/SHT2x OK"));

  //Wire.setClock(400000); //experimental I2C speed! 400KHz, default 100KHz
}


/**************************************************************************/
/*
    loop()

     Main loop
*/
/**************************************************************************/
void loop()
{
  Serial1.println();



  /* read temperature */
  htValue = ht2x.readTemperature();                //accuracy +-0.3C in range 0C..60C at 14-bit

  Serial1.print(F("Temperature...............: "));
  
  if (htValue != HTU2XD_SHT2X_SI70XX_ERROR)        //HTU2XD_SHT2X_SI70XX_ERROR = 255, library returns 255 if error occurs
  {
    Serial1.print(htValue);
    Serial1.println(F(" +-0.3C"));
  }
  else
  {
    Serial1.println(F("<error>"));

    ht2x.softReset();                              //last chance to make it alive, all registers (except heater bit) will set to default
    ht2x.setHeater(false);                         //true=heater on, false=heater off
    ht2x.setResolution(HUMD_12BIT_TEMP_14BIT);     //humidity 12-bit, temperature 14-bit
  }



  /* read compensated humidity */
  delay(500);                                       //measurement with high frequency leads to heating of the sensor, see NOTE

  Serial1.print(F("Compensated Humidity......: "));

  if (htValue != HTU2XD_SHT2X_SI70XX_ERROR)         //if temperature OK, measure RH & calculate compensated humidity
  {
    htValue = ht2x.getCompensatedHumidity(htValue); //accuracy +-2% in range 0%..100%/0C..80C at 12-bit, to compensates influence of T on RH

    if (htValue != HTU2XD_SHT2X_SI70XX_ERROR)
    {
       Serial1.print(htValue);
       Serial1.println(F(" +-2%")); 
    } 
  }
  if (htValue == HTU2XD_SHT2X_SI70XX_ERROR)
  {
    Serial1.println(F("<error>"));
  }



  /* read humidity */
  delay(500);                                      //measurement with high frequency leads to heating of the sensor, see NOTE

  htValue = ht2x.readHumidity();                   //accuracy +-2% in range 20%..80%/25C at 12-bit

  Serial1.print(F("Humidity..................: "));
  
  if (htValue != HTU2XD_SHT2X_SI70XX_ERROR)
  {
    Serial1.print(htValue);
    Serial1.println(F(" +-2%"));
  }
  else
  {
    Serial1.println(F("<error>"));
  }



  /* read power status */
  Serial1.print(F("Power.....................: "));

  if (ht2x.voltageStatus() == true) 
  {
    Serial1.println(F("power OK, VDD > +2.25v"));
  } 
  else
  {
    Serial1.println(F("power FAIL, VDD < +2.25v"));
  }



  /* read device ID */
  Serial1.print(F("Sensor ID.................: "));

  htValue = ht2x.readDeviceID();            //return 21, 7021, 7020, 7013, 7006 & 7000 for Si70xx engineering samples
  
  if (htValue != HTU2XD_SHT2X_SI70XX_ERROR)
  {
    Serial1.println(htValue);
  }
  else
  {
    Serial1.println(F("<error>"));
  }



  /* read FW version */
  Serial1.print(F("Sensor FW.................: "));

  htValue = ht2x.readFirmwareVersion();     //return 1=v1.0, 2=v2.0
  
  if (htValue != HTU2XD_SHT2X_SI70XX_ERROR)
  {
    Serial1.print(F("v"));
    Serial1.println(htValue);
  }
  else
  {
    Serial1.println(F("<error>"));
  }                               



  //ht2x.setHeater(true);                      //true=heater on, false=heater off
  //ht2x.setResolution(HUMD_11BIT_TEMP_11BIT); //change resolution on the fly, RH 11-bit / T 11-bit



  delay(10000); //recomended polling frequency 8sec..30sec
}
