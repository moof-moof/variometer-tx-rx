/**                                                                      xneb3_Tx_42
*                                                                        
*  Transmitter side code for remote variometer telemetry.
*
*  Reads a Bosch Sensortec's BMP085 athmospheric pressure sensor and broadcasts a
*  variometric signal, based on the computed current (positive) vertical velocity. 
*  Uses a Digi XBee Pro 900 radio module set to "Transparent Operation" mode.
*
*  Variometer specific code by Martin Bergman <bergman.martin at gmail dot com> 2011.
*  License: CC BY-SA v3.0 - http://creativecommons.org/licenses/by-sa/3.0/legalcode .
*
*  Sensor connections to Arduino I/O board:
*   Vcc -> 3V3
*   GND -> GND
*   SCL -> A5
*   SDA -> A4
*
*  XBee 900 connections to Arduino I/O board:
*   Vcc -> 3V3
*   GND -> GND
*   DIN -> D1 (UART Tx)
*  
*  Cp. the schematic "Variometer_with_Xbee_Tx__rev06.pdf".
*
*  References: 
*   http://www.bosch-sensortec.com/content/language1/downloads/BST-BMP085-DS000-06.pdf
*   http://interactive-matter.org/2009/12/arduino-barometric-pressure-sensor-bmp085/
*   http://news.jeelabs.org/2009/02/19/hooking-up-a-bmp085-sensor/
*   http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf
*/

//===========[  INCLUSIONS  ]===================================================================

#include <Wire.h>
#include <EWMAcumStiction.h>

//===========[  DEFINITIONS  ]==================================================================

#define PIEZOPIN 9
#define RAWPIN 0 
#define BMP_ADDRESS 0x77  // 1110111b (119)
#define STARTBYTE 0xE0    // 11100000b (224)
#define STOPBYTE 0xE1     // 11100001b (225)
//#define DEBUG
/* Please note that the amount of auxiliary Serial.print-ing performed while in DEBUG mode
    can be expected to interfere with (i.e. corrupt) the genuine transmission, and hence that
    it then becomes "challenging" to simultaneously parse messages from the receiver side.
    If you get inexplicable garbage, this may be the reason.  */
//#define DEBUG2

//===========[  OBJECTS  ]======================================================================

/* We first create a filter object that smoothes and conditions the pressure sensor's output.
   The suggested ranges of the filter's required parameters are shown in brackets [like so]:
        alpha (float):  A smoothing coefficient, such as 0 < alpha < 1           [0.04-0.1]
        laxus (char):   Maximal amount of +/-"spread" in a legitimate sensor value    [2-5]
        sinkref (char): Nominal still-air sinkrate (see header file for explanation)  [1-4]
        limen (char):   Threshold value for signalling climb                          [< 1]  */
EWMAcumStiction vario(0.08, 4, 2, 1);
	

//===========[  GLOBAL VARIABLES & CONSTANTS  ]=================================================

const unsigned char oversampling_setting = 3;     // OSS can be either 0, 1, 2 or 3
const unsigned char pressure_waittime[4] = { 5, 8, 14, 26 }; // corresponding four values
const int loopingtime = 84; // 1 dPa @ 11.9 Hz => 8.4 mm * 11.9 = 100 mm/sek (0.1 m/s)

// Sensor's calibration values (to be retrieved from its EEPROM):
int ac1, ac2, ac3, b1, b2, mb, mc, md;
unsigned int ac4, ac5, ac6;
int temperature = 0;
long pressure = 0;    // the temperature-compensated, "true" pressure of a sample (Pa units)

unsigned long previousMillis = 0;
long Pgnd = 0;        // The registered athmospheric pressure at ground level (deciPascal units)
long deciPasc = 0;  
byte msg[2] = {0,0};  // The two-part message array to be transmitted
int multip_5m;
boolean onoff;
int countDeLoop = 0;  // Debugging variable for checking the looping speed
byte blinker = 0;
byte dVolts = 0;      // Scaled supply voltage reading

//===========[  FUNCTION PROTOTYPES  ]==========================================================

void bmp085_read_temperature_and_pressure(int& temperature, long& pressure);

//===========[  SETUP  ]========================================================================

void setup()
{
    Serial.begin(9600);        
    Wire.begin();
    pinMode(PIEZOPIN, OUTPUT); // On-board audio out (optional).
    pinMode(13, OUTPUT);       // Tx activity LED
    pinMode(RAWPIN, INPUT);    // Pin for sensing supply voltage
    
// Battery status is checked before we begin:
    check_battery();
    play_voltage(dVolts); 
    bmp085_get_cal_data();
    
#ifdef DEBUG
    print_battery_status(dVolts);
    print_info();
    print_cal_data();
#endif

// Set the 'Pgnd' value (Pa) for the session:
    register_ground_pressure();
    
// Attach an init value (dPa) and an output pin to the 'vario' filter object:
    vario.startup((Pgnd * 10), PIEZOPIN);
}

//===========[  LOOP  ]=========================================================================

void loop()
{
    unsigned long currentMillis = millis();
    if(currentMillis - previousMillis >= loopingtime) {
        byte spentTime = currentMillis - previousMillis;
        previousMillis = currentMillis;
        
        bmp085_read_temperature_and_pressure(&temperature,&pressure);
        deciPasc = pressure * 10;  // pressure unit is Pa
        vario.read(deciPasc);      // units for the deciPasc variable are dPa
        msg[0] = vario.signal();
        msg[1] = altimeter(pressure);
	
#ifdef DEBUG
        onboardbeep(int(msg[0])); 
//        char diff = Pgnd - pressure; // diff is the unfiltered sample offset (Pa)
//        Serial.println(); Serial.print("\n Vv (dPa/loop): "); 
        Serial.print(msg[0], DEC);
//        Serial.print("\t Press (Pa): "); Serial.print(pressure, DEC);
//        Serial.print("\t diff (Pa): "); Serial.print(diff, DEC);
        Serial.print("\t alt (@ 5m): "); Serial.print(msg[1], DEC);  
        Serial.print("\t ms: "); Serial.println(spentTime, DEC);
#endif
#ifdef DEBUG2
        countDeLoop++;
        if(countDeLoop > 500){
            tone(PIEZOPIN, 250, 30); 
            countDeLoop = 0;
        }
#endif        
        Serial.write(STARTBYTE);        // 224
        Serial.write(msg[0]);           // 0 ... 50 (dPa)
        Serial.write(msg[1]);           // Much less than level 223 (1115 m) presumably!
        Serial.write(STOPBYTE);         // 225
        if(blinker > 4 && blinker < 10){
            digitalWrite(13, LOW);
        } else {
            digitalWrite(13, HIGH);     // Indicate activity
        }
        blinker++;
        if(blinker > 9){blinker = 0;}
    }
}

//===========[  FUNCTIONS  ]====================================================================


void print_info() 
{ 
    Serial.flush();
    Serial.println();
    Serial.println("    Setting up the BMP085 sensor");
    Serial.println("*******************************************");
    Serial.println();
    Serial.print("(1) Oversampling setting: ");
    Serial.print(oversampling_setting, DEC);
    Serial.print("  (");
    byte overSampling = pow(2,(oversampling_setting));
    Serial.print(overSampling, DEC);
    Serial.println(" samples)");
    Serial.print("    Looping time is: ");
    Serial.print(loopingtime,DEC);
    Serial.print(" ms  (");
    Serial.print(100000/loopingtime,DEC); // 100 secs increases resolution despite int
    Serial.println(" centiHz)");
    Serial.println();
}


void print_battery_status(byte dV)
{ 
    Serial.print(">>>>>> RAW: ");
    Serial.print(dV, DEC);
    Serial.println(" dV");
}


void register_ground_pressure() 
{
    int temperature = 0;
    long pressure = 0;     // the temperature-compensated, "true" pressure of a sample (Pa units)
    long totalTempoPcomp = 0;        // sum of temporary compensated pressure values (Pa units)

    tone(PIEZOPIN, 150, 150); delay(100);   // "Set-up starts" jingle... THIS ASSUMES A SPEAKER!
    tone(PIEZOPIN, 250, 150); delay(100); 
    tone(PIEZOPIN, 400, 300);
  
    for (int index = 0; index < 50; index++) {
      bmp085_read_temperature_and_pressure(&temperature,&pressure);
      totalTempoPcomp += pressure;               // add up 50 compensated sample values
      delay(50);
    }        
    Pgnd = totalTempoPcomp / 50;  // we keep the sampled average expressed as Pascals (Pa)
  
    tone(PIEZOPIN, 400, 150); delay(100);   // "Set-up finished" jingle
    tone(PIEZOPIN, 250, 150); delay(100); 
    tone(PIEZOPIN, 150, 300);
#ifdef DEBUG  
    Serial.println("*******************************************");
    Serial.print("   Pressure at ground level is ");
    Serial.print(Pgnd, DEC);
    Serial.println(" Pa");
    Serial.println("*******************************************");
    Serial.println();
    Serial.print("(4)");delay(1000);Serial.println(" Let's go!"); 
    Serial.println();
    Serial.println("___________________________________________");
#endif
    delay(500);
} 


int altimeter(long Pps)       // Calculates altitude based on the pressure difference
{
    if (Pgnd > Pps){
        multip_5m = (44330 * (1 - pow(((float)Pps / Pgnd), 0.190295)) / 5);
    }
    return multip_5m;
}


void check_battery()
{
    int adcValue = analogRead(RAWPIN);
    delay(100);
    dVolts = adcValue * 2 / 31; // Vcc 3.3 volts == 1024 @ 10 bits resolution,
    // ... so 0.1V == 31  (1 binary unit == 3.22 mV).
    // Using a 1/2 voltage divider (10k+10k): 0.1V (1 dVbatt) == 15.5, 
    // ... so adcValue * 2 / 31 == X dVbatt
}


void play_voltage(byte dV)
{
    if(dV < 35 && dV > 1){alarum();}
    if(dV == 35){deciVoltage(1,0);}     
    if(dV == 36){deciVoltage(2,0);}     
    if(dV == 37){deciVoltage(3,0);}     
    if(dV == 38){deciVoltage(4,0);}     
    if(dV == 39){deciVoltage(4,1);}
    if(dV == 40){deciVoltage(4,2);}
    if(dV == 41){deciVoltage(4,3);}
    if(dV >= 42){deciVoltage(4,4);}
}


void deciVoltage(char critical, char safe)
{
    for(char i = 0; i < critical; i ++){
        criticalBeep();
    }
    for(char i = 0; i < safe; i ++){
        safeBeep();
    }
    delay(500);
}


void alarum()
{
    for(char i=0; i<5; i++){
        slideTweet();
    }
}


void criticalBeep()
{
    tone(PIEZOPIN, 220, 300);         // Longish beep
    delay(1200);
}


void safeBeep()
{
    tone(PIEZOPIN, 440, 150);         // Short beep
    delay(1200);
}
    
    
void slideTweet()                     // Terrifying beep!
{
    delay(100);
    tone(PIEZOPIN, 220, 150); delay(30);
    tone(PIEZOPIN, 245, 150); delay(30);
    tone(PIEZOPIN, 262, 150); delay(30);
    tone(PIEZOPIN, 294, 150); delay(30);
    tone(PIEZOPIN, 330, 150); delay(30);
    tone(PIEZOPIN, 349, 150); delay(30);
    tone(PIEZOPIN, 392, 150); delay(30); 
    tone(PIEZOPIN, 440, 150); delay(50);
    tone(PIEZOPIN, 392, 150); delay(30); 
    tone(PIEZOPIN, 349, 150); delay(30);
    tone(PIEZOPIN, 330, 150); delay(30);
    tone(PIEZOPIN, 294, 150); delay(30); 
    tone(PIEZOPIN, 262, 150); delay(30); 
    tone(PIEZOPIN, 245, 150); delay(30);
    tone(PIEZOPIN, 220, 150); delay(30);
    tone(PIEZOPIN, 196, 150); delay(30); 
    tone(PIEZOPIN, 175, 150); delay(30); 
    tone(PIEZOPIN, 165, 154); delay(29);
    tone(PIEZOPIN, 147, 163); delay(25);
    tone(PIEZOPIN, 131, 170); delay(20);   
    tone(PIEZOPIN, 124, 180); delay(20);
    tone(PIEZOPIN, 110, 192);
}


void onboardbeep(int vVal)
{
    if (onoff == true){
        if (vVal > 2){
            int pitch = map((vVal), 1, 50, 100, 2500);
            tone(PIEZOPIN, pitch, 45); // Generate a suitably high-pitched noise when ascending .
            onoff = false;
        }
        onoff = false;
    } else {
	onoff = true;
    }
}


/** ********** This is the Bosch API main computation, using integer math only: *************/


void bmp085_read_temperature_and_pressure(int* temperature, long* pressure) 
{
  int ut= bmp085_read_ut();
  long up = bmp085_read_up();
  long x1, x2, x3, b3, b5, b6, p;
  unsigned long b4, b7;

  // Calculate the temperature
  x1 = ((long)ut - ac6) * ac5 >> 15;
  x2 = ((long) mc << 11) / (x1 + md);
  b5 = x1 + x2;
  *temperature = (b5 + 8) >> 4;

  // Calculate the pressure
  b6 = b5 - 4000;
  x1 = (b2 * (b6 * b6 >> 12)) >> 11;
  x2 = ac2 * b6 >> 11;
  x3 = x1 + x2;
  b3 = (((((long) ac1) * 4 + x3)<<oversampling_setting) + 2) >> 2;
  x1 = ac3 * b6 >> 13;
  x2 = (b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (unsigned long) (x3 + 32768)) >> 15;
  b7 = ((unsigned long) up - b3) * (50000 >> oversampling_setting);
  p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  *pressure = p + ((x1 + x2 + 3791) >> 4);
}


void bmp085_get_cal_data() 
{
  ac1 = read_int_register(0xAA);
  ac2 = read_int_register(0xAC);
  ac3 = read_int_register(0xAE);
  ac4 = read_int_register(0xB0);
  ac5 = read_int_register(0xB2);
  ac6 = read_int_register(0xB4);
  b1 = read_int_register(0xB6);
  b2 = read_int_register(0xB8);
  mb = read_int_register(0xBA);
  mc = read_int_register(0xBC);
  md = read_int_register(0xBE);
}


void print_cal_data() 
{    // These printlines are for debugging mainly:    
  delay(2000);Serial.println("(2) Now retrieving calibration data...");
  delay(500);Serial.println();
  Serial.print("AC1: ");Serial.println(ac1,DEC);
  Serial.print("AC2: ");Serial.println(ac2,DEC);
  Serial.print("AC3: ");Serial.println(ac3,DEC);
  Serial.print("AC4: ");Serial.println(ac4,DEC);
  Serial.print("AC5: ");Serial.println(ac5,DEC);
  Serial.print("AC6: ");Serial.println(ac6,DEC);
  Serial.print("B1: ");Serial.println(b1,DEC);
  Serial.print("B2: ");Serial.println(b2,DEC);
  Serial.print("MB: ");Serial.println(mb,DEC);
  Serial.print("MC: ");Serial.println(mc,DEC); 
  Serial.print("MD: ");Serial.println(md,DEC); 
  Serial.println();
  delay(500);
  Serial.println("(3) Calibrating for the present conditions... ");
  Serial.println();
}

//=========[  Various I2C (TWI) helper functions  ]=============================================

unsigned int bmp085_read_ut() 
{
  write_register(0xf4,0x2e);
  delay(5); // longer than 4.5 ms
  return read_int_register(0xf6);
}


long bmp085_read_up() 
{
  write_register(0xf4,0x34+(oversampling_setting<<6));
  delay(pressure_waittime[oversampling_setting]);
  unsigned char msb, lsb, xlsb;
  Wire.beginTransmission(BMP_ADDRESS);
  Wire.send(0xf6);                  // register to read
  Wire.endTransmission();

  Wire.requestFrom(BMP_ADDRESS, 3); // read a byte
  while(!Wire.available()) {}       // waiting
  msb = Wire.receive();
  while(!Wire.available()) {}       // waiting
  lsb |= Wire.receive();
  while(!Wire.available()) {}       // waiting
  xlsb |= Wire.receive();
  return (((long)msb<<16) | ((long)lsb<<8) | ((long)xlsb)) >>(8-oversampling_setting);
}


void write_register(unsigned char r, unsigned char v) 
{
  Wire.beginTransmission(BMP_ADDRESS);
  Wire.send(r);
  Wire.send(v);
  Wire.endTransmission();
}


int read_int_register(unsigned char r) 
{
  unsigned char msb, lsb;
  Wire.beginTransmission(BMP_ADDRESS);
  Wire.send(r); 		    // register to read
  Wire.endTransmission();

  Wire.requestFrom(BMP_ADDRESS, 2); // read a byte
  while(!Wire.available()) {}       // waiting
  msb = Wire.receive();
  while(!Wire.available()) {}       // waiting
  lsb = Wire.receive();
  return (((int)msb<<8) | ((int)lsb));
}
