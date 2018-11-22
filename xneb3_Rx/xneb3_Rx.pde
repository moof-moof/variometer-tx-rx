
/**	                                                               xneb3_Rx_62

* Receiver side code for a Digi XBee Pro (900 MHz) radio link from a Tx-ing variometer.
*
* Uses a piezo transducer for variometric audio output with both tone pitch and beep frequency
* proportional to the received value. (This version uses the non-blocking Tone() library.)
*
* Uses a SpeakJet chip to periodically generate synthesized speech altitude reports. 
* If the radio signal is lost, an "attention tweet" reminds us every five seconds.
*
* Variometer specific code by Martin Bergman <bergman.martin at gmail dot com> 2012.
* License: CC BY-SA v3.0 - http://creativecommons.org/licenses/by-sa/3.0/legalcode .
*
* Connections to Arduino I/O board: See schematic "XBee-Piezo-Speakjet_Shield.pdf"
* 
* References: 
* Oxer, J and H Blemings (2009): Practical Arduino - Chapter 9.
* http://www.magnevation.com/software.htm (for Phrase-A-Lator software and dictionary)
*/

//===========[  INCLUDES  ]=====================================================================

#include <SoftwareSerial.h>
#include <Tone.h>
#include "Phrases_LUT.c"

//===========[  DEFINES  ]======================================================================

#define XNEB_VERSION 3
#define RXPIN 2              // Software-serial expects an Rx pin defined, even when not in use.
#define TXPIN 3
#define PIEZOPIN 9
#define MAX_LEVEL 100        // Value must equal FL of highest (last) rung in the "callouts" array.
//#define LAB_LEVEL 10       // Only for simulation of flight levels, where 5 <= FL <= MAX_LEVEL.
//#define DEBUG

//===========[  OBJECTS  ]======================================================================

SoftwareSerial SpJ = SoftwareSerial(RXPIN,TXPIN); // Creates a software serial port object.
Tone Variotone;                                   // Creates an instance of Hagman's Tone library.

//===========[  VARIABLES  ]====================================================================

long startTime = 0;
long lastChecked = 0;
long beepBegan = 0;
byte nilValue = 0;
byte bufr[3] = {0,0,0}; 
byte inByte = 0;
char serialCount = 0;
char rungOld = 0;
int beepCycle = 0;
boolean thisIsIt = false;
const char squelch = 2;        // Sets a threshold lift value to start beeping (units: dm/s).
const int checkAgain = 10000;  // Sets time interval for altitude reports (10 seconds).
const int Ap   = 220;          // Tone pitch mapping range.
const int Op   =1500;          //  "     "      "      "
const int Ad   = 140;          // Tone duration mapping range.  (NON-BLOCKING...)
const int Od   =  60;          // . "      "       "      "
const int Ac   = 300;          // Mapping range of the beep cycle durations.
const int Oc   =  75;          //    "      "    "  "    "    "       "


//===========[  SETUP  ]========================================================================

void setup()
{  
    pinMode(PIEZOPIN, OUTPUT);           // Vario audio out.
    Variotone.begin(PIEZOPIN);           // Initialising the vario audio out pin.
    IDtune(XNEB_VERSION);                // Identify present software version.
    lastChecked = millis();
    Serial.begin(9600);                  // Using the XBee default "Transparent operation" baud.
#ifdef DEBUG
    Serial.println(" ** xneb3_Rx **");
    Serial.println();
    Serial.println("   Ready, Sir!");
    nilValue = LAB_LEVEL;
#endif

    // Configure software-serial port pins
    pinMode(TXPIN, OUTPUT);              // Data out to the SpeakJet IC.
    SpJ.begin(9600);                     // The default is 9600 baud (bits/s).
    
    // SpeakJet initialisation routines
    SpJ.print(20, BYTE);                 // Enter volume set mode.
    SpJ.print(96, BYTE);                 // Set volume (0...127).
    SpJ.print(21, BYTE);                 // Enter speed set mode.
    SpJ.print(110, BYTE);                // Set speed (0...127).
    delay(1000);
}

//===========[  LOOP  ]=========================================================================

void loop()
{
    if(Serial.available() > 3 ){         // We should have received at least four bytes now.
        inByte = Serial.read();          // Read a byte from the serial port,             
        if(inByte == 0xE0){              // ... and if it is indeed a startbyte
            thisIsIt = true;             // ... hoist the Boolean flag and hasten back for more.
            serialCount = 0;
            startTime = millis();
        } 
        else {
            if(thisIsIt){
                bufr[serialCount] = inByte;       // Fill up the three-position buffer array.
                serialCount++;                    // (This needs three loops...)
            }
    	    while (serialCount > 2 ){             // We now have 2 + 1 bytes in the bag.
                if(bufr[2] == 0xE1){              // Final check: Was that the stopbyte?
                    beep_beep((bufr[0]));         // Right, let's hear the appropriate beep.
                    thisIsIt = false;
#ifdef DEBUG
                    Serial.print("\t\t Vv: ");
                    Serial.print(bufr[0],DEC);    // Monitor the vario (vertical velocity) value.
                    Serial.print("  FL: ");
                    Serial.println(bufr[1],DEC);  // Monitor the flight level.
#endif
                    if(millis() - lastChecked > checkAgain){  // Time to speak up again?
                        bufr[1] = bufr[1] + nilValue;  // Injection point for simulated values.
                        Serial.end();             // First we must halt Serial in order to
                        SpJ.begin(9600);          // ... communicate with Software-Serial.
                        delay(100);               //  A slight delay is necessary here
                        speakUp(int(bufr[1]));    // ... before we can speak reliably.
                        Serial.begin(9600);       // And so back to Serial (inhibiting SpeakJet).
                        lastChecked = millis();
                    }
                }
                serialCount = 0;
             }
         }
     } 
     else {
        if(millis() - startTime > 5000){  // If still no signal in the latest five seconds period, 
            startTime = millis();         // ... we restart the timer and
            uh_oh();                      // ... issue an acoustic little show of dissapointment.
        }
    }
}


//===========[  FUNCTIONS  ]====================================================================

void IDtune(char nmbr)
{   
    for (int index = 0; index < nmbr; index++) {
        for (int index2 = 0; index2 < 5; index2++) {
        Variotone.play(500, 50);
        intermezzo(15);
        }
    delay(250);
    }
}


void beep_beep(byte VvVal)
{   
    if (VvVal > squelch){
        if (!(Variotone.isPlaying())){
            if ((millis() - beepBegan) > beepCycle){
                beepBegan = millis();
                int pitch = map(VvVal, 0, 50, Ap, Op);
                int dura =  map(VvVal, 0, 50, Ad, Od);
                Variotone.play(pitch, dura);
                beepCycle = map(VvVal, 0, 50, Ac, Oc);
            } 
        }
    }
}


void speakUp(char pentaLevel)
{
    char lowerLimit = 5;             // Put 5 to set the correct baseline altitude (5*5m).
    if((pentaLevel < lowerLimit) || (pentaLevel > MAX_LEVEL)){ pentaLevel = 0;} // Nullify off-limits.
    if(pentaLevel >= lowerLimit && pentaLevel <= MAX_LEVEL){              // Redundant if expression?
        if(pentaLevel > 40){         // We only want to report even levels above FL 40 (200 m).
            pentaLevel = 40 + ((pentaLevel - 40) / 2);
        }
	char rungNew = pentaLevel - lowerLimit; // Counting begins with "Twentyfive".
        if(rungNew != rungOld){             // "We're in n-th Heaven".
    	    for (int phon = 0; phon < 21; phon++) {
    	        SpJ.print(callouts[rungNew][phon], BYTE);  
    	    }
    	    rungOld = rungNew;
    	}
    	else{
    	    statusQuo();              // A very minute piezo glissando signifying "No change".
    	}
    }
}


void uh_oh()
{
    Variotone.play(250, 30); intermezzo(7);
    Variotone.play(700, 30); intermezzo(7);
    Variotone.play(300, 30); intermezzo(350);
    Variotone.play(200, 30); intermezzo(7);
    Variotone.play(570, 30); intermezzo(7);
    Variotone.play(235, 62);
 
}


void statusQuo()
{
    int hz = 300;
    for (int index = 0; index < 5; index++) {
        Variotone.play(hz, 25); 
        hz = hz * 23/20;
        intermezzo(5);              
    }
}


void intermezzo(int takeFive)
{
    byte adjuster = 3;   // Device to compensate for the evident inherent while()-looping "delay".
                         // (Just use the normal delay values for the takeFive parameter.)
    while(Variotone.isPlaying()){}; // Wait for the non-blocking sound to finish playing,
    delay(takeFive - adjuster);     // ... only then pause awhile for silence.
}

