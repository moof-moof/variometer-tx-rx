/*

By Martin Bergman
*/

#include <EWMAcumStiction.h> 

 
/*  Create a filter object to smooth and analyze the pressure sensor's output. 
    Required parameters are: alpha, laxus, sinkref, limen
	
alpha (float): 	Smoothing coefficient, such as 0 < alpha < 1 [suggestion 0.04 - 0.1]
laxus (int): 	Permitted amount of +/-"spread" in sensor values [suggestion 2-5]
sinkref (int): 	Normal sinkrate (see header file for explanation) [suggestion 2-4]
limen (int):	Threshold value for signalling climb [suggestion 4-6]              

See the header file "EWMAcumStiction.h" for more details.			*/

EWMAcumStiction myvario(0.7, 4, 3, 5); 


void setup() 
{ 
    myvario.startup(9);  // attaches the piezo on pin 9 to the "myvario" object 
    Serial.begin(9600); // For monitoring
} 
 
 
void loop() 
{

    volatile unsigned long sample = getPressureSampleSomehow();
    myvario.read(sample);
    myvario.signal();

} 

int getPressureSampleSomehow(){
    return 0;
}
