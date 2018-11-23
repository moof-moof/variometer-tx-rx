#ifndef EWMAcumStiction_h				    				// (version 4)
#define EWMAcumStiction_h

#include <WProgram.h>


class EWMAcumStiction {

public:
    EWMAcumStiction(float alpha, char laxus, char sinkref, char limen);
    void startup(unsigned long startValue, char outputPin);
    int signal(void);				            // returns _vTrueVal
    int read(unsigned long pressure_comp);   	// returns _vTrue
    int _vTrueVal;
	
private:
    unsigned long _typ;			// Typical sensor value to use as an init input,
								//   in order to ease initial EWMA adjustment.
								
    long _smpl;					// Latest sampled value received.
								//   Values's unit represents just 1/10 Pa,(1 dPa,deciPascal).
								
    float _s;					// Intermediate variable for the smoothed moving average.
    
    long _ewma_state;			// The exponentially weighted moving average (EWMA)
								//   state itself. 
								
    long _ewma_last;    		// Value of _ewma_state last time we looped
    
    unsigned long _ewma_last10; // Value of ewma_state10 last time we looped
    
    float _a;  					// Alpha coefficient (a smoothing factor 0 < a < 1) used
								//   for setting the filter's exponential decay. 
								//   Adjust this as needed -- less is more, i e output's lag
								//   and "inertia" increases.
								
    char _slack;				// Constrainment limit for rejecting sensor noise spikes
								//   and other spurious input data. This is a part of the
								//   "stiction" behaviour.
								
    int _catchAll[5]; 			// FIFO array for detection of genuine divergent trends. 
								//   This influences the amount of "stickiness", as well.
								
    int _d;            		 	// Delta of two consecutive ewma_state values. Units are
								//   still dPa here.
								
    int _rw[5];					// Simplistic running window FIFO array for secondary
								//   smoothing of jitter traces. 3 to 5 items in
								//   array are a reasonable compromise, we think.
								
    int _vAppa;         		// (Apparent vector)Average of deltas presently contained in _rw.
    
    int _vTrue;         		// _vAppa offset by _sinkref to compensate for a specific
								//   glider's reference sink rate, thereby reflecting the
								//   actual vertical vector of the surrounding airmass.
								
    char _sinkref; 				// Normal (best) sinkrate. Units are 1 deciPascal (dPa) per
								//   loop [115 dPa ~~ 1 m].
								
    char _piezoPin;				// Hook up the beeper (to digital pin 9 for instance).
    
    char _t;					// Threshold value (+) for signalling climb. (Any sink is
								//   quietly ignored). Units are dPa.
};
#endif


