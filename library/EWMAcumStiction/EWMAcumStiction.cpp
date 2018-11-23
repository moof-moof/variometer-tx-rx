#include "EWMAcumStiction.h"				    // (version 4)


EWMAcumStiction::EWMAcumStiction(float alpha, char laxus, char sinkref, char limen)
{
    _a		    = 	alpha;
    _slack 	    = 	laxus;
    _sinkref	=	sinkref;
    _t		    = 	limen;
    
/** * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * The sensitivity of the variometer algorithm, in terms of response lag
 * and noise rejection, may be tweaked with the constants alpha, laxus and limen.
 * Constant sinkref sets the desired sinkrate nominal value.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
}

void EWMAcumStiction::startup(unsigned long startValue, char outputPin) 
{
    _typ = startValue;
    _s = _typ;	
    _ewma_state =_typ;

    _catchAll[5] = 0, 0, 0, 0, 0;
    _rw[5] = 0, 0, 0, 0, 0;

    _piezoPin = outputPin;
    pinMode(_piezoPin, OUTPUT);                 //  For optional audio in vario/tx unit
    
    for (int index = 0; index < 5; index++) {   // Start-up ditty
		tone(_piezoPin, 500, 50); 
		delay(75);
    }
    
    Serial.print("\n");
    Serial.print("   (StartValue: ");
    Serial.print(startValue, DEC);
    Serial.println(" dPa)");
    Serial.println("___________________________________________");
    Serial.print("\n");
}

   
int EWMAcumStiction::signal()
{    
	if(_vTrue >= _t) {	        // Definitely rising now?
	    if ( _vTrue > 50 ) {    // Oh, stop this horrible squeaking!
	    	 _vTrueVal = 50; 
	    } 
	    else {
		  _vTrueVal = _vTrue;
	    } 
	} 
	else {
		_vTrueVal = 0;	        // Since we're not really rising
	}   
    return _vTrueVal;           // Now beep (or rather broadcast data) like a bat!
}


int EWMAcumStiction::read(unsigned long pressure_comp)
{
    _smpl = pressure_comp;

    _catchAll[0] = _catchAll[1];                // Nudge the array one tick...
    _catchAll[1] = _catchAll[2];
    _catchAll[2] = _catchAll[3];
    _catchAll[3] = _catchAll[4];
    
    
    if (abs(_smpl -_sinkref -_ewma_state) > _slack){// Is the latest value off limits?
		if ((_smpl -_sinkref) > _ewma_state) {      // If so: Is it higher?
			_catchAll[4] = 1;                	    // -- score it as such.
		} else {                             	    // Or perhaps lower?
			_catchAll[4] = -1;               	    // -- score it as such.
		}
    } else {                             	        // Nah, it's another middleoftheroader
		_catchAll[4] = 0;                    	    // -- no score.
    }

    if (abs(_catchAll[0]+_catchAll[1]+_catchAll[2]+_catchAll[3]+_catchAll[4]) != 5) {
// Do the recursive filter EWMA routine, using a possibly constrained value:
		_smpl = constrain(_smpl, _ewma_state - _slack, _ewma_state + _slack);
		_s = _smpl * _a +_s * (1 - _a);   
		_ewma_state = ( long) _s;

    } else {
// We may have struck a trend here! Update EWMA with the full latest divergent value
// (This is the equivalent of a first order low-pass filter) :
		_s = _smpl * _a + _s * (1 - _a);   
		_ewma_state = ( long) _s;         
    }

    _d     = _ewma_state - _ewma_last;      // Get the latest pressure delta
    _rw[0] = _rw[1];                        // Nudge-nudge (know what I mean?)
    _rw[1] = _rw[2];
    _rw[2] = _rw[3];
    _rw[3] = _rw[4];
    _rw[4] = _d;
    
// A final rubbing down of any remaining roughness:   
    _vAppa = (_rw[0]+_rw[1]+_rw[2]+_rw[3]+_rw[4])/5; // Not very elegant. So what?
    _vAppa = -1 * _vAppa;      	            // Sign reversal: negative deltas == up we go!
    _vTrue = _vAppa + _sinkref;             // Bias vAppa with the net vertical velocity.
    
    _ewma_last   = _ewma_state;
    
    return _vTrue;
}
