// Simple presure based variometer, using a bmp180 to drive an audio and a stepper needle.
// Copyright Niv Levy July 11th 2015 and later
// released under GPL v2

// originally based on http://www.rcgroups.com/forums/showthread.php?t=1749208  ( Rolf R Bakke, Oct 2012), but zero remains.

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Stepper.h>
#include <TimerOne.h>

#define speaker_PIN 10

// change this to the number of steps on your motor
#define STEPS 100
    
// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to
Stepper stepper(STEPS, 4, 5, 6, 7);
    
// i'm going to do the filtering not so nicely, by just implementing a ring buffer, storing deltas and going from there. Since bmp180 seems to get about 30Hz, and i'd like at most 0.5sec, we'll start with 20 samples.
// we should at least use a non square window, TODO
#define N_samples 75 // 25sampls/sec and a 1.5sec time constant : 37.5 samples x 2  = 75

const int dT = 40; // time between samples, msec

const float ddsAcc_limit = dT * 2;
const float sound_threshold = ddsAcc_limit / 2;

// in m/sec (or something - the time unit is less certain here!
const float neg_dead_band = -0.3F;
const float pos_dead_band = 0.3F;

float H[N_samples];
float sample_times[N_samples];
float delta_times[N_samples];
float lrCoef[2];

byte counter = 0;
bool grab_data = false;
long since_last;

elapsedMillis t = 0;

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(BMP085_MODE_ULTRAHIGHRES);

void time_isr(void) {
    since_last = long(t);
    t = 0;
    grab_data = true;
}



unsigned long time = 0, last_time = 0;

float toneFreq, pressure, altitude, old_altitude ;

short ddsAcc; // int is 4 bytes on the later arduino and teensy!

// borrowed from http://jwbrooks.blogspot.com/2014/02/arduino-linear-regression-function.html
void simpLinReg(float* x, float* y, float* lrCoef, int n){
    // pass x and y arrays (pointers), lrCoef pointer, and n.  The lrCoef array is comprised of the slope=lrCoef[0] and intercept=lrCoef[1].  n is length of the x and y arrays.
    // http://en.wikipedia.org/wiki/Simple_linear_regression

    // initialize variables
    float xbar=0;
    float ybar=0;
    float xybar=0;
    float xsqbar=0;
    
    // calculations required for linear regression
    for (int i=0; i<n; i++){
	xbar=xbar+x[i];
	ybar=ybar+y[i];
	xybar=xybar+x[i]*y[i];
	xsqbar=xsqbar+x[i]*x[i];
    }
    xbar=xbar/n;
    ybar=ybar/n;
    xybar=xybar/n;
    xsqbar=xsqbar/n;
    
    // simple linear regression algorithm
    lrCoef[0]=(xybar-xbar*ybar)/(xsqbar-xbar*xbar);
    lrCoef[1]=ybar-lrCoef[0]*xbar;
}

//lowpass filter bessel, 2nd order, corner = 0.04 (normalized - 1Hz with a 25Hz sampling), as calculated using http://www.schwietering.com/jayduino/filtuino/index.php?characteristic=be&passmode=lp&order=2&usesr=usesr&sr=25&frequencyLow=1&noteLow=&noteHigh=&pw=pw&calctype=float&run=Send

class filter
{
	public:
		filter()
		{
			v[0]=0.0;
			v[1]=0.0;
		}
	private:
		float v[3];
	public:
		float step(float x) //class II 
		{
			v[0] = v[1];
			v[1] = v[2];
			v[2] = (1.980014076209e-2 * x)
				 + ( -0.5731643146 * v[0])
				 + (  1.4939637515 * v[1]);
			return 
				 (v[0] + v[2])
				+2 * v[1];
		}
};

filter lowpass;

void setup()
{
    int i;
    /* Initialise the sensor */
    if(!bmp.begin())
    {
	/* There was a problem detecting the BMP085 ... check your connections */
	Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
	while(1);
    }
    Serial.begin(115200); // note that number i BS for teensy.
    // just a place to start - should be actual pressure, but whatever
    //initialize pressure
    bmp.getPressure(&pressure);
    old_altitude = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, pressure / 100.0F);
    
    for (i =0; i < N_samples; i++) {
	H[i] = old_altitude;
	sample_times[i] = 0.0;
    }
    
    stepper.setSpeed(60);
    // try to get it to center
    // uncomment this if necessary to set - but it does bang the motor against the stop!
//     stepper.step(-600);
//     stepper.step(300);
    
    Timer1.initialize(dT * 1000);
    Timer1.attachInterrupt(time_isr);
    
    Serial.println("start");
}


void loop()
{
    int i;
    long since_last_copy;
    float climb_rate, climb_rate_filter;

    if (grab_data) {
	noInterrupts();
	since_last_copy = since_last;
	grab_data = false;
	interrupts();
	
	bmp.getPressure(&pressure);
	H[counter % N_samples] = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, pressure / 100.0F);
	sample_times[counter % N_samples] = float(millis());
	for (i=0; i<N_samples ; i++)
	    delta_times[i] = sample_times[i] - sample_times[0];
	// KLUDGE - i should probably allocate mem properly etc
	simpLinReg(&(delta_times[0]), &(H[0]), &(lrCoef[0]), N_samples);
	
	// NOTE : the filter here is for a fixed 25Hz sampling rate!
	if (counter > 0)
	    climb_rate_filter = lowpass.step(H[counter % N_samples] - H[(counter-1) % N_samples]);
	else // KLUDGE
	    climb_rate_filter = lowpass.step(H[0] - H[N_samples-1]);
	
	//climb_rate = lrCoef[0] * 1000; // in m/sec
	climb_rate = climb_rate_filter;
	
	// TODO: need to deal with residual, otherwise the needle drifts (since it's not wite noise)
	stepper.step(-int(climb_rate/ 1));

	// NOTE: can do non linear function here, e.g. logarithmic
	toneFreq = constrain(climb_rate * 4, -500, 500);
	
	// copied from my mkiv audio code
	if (toneFreq > 0)
	    toneFreq += 150 ; 

	ddsAcc += int(climb_rate);
	if (ddsAcc > ddsAcc_limit) 
	    ddsAcc = 0;
	if (ddsAcc < 0)
	    ddsAcc = 0;

	Serial.print("i= ");
	Serial.print(counter);
	Serial.print(", dt (ms)= ");
	Serial.print(since_last_copy);
    //     Serial.print(", P= ");
    //     Serial.print(pressure);
	Serial.print(", Alt= ");
	Serial.print(H[counter % N_samples]);
	Serial.print("m , lrCoef: ");
	Serial.print(lrCoef[0] * 1000);
	Serial.print(" m/sec,");
	Serial.print(lrCoef[1]);
	Serial.print(" m, f= ");
	Serial.print(toneFreq + 510);
	Serial.print(", ddsAcc = ");
	Serial.print(ddsAcc);
	Serial.print(", climb_rate_filter = ");
	Serial.print(climb_rate_filter);
	Serial.println(" m/s");
	
	// using the linear regression instead of my stupidity
	if (climb_rate < neg_dead_band || ( climb_rate > pos_dead_band && ddsAcc > sound_threshold))
	    tone(speaker_PIN, toneFreq + 510);
	else
	    noTone(speaker_PIN);
	counter++;
    }
}